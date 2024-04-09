#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "TASK_read_temp.h"
#include "TASK_modbus_control.h"

// #include "TASK_HMI_Serial.h"

String local_IP;

char ap_name[30];
uint8_t macAddr[6];
extern double BT_TEMP;


pid_setting_t pid_parm = {
    2000, // uint16_t pid_CT;
    2.0,  // double p ;
    0.12, // double i ;
    5.0,  // double d ;
    0.0,  // uint16_t BT_tempfix;
    0.0   // uint16_t ET_tempfix;
};

void setup()
{

    xThermoDataMutex = xSemaphoreCreateMutex();
    xSerialReadBufferMutex = xSemaphoreCreateMutex();

    pinMode(SYSTEM_RLY, OUTPUT);
    pinMode(FAN_RLY, OUTPUT);
    pinMode(HEAT_RLY, OUTPUT);

    digitalWrite(SYSTEM_RLY, LOW); // 初始化电路启动；
    digitalWrite(FAN_RLY, LOW);    // 初始化电路启动；
    digitalWrite(HEAT_RLY, LOW);   // 初始化电路启动；


    // read pid data from EEPROM
    EEPROM.begin(sizeof(pid_parm));
    EEPROM.get(0, pid_parm);


    Serial.begin(BAUDRATE); // for MODBUS TCP debug

    // Serial_HMI.begin(BAUD_HMI, SERIAL_8N1, HMI_RX, HMI_TX);

#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nSerial Started");
#endif

    // INIT SENSOR
    thermo_INLET.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary
    thermo_BT.begin(MAX31865_2WIRE);    // set to 2WIRE or 4WIRE as necessary
#if defined(MODEL_M6S)
    thermo_ET.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary
#endif

    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // Serial_debug.println("WiFi.mode(AP):");
    WiFi.mode(WIFI_AP);
    sprintf(ap_name, "HB-%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    if (WiFi.softAP(ap_name, "12345678"))
    { // defualt IP address :192.168.4.1 password min 8 digis
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
        Serial.printf("\nWiFi AP: %s Started\n", ap_name);
#endif
    }
    else
    {
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
        Serial.printf("\nWiFi AP NOT OK YET...\n");
#endif
        vTaskDelay(500);
    }

    // Init pwm output
    pwm_heat.pause();
    pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
    pwm_heat.resume();
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    pwm_heat.printDebug();
    Serial.println("\nPWM started");
#endif

    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        Task_Thermo_get_data, "Thermo_get_data" //
        ,
        1024 * 8 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 5 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nTASK=1:Thermo_get_data OK");
#endif

    xTaskCreatePinnedToCore(
        Task_modbus_control, "modbus_control" //
        ,
        1024 * 10 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTask_modbus_control, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nTASK=2:modbus_control OK");
#endif

    xTaskCreatePinnedToCore(
        Task_PID_autotune, "PID autotune" //
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 4 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTask_PID_autotune, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nTASK=3:PID autotune OK");
#endif

    //     xTaskCreatePinnedToCore(
    //         TASK_data_to_HMI, "data_to_HMI" //
    //         ,
    //         1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         &xTASK_data_to_HMI, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    //     Serial.printf("\nTASK=3:data_to_HMI OK");
    // #endif

    //     xTaskCreatePinnedToCore(
    //         TASK_CMD_FROM_HMI, "TCMD_FROM_HMI" //
    //         ,
    //         1024 * 2 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         &xTASK_CMD_HMI, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    //     Serial.printf("\nTASK=4:CMD_FROM_HMI OK");
    // #endif

    //     xTaskCreatePinnedToCore(
    //         TASK_HMI_CMD_handle, "handle_CMD_FROM_HMI" //
    //         ,
    //         1024 * 8 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         &xTASK_HMI_CMD_handle, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    //     Serial.printf("\nTASK=5:handle_CMD_FROM_HMI OK");
    // #endif

    // INIT MODBUS

    mb.server(502); // Start Modbus IP //default port :502

#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nStart Modbus-TCP  service OK\n");
#endif
    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    mb.addHreg(BT_HREG);
    mb.addHreg(ET_HREG);
    mb.addHreg(INLET_HREG);
    mb.addHreg(EXHAUST_HREG);

    mb.addHreg(HEAT_HREG);
    mb.addHreg(FAN_HREG);
    mb.addHreg(PWR_HREG);

    mb.addHreg(PID_SV_HREG);
    mb.addHreg(PID_STATUS_HREG);

    // INIT MODBUS HREG VALUE
    mb.Hreg(BT_HREG, 0);      // 初始化赋值
    mb.Hreg(ET_HREG, 0);      // 初始化赋值
    mb.Hreg(INLET_HREG, 0);   // 初始化赋值
    mb.Hreg(EXHAUST_HREG, 0); // 初始化赋值

    mb.Hreg(HEAT_HREG, 0); // 初始化赋值
    mb.Hreg(FAN_HREG, 0);  // 初始化赋值
    mb.Hreg(PWR_HREG, 0);  // 初始化赋值

    mb.Hreg(PID_SV_HREG, 0);     // 初始化赋值
    mb.Hreg(PID_STATUS_HREG, 0); // 初始化赋值

    // init PID
    Heat_pid_controller.begin(&BT_TEMP, &PID_output, &pid_sv, pid_parm.p, pid_parm.i, pid_parm.d);
    Heat_pid_controller.setSampleTime(pid_parm.pid_CT); // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
    Heat_pid_controller.setOutputLimits(0,255); //取值范围（0-255）
    Heat_pid_controller.setBias(255.0 / 2.0);
    Heat_pid_controller.setWindUpLimits(-3, 3); // Groth bounds for the integral term to prevent integral wind-up
    Heat_pid_controller.start();

    // INIT PID AUTOTUNE

    tuner.setTargetInputValue(180.0);
    tuner.setLoopInterval(pid_parm.pid_CT);
    tuner.setOutputRange(map(pid_out_min,0,100,0,255), map(pid_out_max,0,100,0,255));//取值范围转换为（0-255）
    tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

    ////////////////////////////////////////////////////////////////
    vTaskDelay(3000);
    digitalWrite(SYSTEM_RLY, HIGH); // 启动机器
    
}

void loop()
{
    mb.task();
}