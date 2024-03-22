#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "TASK_read_temp.h"
#include "TASK_data_to_HMI.h"
#include "TASK_modbus_control.h"
#include "TASK_CMD_from_HMI_handle.h"

String local_IP;

char ap_name[30];
uint8_t macAddr[6];

void setup()
{

    xThermoDataMutex = xSemaphoreCreateMutex();
    //xSerialReadBufferMutex = xSemaphoreCreateMutex();

    pinMode(SYSTEM_RLY, OUTPUT);
    pinMode(FAN_RLY, OUTPUT);
    pinMode(HEAT_RLY, OUTPUT);

    digitalWrite(SYSTEM_RLY, LOW); // 初始化电路启动；
    digitalWrite(FAN_RLY, LOW);    // 初始化电路启动；
    digitalWrite(HEAT_RLY, LOW);   // 初始化电路启动；

    Serial.begin(BAUDRATE); // for MODBUS TCP debug

    Serial_HMI.begin(BAUD_HMI, SERIAL_8N1, HMI_RX, HMI_TX);

    // #if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    //     Serial.printf("\nSerial Started");
    // #endif

    // INIT SENSOR
    thermo_INLET.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary
    thermo_BT.begin(MAX31865_2WIRE);    // set to 2WIRE or 4WIRE as necessary
#if defined(MODEL_M6S)
    thermo_ET.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary
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
        TASK_data_to_HMI, "data_to_HMI" //
        ,
        1024 * 4 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nTASK=2:data_to_HMI OK");
#endif

    xTaskCreatePinnedToCore(
        Task_modbus_control, "modbus_control" //
        ,
        1024 * 8 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nTASK=3:modbus_control OK");
#endif

    xTaskCreatePinnedToCore(
        TASK_CMD_From_HMI, "CMD_From_HMI" //
        ,
        1024 * 8 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 4 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    Serial.printf("\nTASK=4:CMD_From_HMI OK");
#endif

    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // Serial_debug.println("WiFi.mode(AP):");
    WiFi.mode(WIFI_AP);
    sprintf(ap_name, "HB-%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    if (WiFi.softAP(ap_name, "12345678"))
    { // defualt IP address :192.168.4.1 password min 8 digis
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
        Serial.printf("\nWiFi AP: %s Started", ap_name);
#endif
    }
    else
    {
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
        Serial.printf("\nWiFi AP NOT OK YET...");
#endif
        vTaskDelay(500);
    }

    // Init pwm output
    pwm_heat.pause();
    pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
    pwm_heat.resume();
#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
    pwm_heat.printDebug();
    Serial.println("PWM started");
#endif

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

    // mb.addHreg(SV_HREG);
    // mb.addHreg(PID_HREG);
    // mb.addHreg(PID_P_HREG);
    // mb.addHreg(PID_I_HREG);
    // mb.addHreg(PID_D_HREG);
    // INIT MODBUS HREG VALUE
    mb.Hreg(BT_HREG, 0);      // 初始化赋值
    mb.Hreg(ET_HREG, 0);      // 初始化赋值
    mb.Hreg(INLET_HREG, 0);   // 初始化赋值
    mb.Hreg(EXHAUST_HREG, 0); // 初始化赋值

    mb.Hreg(HEAT_HREG, 0); // 初始化赋值
    mb.Hreg(FAN_HREG, 0);  // 初始化赋值

    // mb.Hreg(SV_HREG, 0);      // 初始化赋值
    // mb.Hreg(PID_HREG, 0);     // 初始化赋值
    // mb.Hreg(PID_P_HREG, 500); // 初始化赋值 X100
    // mb.Hreg(PID_I_HREG, 0);   // 初始化赋值 X100
    // mb.Hreg(PID_D_HREG, 10);  // 初始化赋值 X100

    ////////////////////////////////////////////////////////////////

    digitalWrite(SYSTEM_RLY, HIGH); // 启动机器
}

void loop()
{
    mb.task();
}