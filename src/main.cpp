#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "HardwareSerial.h"
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include <ESP32Servo.h>

#include "TASK_read_temp.h"
#include "TASK_modbus_control.h"

String local_IP;
ExternalEEPROM I2C_EEPROM;
ESP32PWM pwm_heat;

char ap_name[30];
uint8_t macAddr[6];
extern double BT_TEMP;
const byte pwm_heat_out = PWM_HEAT;

pid_setting_t pid_parm = {
    .pid_CT = 2,
    .p = 25.41,
    .i = 1.81,
    .d = 99.74,
    .BT_tempfix = 0.0,
    .ET_tempfix = 0.0,
    .inlet_tempfix = 0.0,
    .EX_tempfix = 0.0};

void setup()
{
    // loopTaskWDTEnabled = true;
    xThermoDataMutex = xSemaphoreCreateMutex();
    ESP32PWM::allocateTimer(0);

    pinMode(SYSTEM_RLY, OUTPUT);
    pinMode(FAN_RLY, OUTPUT);
    pinMode(HEAT_RLY, OUTPUT);

    digitalWrite(SYSTEM_RLY, LOW); // 初始化电路启动；
    digitalWrite(FAN_RLY, LOW);    // 初始化电路启动；
    digitalWrite(HEAT_RLY, LOW);   // 初始化电路启动；

    Serial.begin(BAUDRATE); //

    // Serial_HMI.begin(BAUD_HMI, SERIAL_8N1, HMI_RX, HMI_TX);

#if defined(DEBUG_MODE)
    Serial.printf("\nSerial Started:");
    Serial.println(VERSION);
#endif

    // INIT SENSOR
    aht20.begin();
    ADC_MCP3424.NewConversion();

#if defined(DEBUG_MODE)
    Serial.println("start Reading EEPROM setting ...");
#endif
    if (!I2C_EEPROM.begin())
    {
        Serial.println("failed to initialise EEPROM");
        delay(1000);
    }
    else
    {
        I2C_EEPROM.get(0, pid_parm);

#if defined(DEBUG_MODE)
        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("pid_CT:%d\n", pid_parm.pid_CT);
        Serial.printf("PID kp:%4.2f\n", pid_parm.p);
        Serial.printf("PID ki:%4.2f\n", pid_parm.i);
        Serial.printf("PID kd:%4.2f\n", pid_parm.d);
        Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
        Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
        Serial.printf("Inlet fix:%4.2f\n", pid_parm.inlet_tempfix);
        Serial.printf("EX fix:%4.2f\n", pid_parm.EX_tempfix);
#endif
    }

    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // Serial_debug.println("WiFi.mode(AP):");
    WiFi.mode(WIFI_AP);
    sprintf(ap_name, "HB-%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    if (WiFi.softAP(ap_name, "88888888"))
    { // defualt IP address :192.168.4.1 password min 8 digis
#if defined(DEBUG_MODE)
        Serial.printf("\nWiFi AP: %s Started\n", ap_name);
#endif
    }
    else
    {
#if defined(DEBUG_MODE)
        Serial.printf("\nWiFi AP NOT OK YET...\n");
#endif
        vTaskDelay(1000);
    }

    // Init pwm output
    pwm_heat.attachPin(pwm_heat_out, frequency, resolution); // 1KHz 8 bit
    pwm_heat.writeScaled(0.0);

    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreate(
        Task_Thermo_get_data, "Thermo_get_data" //
        ,
        1024 * 8 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 5 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=1:Thermo_get_data OK");
#endif

    xTaskCreate(
        Task_modbus_control, "modbus_control" //
        ,
        1024 * 10 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTask_modbus_control // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=2:modbus_control OK");
#endif
    // INIT MODBUS

    mb.server(502); // Start Modbus IP //default port :502

#if defined(DEBUG_MODE)
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

    mb.addHreg(AMB_RH_HREG);
    mb.addHreg(AMB_TEMP_HREG);

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

    mb.Hreg(AMB_RH_HREG, 0);   // 初始化赋值
    mb.Hreg(AMB_TEMP_HREG, 0); // 初始化赋值

    // init PID
    Heat_pid_controller.begin(&BT_TEMP, &PID_output, &pid_sv, pid_parm.p, pid_parm.i, pid_parm.d);
    Heat_pid_controller.setSampleTime(pid_parm.pid_CT * 1000);                                               // 3000ms
    Heat_pid_controller.setOutputLimits(map(pid_out_min, 0, 100, 0, 255), map(pid_out_max, 0, 100, 0, 255)); // 取值范围（0-255）-> (76-205)
    Heat_pid_controller.setBias(255.0 / 2.0);
    Heat_pid_controller.setWindUpLimits(-1, 1); // Groth bounds for the integral term to prevent integral wind-up
    Heat_pid_controller.start();

    ////////////////////////////////////////////////////////////////
    vTaskDelay(3000);
    digitalWrite(SYSTEM_RLY, HIGH); // 启动机器

#if defined(DEBUG_MODE)
        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("pid_CT:%d\n", pid_parm.pid_CT);
        Serial.printf("PID kp:%4.2f\n", pid_parm.p);
        Serial.printf("PID ki:%4.2f\n", pid_parm.i);
        Serial.printf("PID kd:%4.2f\n", pid_parm.d);
        Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
        Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
        Serial.printf("Inlet fix:%4.2f\n", pid_parm.inlet_tempfix);
        Serial.printf("EX fix:%4.2f\n", pid_parm.EX_tempfix);
#endif
}

void loop()
{
    mb.task();
}