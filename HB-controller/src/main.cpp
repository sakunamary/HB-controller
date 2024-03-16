#include <Arduino.h>
#include "config.h"
#include "TASK_read_temp.h"
#include "TASK_modbus_control.h"
#include "TASK_CMD_from_HMI.h"
#include "TASK_data_to_HMI.h"



// #include <cmndreader.h>


HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer
SemaphoreHandle_t xserialReadBufferMutex = NULL;                     // Mutex for TC4数据输出时写入队列的数据
QueueHandle_t queueCMD = xQueueCreate(8, sizeof(char[BUFFER_SIZE])); // 发送到TC4的命令队列

String local_IP;

bool init_check();

const uint16_t HEAT_HREG = 3005;
const uint16_t SV_HREG = 3006;
const uint16_t PID_HREG = 3007;
const uint16_t PID_P_HREG = 3008;
const uint16_t PID_I_HREG = 3009;
const uint16_t PID_D_HREG = 30110;

double BT_TEMP;
double ET_TEMP;
double INLET_TEMP;
double EX_TEMP;

char ap_name[30];
uint8_t macAddr[6];

bool init_status = true;
bool pid_on_status = false;

// CmndInterp ci(DELIM); // command interpreter object
uint8_t serialReadBuffer[BUFFER_SIZE];

bool init_check()
{
    if (!thermo_INLET.begin(MAX31865_4WIRE))
    {
        Serial.printf("\nSensor INLET is Not ready...");
        return false;
    }
    if (!thermo_BT.begin(MAX31865_4WIRE))
    {
        Serial.printf("\nSensor BT is Not ready...");
        return false;
    }
#if defined(MODEL_M6S)
    if (!thermo_ET.begin(MAX31865_4WIRE))
    {
        Serial.printf("\nSensor ET is Not ready...");
        return false;
    }
#endif

#if defined(DEBUG_MODE)
    Serial.printf("\nThermo sensor Started");
#endif

    return true;
}

void setup()
{

    xThermoDataMutex = xSemaphoreCreateMutex();
    //xserialReadBufferMutex = xSemaphoreCreateMutex();

    pinMode(SYSTEM_RLY, OUTPUT);
    pinMode(FAN_RLY, OUTPUT);
    pinMode(HEAT_RLY, OUTPUT);

    digitalWrite(SYSTEM_RLY, LOW); // 初始化电路启动；

    Serial.begin(BAUDRATE);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, HMI_RX, HMI_TX);
#if defined(DEBUG_MODE)
    Serial.printf("\nSerial Started");
#endif

    while (!init_check())
    {
        Serial.printf("\nSensor is Not ready...");
        vTaskDelay(1000);
    }
    digitalWrite(HEAT_RLY, HIGH); // 火力电路启动；

    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TaskThermo_get_data, "Thermo_get_data" // 测量电池电源数据，每分钟测量一次
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 4 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=1:Thermo_get_data OK");
#endif

    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        Task_modbus_control, "modbus_control" // 测量电池电源数据，每分钟测量一次
        ,
        1024 * 4 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=2:modbus_control OK");
#endif

    //     // Setup tasks to run independently.
    //     xTaskCreate(
    //         TASK_Send_READ_CMDtoTC4, "READ_CMDtoTC4" // 测量电池电源数据，每分钟测量一次
    //         ,
    //         1024 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );

    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK=4:READ_CMDtoTC4 OK \n");
    // #endif

    //     // Setup tasks to run independently.
    //     xTaskCreate(
    //         TASK_SendCMDtoTC4, "SendCMDtoTC4" // 测量电池电源数据，每分钟测量一次
    //         ,
    //         1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );

    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK=5:SendCMDtoTC4 OK \n");
    // #endif

    //     // Setup tasks to run independently.
    //     xTaskCreatePinnedToCore(
    //         TASK_Modbus_From_CMD, "TASK_Modbus_From_CMD" // 测量电池电源数据，每分钟测量一次
    //         ,
    //         1024 * 10 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );

    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK=6:TASK_Modbus_From_CMD OK \n");
    // #endif

    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // Serial_debug.println("WiFi.mode(AP):");
    WiFi.mode(WIFI_AP);
    sprintf(ap_name, "HB-%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    if (WiFi.softAP(ap_name, "12345678"))
    { // defualt IP address :192.168.4.1 password min 8 digis
#if defined(DEBUG_MODE)
        Serial.printf("\nWiFi AP: %s Started", ap_name);
#endif
    }
    else
    {
#if defined(DEBUG_MODE)
        Serial.printf("\nWiFi AP NOT OK YET...");
#endif
        vTaskDelay(500);
    }

// Init Modbus-TCP
#if defined(DEBUG_MODE)
    Serial.printf("\nStart Modbus-TCP  service OK\n");
#endif

    mb.server(502); // Start Modbus IP //default port :502
    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    mb.addHreg(BT_HREG);
    mb.addHreg(ET_HREG);
    mb.addHreg(INLET_HREG);
    mb.addHreg(EXHAUST_HREG);

    mb.addHreg(HEAT_HREG);

    mb.addHreg(SV_HREG);
    mb.addHreg(PID_HREG);
    mb.addHreg(PID_P_HREG);
    mb.addHreg(PID_I_HREG);
    mb.addHreg(PID_D_HREG);

    mb.Hreg(BT_HREG, 0);      // 初始化赋值
    mb.Hreg(ET_HREG, 0);      // 初始化赋值
    mb.Hreg(INLET_HREG, 0);   // 初始化赋值
    mb.Hreg(EXHAUST_HREG, 0); // 初始化赋值

    mb.Hreg(HEAT_HREG, 0); // 初始化赋值

    mb.Hreg(SV_HREG, 0);      // 初始化赋值
    mb.Hreg(PID_HREG, 0);     // 初始化赋值
    mb.Hreg(PID_P_HREG, 500); // 初始化赋值 X100
    mb.Hreg(PID_I_HREG, 0);   // 初始化赋值 X100
    mb.Hreg(PID_D_HREG, 10);  // 初始化赋值 X100

    ////////////////////////////////////////////////////////////////

    // ci.addCommand(&pid);
    // ci.addCommand(&io3);
    // ci.addCommand(&ot1);
}

void loop()
{
    mb.task();
}