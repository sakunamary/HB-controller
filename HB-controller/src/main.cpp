#include <Arduino.h>
#include "config.h"
#include "TASK_read_temp.h"

// #include <cmndreader.h>

#include <ModbusIP_ESP8266.h>

SemaphoreHandle_t xserialReadBufferMutex = NULL;                           // Mutex for TC4数据输出时写入队列的数据
QueueHandle_t queueCMD = xQueueCreate(8, sizeof(char[BUFFER_SIZE]));       // 发送到TC4的命令队列
String local_IP;

// Modbus Registers Offsets
const uint16_t BT_HREG = 3001;
const uint16_t ET_HREG = 3002;
const uint16_t INLET_HREG = 3003;
const uint16_t EXHAUST_HREG = 3004;
const uint16_t HEAT_HREG = 3005;
const uint16_t FAN_HREG = 3006;
const uint16_t SV_HREG = 3007;
const uint16_t RESET_HREG = 3008;
const uint16_t PID_HREG = 3007;
const uint16_t PID_P_HREG = 3009;
const uint16_t PID_I_HREG = 3010;
const uint16_t PID_D_HREG = 3011;

double BT_TEMP ; 
double ET_TEMP ;
double INLET_TEMP ;
double EX_TEMP ;

char ap_name[30];
uint8_t macAddr[6];


bool init_status = true;
bool pid_on_status = false;

uint16_t last_SV;
uint16_t last_FAN;
uint16_t last_PWR;

// ModbusIP object
ModbusIP mb;

// CmndInterp ci(DELIM); // command interpreter object
uint8_t serialReadBuffer[BUFFER_SIZE];


void setup()
{

    xThermoDataMutex = xSemaphoreCreateMutex();
    xserialReadBufferMutex = xSemaphoreCreateMutex();
    Serial.begin(BAUDRATE);

#if defined(DEBUG_MODE)
    Serial.printf("\nSerial Started\n");
#endif

  thermo_INLET.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo_BT.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
#if defined(MODEL_M6S)
  thermo_ET.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
#endif

#if defined(DEBUG_MODE)
    Serial.printf("\nThermo sensor Started\n");
#endif


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
    Serial.printf("\nTASK=1:Thermo_get_data OK\n");
#endif


//     // Setup tasks to run independently.
//     xTaskCreatePinnedToCore(
//         TASK_Modbus_Send_DATA, "ModbusSendTask" // 测量电池电源数据，每分钟测量一次
//         ,
//         1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
//         ,
//         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//         ,
//         NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
//     );
// #if defined(DEBUG_MODE)
//     Serial.printf("\nTASK=3:ModbusSendTask OK\n");
// #endif

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
    sprintf(ap_name, "MBox-%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    if (WiFi.softAP(ap_name, "12345678"))
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
    mb.addHreg(FAN_HREG);
    mb.addHreg(SV_HREG);
    mb.addHreg(RESET_HREG);
    mb.addHreg(PID_HREG);
    mb.addHreg(PID_P_HREG);
    mb.addHreg(PID_I_HREG);
    mb.addHreg(PID_D_HREG);

    mb.Hreg(BT_HREG, 0);    // 初始化赋值
    mb.Hreg(ET_HREG, 0);    // 初始化赋值
    mb.Hreg(INLET_HREG, 0);  // 初始化赋值
    mb.Hreg(EXHAUST_HREG, 0);   // 初始化赋值
    mb.Hreg(HEAT_HREG, 0);  // 初始化赋值
    mb.Hreg(FAN_HREG, 0);   // 初始化赋值
    mb.Hreg(SV_HREG, 0);    // 初始化赋值
    mb.Hreg(RESET_HREG, 0); // 初始化赋值
    mb.Hreg(PID_HREG, 0);   // 初始化赋值
    mb.Hreg(PID_P_HREG, 500);   // 初始化赋值 X100
    mb.Hreg(PID_I_HREG, 0);   // 初始化赋值 X100
    mb.Hreg(PID_D_HREG, 10);   // 初始化赋值 X100


    ////////////////////////////////////////////////////////////////

    // ci.addCommand(&pid);
    // ci.addCommand(&io3);
    // ci.addCommand(&ot1);
}

void loop()
{
    mb.task();
}