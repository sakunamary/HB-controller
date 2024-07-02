
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define VERSION "1.0.5"
#define DEBUG_MODE
//  uncomment to make work for HB M6SE ,default is work for HB M2SE
// #define MODEL_M6S

// pwm setting
#define PWM_FREQ 10000
#define PWM_RESOLUTION 10 // 0-1024

///////////////////////////////////////
//   DO NOT make any change below    //
///////////////////////////////////////
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

// pinout define
#define SPI_MISO 8
#define SPI_SCK 9
#define SPI_MOSI 10

#define SYSTEM_RLY 3
#define FAN_RLY 4
#define HEAT_RLY 2
#define PWM_HEAT 5

#define HMI_TX 21
#define HMI_RX 20

#define I2C_SDA 6
#define I2C_SCL 7

#define PID_MIN_OUT 0
#define PID_MAX_OUT 60

const int BUFFER_SIZE = 16;

// parametsrs of MAX31865
#define RREF 100
#define RNOMINAL 1000
#define ADC_BIT 16

// EEPROM address
#define LOCATION_SETTINGS 0
//
typedef struct eeprom_settings
{
    int pid_CT;
    double p;
    double i;
    double d;
    double BT_tempfix;
    double ET_tempfix;
    double inlet_tempfix;
    double EX_tempfix;
} pid_setting_t;


static TaskHandle_t xTASK_data_to_HMI = NULL;
static TaskHandle_t xTASK_CMD_HMI = NULL;
static TaskHandle_t xTASK_HMI_CMD_handle = NULL;
static TaskHandle_t xTask_PID_autotune = NULL;
static TaskHandle_t xTask_modbus_control = NULL;

SemaphoreHandle_t xThermoDataMutex = NULL;
SemaphoreHandle_t xSerialReadBufferMutex = NULL;

QueueHandle_t queue_data_to_HMI = xQueueCreate(15, sizeof(uint8_t[BUFFER_SIZE])); // 发送到TC4的命令队列
QueueHandle_t queueCMD = xQueueCreate(15, sizeof(uint8_t[BUFFER_SIZE]));          // 发送到TC4的命令队列

#endif
// HB --> HMI的数据帧 FrameLenght = 16
// 帧头: 69 FF
// 类型: 01温度数据
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// 温度3: 00 00 // uint16
// 温度4: 00 00 // uint16
// NULL: 00 00
// 帧尾:FF FF FF

// HB --> HMI的控制状态帧 FrameLenght = 16
// 帧头: 69 FF
// 类型:02控制数据
// 系统OK : 00
// 火力: 00 00 // uint16
// PID SV : 00 00 // uint16
// PID_STATUS: 00 // uint8
// 火力开关: 00
// 冷却开关: 00 // uint16
// PID_TUNE :00
// NULL: 00
// 帧尾:FF FF FF

// 温度为小端模式   dec 2222  hex AE 08

// pid

// 20pa  CT:3s SV:180  60 -5
// kp 12.95
// ki 1.05
// kd 105.4

// kp 16.14
// ki 1.42
// kd 121.33

// kp 15.59
// ki 1.36
// kd 120.24

// 20 pa CT 3s SV 180  60-0
// p 11.52
// i 0.98
// d 89.74