
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define VERSION "1.0.3"
// #define DEBUG_MODE
//  uncomment to make work for HB M6SE ,default is work for HB M2SE
//  #define MODEL_M6S
//  uncomment to switch to MODBUS RTU   ,default is work in MODBUS TCP MODE
//  #define MODBUS_RTU
#define SLAVE_ID 1 // MODBUS RTU SLAVE ID

// pwm setting
#define PWM_FREQ = 10000
#define PWM_RESOLUTION 10 // 0-1024

///////////////////////////////////////
//   DO NOT make any change below    //
///////////////////////////////////////
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate
#define BAUD_HMI 9600
// pinout define
#define ENC_BUTTON 7
#define ENC_CLK 15
#define ENC_DT 16

#define SPI_MISO 36
#define SPI_SCK 37
#define SPI_MOSI 38

#define SPI_CS_INLET 42 // ch1 OK
#define SPI_CS_EX 41    // ch2 OK
#define SPI_CS_BT 40    // ch3 OK
#define SPI_CS_ET 39    // ch4 OK

#define SYSTEM_RLY 48
#define FAN_RLY 45
#define HEAT_RLY 47
#define PWM_HEAT 8

#define HMI_TX 13
#define HMI_RX 14

#define PID_MIN_OUT 30
#define PID_MAX_OUT 80

const int BUFFER_SIZE = 16;

// parametsrs of MAX31865
#define RREF 430.0
#define RNOMINAL 100.0
//
typedef struct eeprom_settings
{
    uint16_t pid_CT;
    double p;
    double i;
    double d;
    double BT_tempfix;
    double ET_tempfix;
} pid_setting_t;

// publc funciton

uint8_t make_frame_package(uint8_t data_array[BUFFER_SIZE], bool cmd_inbond, int cmd_type)
// pagkage the data frame end .cmd_type:1/data_frame;2/run_status;3/HMI_cmd
{
    if (cmd_inbond == true)
    {
        data_array[0] = 0x67; // frame head
    }
    else
    {
        data_array[0] = 0x69; // frame head
    }

    data_array[1] = 0xff; // frame head

    switch (cmd_type)
    {
    case 1:                    // data_frame
        data_array[2] = 0x01;  // data type
        data_array[11] = 0x00; // frame end
        data_array[12] = 0x00; // frame end
        data_array[13] = 0xff; // frame end
        data_array[14] = 0xff; // frame end
        data_array[15] = 0xff; // frame end

        break;
    case 2:                    // run_status
        data_array[2] = 0x02;  // data type
        data_array[12] = 0x00; // frame end
        data_array[13] = 0xff; // frame end
        data_array[14] = 0xff; // frame end
        data_array[15] = 0xff; // frame end
        break;
    default:
        break;
    }
    return data_array[BUFFER_SIZE];
}


uint8_t make_frame_data(uint8_t data_array[BUFFER_SIZE], int cmd_type, uint16_t in_val, int uBit)
// pagkage the data frame.cmd_type:1/data_frame;2/run_status;
{
    uint8_t high = highByte(in_val);
    uint8_t low = lowByte(in_val);
    switch (cmd_type)
    {
    case 1:
        if (uBit > 2 && uBit < 13)
        {
            data_array[uBit] = low;      // frame end
            data_array[uBit + 1] = high; // frame end
        }

        break;
    case 2:
        if (uBit > 3 && uBit < 8)
        {
            data_array[uBit] = low; // frame end
        }
        break;
    default:
        break;
    }
    return data_array[BUFFER_SIZE];
}

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
