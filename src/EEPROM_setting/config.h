
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define VERSION "1.0.3"
// pwm setting
#define PWM_FREQ 10000
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
#define PID_TUNE_SV 180.0 // flaot 


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
