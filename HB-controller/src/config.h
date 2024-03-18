
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define VERSION "1.0.0"
#define DEBUG_MODE
// uncomment to make work for HB M6SE ,default is work for HB M2SE
// #define MODEL_M6S
// uncomment to switch to MODBUS RTU   ,default is work in MODBUS TCP MODE
// #define MODBUS_RTU
#define SLAVE_ID 1 // MODBUS RTU SLAVE ID

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
#define ENC_BUTTON 34
#define ENC_CLK 32
#define ENC_DT 35

#define SPI_MISO 19
#define SPI_SCK 18
#define SPI_MOSI 23

#define SPI_CS_INLET 27 // ch1 OK
#define SPI_CS_EX 14    // ch2 OK
#define SPI_CS_BT 26    // ch3 OK
#define SPI_CS_ET 25    // ch4 OK

#define SYSTEM_RLY 22
#define FAN_RLY 21
#define HEAT_RLY 12
#define PWM_HEAT 33

#define HMI_TX 17
#define HMI_RX 16

const int BUFFER_SIZE = 32;

// parametsrs of MAX31865
#define RREF 430.0
#define RNOMINAL 100.0

// 网页设置的参数
typedef struct S_data_frame
{
    double BT_TEMP;
    double ET_TEMP;
    double INLET_TEMP;
    double EX_TEMP;
    int HEAT_SWITCH;
    int FAN_SWITCH;
    int HEAT_PWR;
} S_data_frame_t;

//extern S_data_frame_t user_wifi;

// user_wifi_t user_wifi = {
// BT_TEMP = 0.0;
// ET_TEMP = 0.0;
// INLET_TEMP = 0.0;
// EX_TEMP = 0.0;
// HEAT_SWITCH = 0;
// FAN_SWITCH = 0;
// HEAT_PWR = 0;
// }
// ;

#endif