
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define VERSION "1.0.7"
// #define DEBUG_MODE
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
#define PID_TUNE_SV 180

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

#endif
// pid
// 20 pa CT 3s SV 180  60-0
// p 11.52
// i 0.98
// d 89.74


// 25 pa CT 3s SV 180  60-0
// p 7.42
// i 0.46
// d 79.48


// 25 pa CT 2s SV 180  60-0
// p 11.52
// i 0.98
// d 89.74