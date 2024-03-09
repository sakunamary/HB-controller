
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define VERSION "1.0.0"

#define MODEL_M6S

#define SPI_MISO 19
#define SPI_SCK 18
#define SPI_MOSI 23
#define SPI_CS_BT 26
#define SPI_CS_INLET 27
#define SPI_CS_EX 5
#define SPI_CS_ET 25

#define SYSTEM_RLY 22
#define FAN_RLY 21
#define HEAT_RLY 14
#define PWM_HEAT 33


#define DEBUG_MODE
#define HMI_TX 17
#define HMI_RX 16

const int BUFFER_SIZE = 64;

#endif