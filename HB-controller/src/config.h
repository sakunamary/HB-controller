
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Wire.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define VERSION "1.0.0"

#define MODEL_M6S

#define ENC_BUTTON 34
#define ENC_CLK 32
#define ENC_DT 35

#define SPI_MISO 19
#define SPI_SCK 18
#define SPI_MOSI 23

#define SPI_CS_INLET 25 // ch1 OK
#define SPI_CS_EX 14    // ch2 OK
#define SPI_CS_BT 26    // ch3 OK 
#define SPI_CS_ET 27    // ch4 OK 

#define SYSTEM_RLY 22
#define FAN_RLY 21
#define HEAT_RLY 12
#define PWM_HEAT 33

#define DEBUG_MODE
#define HMI_TX 17
#define HMI_RX 16

const int BUFFER_SIZE = 32;



// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0


#endif