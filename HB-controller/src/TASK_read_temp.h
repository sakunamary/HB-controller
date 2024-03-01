#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__
#include <Arduino.h>
#include "max6675.h"
#include <Adafruit_MAX31865.h>



int thermoDO = 19;
int thermoCLK = 5;
int thermoCS_ET = 16;
int thermoCS_BT = 17;


MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


#endif