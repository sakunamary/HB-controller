#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include <Wire.h>
#include "max6675.h"
#include <Adafruit_MAX31865.h>
#include <ModbusIP_ESP8266.h>

#define SPI_CS_INLET 25 // ch1 ok
#define SPI_CS_EX 14    // ch2 ok
#define SPI_CS_BT 26    // ch3
#define SPI_CS_ET 27    // ch4

#define SPI_MISO 19
#define SPI_SCK 18
#define SPI_MOSI 23

extern double BT_TEMP;
extern double ET_TEMP;
extern double INLET_TEMP;
extern double EX_TEMP;

MAX6675 thermo_EX(SPI_SCK, SPI_CS_EX, SPI_MISO); // CH2  thermoEX

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo_INLET = Adafruit_MAX31865(SPI_CS_INLET, SPI_MOSI, SPI_MISO, SPI_SCK); // CH1
Adafruit_MAX31865 thermo_BT = Adafruit_MAX31865(SPI_CS_BT, SPI_MOSI, SPI_MISO, SPI_SCK);       // CH3

SemaphoreHandle_t xThermoDataMutex = NULL;

#if defined(MODEL_M6S)
Adafruit_MAX31865 thermo_ET = Adafruit_MAX31865(SPI_CS_ET, SPI_MOSI, SPI_MISO, SPI_SCK); // CH4
#endif

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

// ModbusIP object
ModbusIP mb;

// Modbus Registers Offsets
const uint16_t BT_HREG = 3001;
const uint16_t ET_HREG = 3002;
const uint16_t INLET_HREG = 3003;
const uint16_t EXHAUST_HREG = 3004;

void TaskThermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;

    const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {                                                          // lock the  mutex
            // 读取max6675数据
            EX_TEMP = round((thermo_EX.readCelsius() * 10) / 10);
            vTaskDelay(20);
            INLET_TEMP = round((thermo_INLET.temperature(RNOMINAL, RREF) * 10) / 10);
            vTaskDelay(20);
            BT_TEMP = round((thermo_BT.temperature(RNOMINAL, RREF) * 10) / 10);
            vTaskDelay(20);

            mb.Hreg(BT_HREG, BT_TEMP);       // 初始化赋值
            mb.Hreg(INLET_HREG, INLET_TEMP); // 初始化赋值
            mb.Hreg(EXHAUST_HREG, EX_TEMP);  // 初始化赋值

#if defined(MODEL_M6S)
            ET_TEMP = round((thermo_ET.temperature(RNOMINAL, RREF) * 10) / 10);
            mb.Hreg(ET_HREG, ET_TEMP); // 初始化赋值
#endif
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
    }

} // function

#endif