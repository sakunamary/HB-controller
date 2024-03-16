#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include "max6675.h"
#include <Adafruit_MAX31865.h>

#if defined(MODBUS_RTU)
#include <ModbusRTU.h>
ModbusRTU mb;
#else
#include <ModbusIP_ESP8266.h>
ModbusIP mb;  //declear object 

uint16_t BT_TEMP;
uint16_t ET_TEMP;
uint16_t INLET_TEMP;
uint16_t EX_TEMP;

SemaphoreHandle_t xThermoDataMutex = NULL;
QueueHandle_t queue_data_to_HMI = xQueueCreate(10, sizeof(char[BUFFER_SIZE])); // 发送到TC4的命令队列

MAX6675 thermo_EX(SPI_SCK, SPI_CS_EX, SPI_MISO); // CH2  thermoEX

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo_INLET = Adafruit_MAX31865(SPI_CS_INLET, SPI_MOSI, SPI_MISO, SPI_SCK); // CH1
Adafruit_MAX31865 thermo_BT = Adafruit_MAX31865(SPI_CS_BT, SPI_MOSI, SPI_MISO, SPI_SCK);       // CH3

#if defined(MODEL_M6S)
Adafruit_MAX31865 thermo_ET = Adafruit_MAX31865(SPI_CS_ET, SPI_MOSI, SPI_MISO, SPI_SCK); // CH4
#endif


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
    char DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 2000 / portTICK_PERIOD_MS;
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
            EX_TEMP = int(round((thermo_EX.readCelsius() * 10) / 10) * 100);
            vTaskDelay(50);
            INLET_TEMP = int(round((thermo_INLET.temperature(RNOMINAL, RREF) * 10) / 10) * 100);
            vTaskDelay(50);
            BT_TEMP = int(round((thermo_BT.temperature(RNOMINAL, RREF) * 10) / 10) * 100);
            vTaskDelay(50);

            // send temp data to queue to HMI
            memset(DATA_Buffer, '\0', sizeof(DATA_Buffer));
            sprintf(DATA_Buffer, "@SEND 103 %d", BT_TEMP);
            xQueueSend(queue_data_to_HMI, &DATA_Buffer, xIntervel / 4);

            memset(DATA_Buffer, '\0', sizeof(DATA_Buffer));
            sprintf(DATA_Buffer, "@SEND 103 %d", INLET_TEMP);
            xQueueSend(queue_data_to_HMI, &DATA_Buffer, xIntervel / 4);

            memset(DATA_Buffer, '\0', sizeof(DATA_Buffer));
            sprintf(DATA_Buffer, "@SEND 103 %d", EX_TEMP);
            xQueueSend(queue_data_to_HMI, &DATA_Buffer, xIntervel / 4);

#if defined(MODEL_M6S)
            ET_TEMP = int(round((thermo_ET.temperature(RNOMINAL, RREF) * 10) / 10) * 100);
            vTaskDelay(50);
            memset(DATA_Buffer, '\0', sizeof(DATA_Buffer));
            sprintf(DATA_Buffer, "@SEND 103 %d", ET_TEMP);
            xQueueSend(queue_data_to_HMI, &DATA_Buffer, xIntervel / 4);
#endif

            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
        // update  Hreg data
        mb.Hreg(BT_HREG, BT_TEMP);       // 初始化赋值
        mb.Hreg(INLET_HREG, INLET_TEMP); // 初始化赋值
        mb.Hreg(EXHAUST_HREG, EX_TEMP);  // 初始化赋值
#if defined(MODEL_M6S)
        mb.Hreg(ET_HREG, ET_TEMP); // 初始化赋值
#endif
    }

} // function

#endif