#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include "max6675.h"
#include <Adafruit_MAX31865.h>
#include <WiFi.h>

#include <ModbusIP_ESP8266.h>
ModbusIP mb; // declear object

double BT_TEMP;
double ET_TEMP;
double INLET_TEMP;
double EX_TEMP;

SemaphoreHandle_t xThermoDataMutex = NULL;
QueueHandle_t queue_data_to_HMI = xQueueCreate(15, sizeof(char[BUFFER_SIZE])); // 发送到TC4的命令队列

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

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char TEMP_DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 2000 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    for (;;) // A Task shall never return or exit.
    {        // for loop
        // Wait for the next cycle (intervel 2000ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            vTaskDelay(60);
            INLET_TEMP = thermo_INLET.temperature(RNOMINAL, RREF); // CH1
            vTaskDelay(60);
            EX_TEMP = thermo_EX.readCelsius(); // CH2
            vTaskDelay(60);
            BT_TEMP = thermo_BT.temperature(RNOMINAL, RREF); // CH3

#if defined(MODEL_M6S)

            vTaskDelay(60);
            ET_TEMP = int(round(thermo_ET.temperature(RNOMINAL, RREF) * 10));
#endif
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }

#if defined(DEBUG_MODE) && !defined(MODBUS_RTU)
        Serial.printf("CH3 bt:%d\n", int(round(BT_TEMP * 10)));
        Serial.printf("CH1 inlet:%d\n", int(round(INLET_TEMP * 10)));
        Serial.printf("CH2 ex:%d\n", int(round(BT_TEMP * 10)));
        Serial.println();
#endif
        // update  Hreg data
        mb.Hreg(BT_HREG, int(round(BT_TEMP * 10)));       // 初始化赋值
        mb.Hreg(INLET_HREG, int(round(INLET_TEMP * 10))); // 初始化赋值
        mb.Hreg(EXHAUST_HREG, int(round(EX_TEMP * 10)));  // 初始化赋值

#if defined(MODEL_M6S)
        mb.Hreg(ET_HREG, int(round(ET_TEMP * 10)) ET_TEMP); // 初始化赋值
#endif

        sprintf(TEMP_DATA_Buffer, "float_ex.val=%d\xff\xff\xff", int(round(EX_TEMP * 10)));
        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 4);

        sprintf(TEMP_DATA_Buffer, "float_bt.val=%d\xff\xff\xff", int(round(BT_TEMP * 10)));
        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 4);

        sprintf(TEMP_DATA_Buffer, "float_in.val=%d\xff\xff\xff", int(round(INLET_TEMP * 10)));
        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 4);

#if defined(MODEL_M6S)

        sprintf(TEMP_DATA_Buffer, "float_et.val=%d\xff\xff\xff", ET_TEMP);
        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 2);
#endif
    }

} // function

#endif


// HB --> HMI的 自定义数据帧 
// 帧头: 69 FF  
// 温度1: 00 00  
// 温度2: 00 00 
// 温度3: 00 00  
// 温度4: 00 00  
// 火力: 00
// 火力开关: 00
// 冷却开关: 00
// 帧尾:FF FF FF 

//FrameLenght 16 bit 
//温度为小端模式   dec 2222  hex AE 08