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
    uint8_t TEMP_DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
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
            ET_TEMP = thermo_ET.temperature(RNOMINAL, RREF);
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
        mb.Hreg(ET_HREG, int(round(ET_TEMP * 10))); // 初始化赋值
        TEMP_DATA_Buffer[9] = lowByte(int(round(ET_TEMP * 10)));
        TEMP_DATA_Buffer[10] = highByte(int(round(ET_TEMP * 10)));
#else
        TEMP_DATA_Buffer[9] = 0x00;
        TEMP_DATA_Buffer[10] = 0x00;
#endif
        TEMP_DATA_Buffer[0] = 0x69; // frame head
        TEMP_DATA_Buffer[1] = 0xff; // frame head
        TEMP_DATA_Buffer[2] = 0x01; // data type
        TEMP_DATA_Buffer[3] = lowByte(int(round(BT_TEMP * 10)));
        TEMP_DATA_Buffer[4] = highByte(int(round(BT_TEMP * 10)));
        TEMP_DATA_Buffer[5] = lowByte(int(round(INLET_HREG * 10)));
        TEMP_DATA_Buffer[6] = highByte(int(round(INLET_HREG * 10)));
        TEMP_DATA_Buffer[7] = lowByte(int(round(INLET_HREG * 10)));
        TEMP_DATA_Buffer[8] = highByte(int(round(INLET_HREG * 10)));
        TEMP_DATA_Buffer[11] = 0xff; // frame end
        TEMP_DATA_Buffer[12] = 0xff; // frame end
        TEMP_DATA_Buffer[13] = 0xff; // frame end

        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 3);
    }

} // function

#endif

// HB --> HMI的数据帧 FrameLenght = 14
// 帧头: 69 FF
// 类型: 01温度数据
// BT: 00 00 // uint16
// Inlet: 00 00 // uint16
// EX: 00 00 // uint16
// ET: 00 00 // uint16
// 帧尾:FF FF FF

// HB --> HMI的控制状态帧 FrameLenght = 9
// 帧头: 67 FF
// 类型:02控制数据
// 火力: 00  // uint8
// 火力开关: 00 // uint8
// 冷却开关: 00 // uint8
// 帧尾:FF FF FF

// HMI --> HB的 命令帧 FrameLenght = 9
// 帧头: 67 FF
// 类型:03 控制数据
// 火力: 00  // uint8
// 火力开关: 00 // uint8
// 冷却开关: 00 // uint8
// 帧尾:FF FF FF

// 温度为小端模式   dec 2222  hex AE 08