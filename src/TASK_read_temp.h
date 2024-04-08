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

extern pid_setting_t pid_parm;

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
    BaseType_t xResult;
    uint8_t TEMP_DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 2000 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // setup for the the SPI library:

    while (1)
    { // for loop
        // Wait for the next cycle (intervel 2000ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            vTaskDelay(60);
            EX_TEMP = thermo_EX.readCelsius()+pid_parm.ET_tempfix; // CH2
            vTaskDelay(60);
            INLET_TEMP = thermo_INLET.temperature(RNOMINAL, RREF); // CH1
            vTaskDelay(60);
            BT_TEMP = thermo_BT.temperature(RNOMINAL, RREF)+pid_parm.BT_tempfix; // CH3

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
// making the HMI frame
#if defined(MODEL_M6S)
        mb.Hreg(ET_HREG, int(round(ET_TEMP * 10))); // 初始化赋值
//         make_frame_data(TEMP_DATA_Buffer, 1, int(round(ET_TEMP * 10)), 9);
// #else
//         make_frame_data(TEMP_DATA_Buffer, 1, 0, 9);
#endif
        // make_frame_head(TEMP_DATA_Buffer, 1);
        // make_frame_end(TEMP_DATA_Buffer, 1);
        // make_frame_data(TEMP_DATA_Buffer, 1, int(round(BT_TEMP * 10)), 3);
        // make_frame_data(TEMP_DATA_Buffer, 1, int(round(INLET_TEMP * 10)), 5);
        // make_frame_data(TEMP_DATA_Buffer, 1, int(round(EX_TEMP * 10)), 7);
        // xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 3);
        // // send notify to TASK_data_to_HMI
        // xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
    }

} // function

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
// 火力: 00 00 // uint16
// 火力开关: 00 00 // uint16
// 冷却开关: 00 00 // uint16
// NULL: 00 00 // uint16
// NULL: 00 00 // uint16
// 帧尾:FF FF FF

// HMI --> HB的 命令帧 FrameLenght = 16
// 帧头: 67 FF
// 类型:03 控制数据
// 火力: 00  00 // uint16
// 火力开关: 00 00// uint16
// 冷却开关: 00 00// uint16
// NULL: 00 00 // uint16
// NULL: 00 00 // uint16
// 帧尾:FF FF FF

// 温度为小端模式   dec 2222  hex AE 08
