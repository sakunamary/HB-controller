#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP3424.h>
#include "TypeK.h"
#include <Adafruit_AHTX0.h>


#include <WiFi.h>

#include <ModbusIP_ESP8266.h>
ModbusIP mb; // declear object

double BT_TEMP;
double ET_TEMP;
double INLET_TEMP;
double EX_TEMP;
int i, j;
double bt_temp[5];
double temp_;
extern pid_setting_t pid_parm;


// Need this for the lower level access to set them up.
uint8_t address = 0x68;
long Voltage; // Array used to store results


MCP3424 ADC_MPC3424(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
Adafruit_AHTX0 aht;
sensors_event_t temp_humidity, temp_aht20;
TypeK temp_K_cal;



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
            EX_TEMP = thermo_EX.readCelsius(); // CH2
            vTaskDelay(60);
            INLET_TEMP = thermo_INLET.temperature(RNOMINAL, RREF); // CH1
            for (i = 0; i < 5; i++)
            {
                vTaskDelay(60);
                bt_temp[i] = thermo_BT.temperature(RNOMINAL, RREF); // CH3
                for (j = i + 1; j < 5; j++)
                {
                    if (bt_temp[i] > bt_temp[j])
                    {
                        temp_ = bt_temp[i];
                        bt_temp[i] = bt_temp[j];
                        bt_temp[j] = temp_;
                    }
                }
            }
            BT_TEMP = bt_temp[2]; // for bt temp more accuricy

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
        make_frame_package(TEMP_DATA_Buffer, true, 1);
        make_frame_data(TEMP_DATA_Buffer, 1, int(round(BT_TEMP * 10)), 3);
        make_frame_data(TEMP_DATA_Buffer, 1, int(round(INLET_TEMP * 10)), 5);
        make_frame_data(TEMP_DATA_Buffer, 1, int(round(EX_TEMP * 10)), 7);
        xQueueSend(queue_data_to_HMI, &TEMP_DATA_Buffer, xIntervel / 3);
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
// 系统OK : 00
// 火力: 00 00 // uint16
// PID SV : 00 00 // uint16
// PID_STATUS: 00 // uint8
// 火力开关: 00
// 冷却开关: 00 // uint16
// PID_TUNE :00
// NULL: 00
// 帧尾:FF FF FF

// 温度为小端模式   dec 2222  hex AE 08
