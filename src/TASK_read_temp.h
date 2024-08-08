#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__

#include <Arduino.h>
#include "config.h"
#include <Wire.h>
#include <MCP3424.h>
#include "TypeK.h"
#include "DFRobot_AHT20.h"

#include <WiFi.h>

#include <ModbusIP_ESP8266.h>
ModbusIP mb; // declear object
uint8_t MCP3424_address = 0x68;
long Voltage; // Array used to store results

MCP3424 ADC_MCP3424(MCP3424_address); // Declaration of MCP3424 A2=0 A1=1 A0=0

DFRobot_AHT20 aht20;

TypeK temp_K_cal;

double BT_TEMP;
double ET_TEMP;
double INLET_TEMP;
double EX_TEMP;
double AMB_RH;
double AMB_TEMP;

int i, j;
double bt_temp[5];
double temp_;
extern pid_setting_t pid_parm;

// Need this for the lower level access to set them up.

// Modbus Registers Offsets
const uint16_t BT_HREG = 3001;
const uint16_t ET_HREG = 3002;
const uint16_t INLET_HREG = 3003;
const uint16_t EXHAUST_HREG = 3004;
const uint16_t AMB_RH_HREG = 3010;
const uint16_t AMB_TEMP_HREG = 3011;

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

    while (1)
    { // for loop
        // Wait for the next cycle (intervel 2000ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {

            if (aht20.startMeasurementReady(/* crcEn = */ true))
            {
                AMB_TEMP = aht20.getTemperature_C();
                AMB_RH = aht20.getHumidity_RH();
            }
            vTaskDelay(50);

            ADC_MCP3424.Configuration(2, ADC_BIT, 1, 8);                                                  // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
            Voltage = ADC_MCP3424.Measure();                                                              // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
            EX_TEMP = temp_K_cal.Temp_C(Voltage * 0.001, aht20.getTemperature_C()) + pid_parm.EX_tempfix; // CH2

            vTaskDelay(50);
            ADC_MCP3424.Configuration(1, ADC_BIT, 1, 1);
            Voltage = ADC_MCP3424.Measure();
            INLET_TEMP = pid_parm.inlet_tempfix + (((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083)); // CH1
            vTaskDelay(50);
            ADC_MCP3424.Configuration(3, ADC_BIT, 1, 1);
            BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083)); // CH3

#if defined(MODEL_M6S)
            ADC_MCP3424.Configuration(4, ADC_BIT, 1, 1);
            Voltage = ADC_MCP3424.Measure();
            ET_TEMP = pid_parm.ET_tempfix + (((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083)); // CH4
#endif
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }

#if defined(DEBUG_MODE) 
        Serial.printf("CH3 bt:%d\n", int(round(BT_TEMP * 10)));
        Serial.printf("CH1 inlet:%d\n", int(round(INLET_TEMP * 10)));
        Serial.printf("CH2 ex:%d\n", int(round(BT_TEMP * 10)));
        Serial.println();
#endif
        // update  Hreg data
        mb.Hreg(BT_HREG, int(round(BT_TEMP * 10)));        // 初始化赋值
        mb.Hreg(INLET_HREG, int(round(INLET_TEMP * 10)));  // 初始化赋值
        mb.Hreg(EXHAUST_HREG, int(round(EX_TEMP * 10)));   // 初始化赋值
        mb.Hreg(AMB_RH_HREG, int(round(AMB_RH * 10)));     // 初始化赋值
        mb.Hreg(AMB_TEMP_HREG, int(round(AMB_TEMP * 10))); // 初始化赋值

// making the HMI frame
#if defined(MODEL_M6S)
        mb.Hreg(ET_HREG, int(round(ET_TEMP * 10))); // 初始化赋值
#endif
    }

} // function

#endif
