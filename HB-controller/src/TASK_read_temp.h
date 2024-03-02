#ifndef __TASK_READ_TEMP_H__
#define __TASK_READ_TEMP_H__



#include <Arduino.h>
#include <Wire.h>
#include "max6675.h"
#include <Adafruit_MAX31865.h>

extern  double BT_TEMP ; 
extern  double ET_TEMP ;
extern  double INLET_TEMP ;
extern  double EX_TEMP ;

int thermoMISO = 19;
int thermoCLK = 18;
int thermoMOSI = 23 ;

int thermoCS_BT = 35;
int thermoCS_INLET = 32;
int thermoCS_EX = 33;

MAX6675 thermo_EX(thermoCLK, thermoCS_EX, thermoMISO); //CH2  thermoEX
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo_INLET = Adafruit_MAX31865(thermoCS_INLET, thermoMOSI, thermoMISO, thermoCLK); //CH1
Adafruit_MAX31865 thermo_BT = Adafruit_MAX31865(thermoCS_BT, thermoMOSI, thermoMISO, thermoCLK); //CH3

SemaphoreHandle_t xThermoDataMutex = NULL;


#if defined(MODEL_M6S)
    int thermoCS_ET = 34;
    Adafruit_MAX31865 thermo_ET = Adafruit_MAX31865(thermoCS_ET, thermoMOSI, thermoMISO, thermoCLK); //CH4
#endif

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

// // Modbus Registers Offsets
// const uint16_t BT_HREG = 3001;
// const uint16_t ET_HREG = 3002;
// const uint16_t INLET_HREG = 3003;
// const uint16_t EXHAUST_HREG = 3004;


void TaskThermo_get_data(void *pvParameters)
{ //function 

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;

    const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    { //for loop
        // Wait for the next cycle (intervel 1500ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
               if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS)  //给温度数组的最后一个数值写入数据
                {//lock the  mutex     
                //读取max6675数据
                EX_TEMP = round((thermo_EX.readCelsius()*10)/10) ;
                vTaskDelay(20);
                INLET_TEMP = round((thermo_INLET.temperature(RNOMINAL, RREF)*10)/10) ;
                vTaskDelay(20);
                BT_TEMP = round((thermo_BT.temperature(RNOMINAL, RREF)*10)/10) ;
                vTaskDelay(20);

#if defined(MODEL_M6S)
                ET_TEMP = round((thermo_ET.temperature(RNOMINAL, RREF)*10)/10) ;
#endif
                xSemaphoreGive(xThermoDataMutex);  //end of lock mutex
                }   
    }   

}//function 




#endif