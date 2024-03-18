#ifndef __TASK_MODBUS_CONTROL_H__
#define __TASK_MODBUS_CONTROL_H__

#include <Arduino.h>
#include <config.h>

#include <pwmWrite.h>
// PWM Pins
const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-4096

Pwm pwm_heat = Pwm();

const uint16_t HEAT_HREG = 3005;
const uint16_t SV_HREG = 3006;
const uint16_t PID_HREG = 3007;
const uint16_t PID_P_HREG = 3008;
const uint16_t PID_I_HREG = 3009;
const uint16_t PID_D_HREG = 3010;
const uint16_t FAN_HREG = 3011;

uint16_t last_PWR;
uint16_t last_SV;
 uint16_t last_PID_P;
 uint16_t last_PID_I;
 uint16_t last_PID_D;

int heat_level_to_artisan = 0;
bool init_status = true;
bool pid_on_status = false;

void Task_modbus_control(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char CMD_DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 100 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        if (init_status)
        {
            last_PWR = mb.Hreg(HEAT_HREG);
            // send temp data to queue to HMI
            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
            {
                last_PWR = mb.Hreg(HEAT_HREG);
                heat_level_to_artisan = 0;
                sprintf(CMD_DATA_Buffer, "num_pwr.val=%d\xff\xff\xff", last_PWR);
                xQueueSend(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                //Serial.printf("pwr(in task_modbus_control):%d\n", heat_level_to_artisan);

                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
            }
            init_status = false;
        }
        else
        {
            if (last_PWR != mb.Hreg(HEAT_HREG)) // 发生变动
            {

                if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                {
                    heat_level_to_artisan = mb.Hreg(HEAT_HREG);
                    last_PWR = heat_level_to_artisan;
                    sprintf(CMD_DATA_Buffer, "num_pwr.val=%d\xff\xff\xff", last_PWR);
                    xQueueSend(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                    //Serial.printf("pwr(in task_modbus_control):%d\n", heat_level_to_artisan);

                    xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                }
            }
        }
    }
}
#endif