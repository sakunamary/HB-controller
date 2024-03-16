#ifndef __TASK_MODBUS_CONTROL_H__
#define __TASK_MODBUS_CONTROL_H__

#include <Arduino.h>
#include <config.h>
#include <pwmWrite.h>

const uint16_t HEAT_HREG = 3005;
const uint16_t SV_HREG = 3006;
const uint16_t PID_HREG = 3007;
const uint16_t PID_P_HREG = 3008;
const uint16_t PID_I_HREG = 3009;
const uint16_t PID_D_HREG = 30110;

uint16_t last_PWR;
uint16_t last_SV;
uint16_t last_PID_P;
uint16_t last_PID_I;
uint16_t last_PID_D;

// PWM Pins
const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-4096

Pwm pwm_heat = Pwm();

int heat_level_to_artisan = 0;
bool init_status = true;
bool pid_on_status = false;

void Task_modbus_control(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    char DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 100 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        {
            if (init_status)
            {
                last_PWR = mb.Hreg(HEAT_HREG);
                // send temp data to queue to HMI
                memset(DATA_Buffer, '\0', sizeof(DATA_Buffer));
                sprintf(DATA_Buffer, "@SEND 103 %d", last_PWR);
                xQueueSend(queue_data_to_HMI, &DATA_Buffer, xIntervel);

                init_status = true;
                pid_on_status = false;
            }
            else
            {
                if (last_PWR != mb.Hreg(HEAT_HREG)) // 发生变动
                {
                    heat_level_to_artisan = mb.Hreg(HEAT_HREG);
                }
            }
            // xSemaphoreGive(xGetDataMutex); // end of lock mutex
        }
        pwm_heat.write(HEAT_OUT_PIN, map(heat_level_to_artisan, 0, 100, 230, 850), PWM_FREQ, resolution);
    }
}

#endif