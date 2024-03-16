#ifndef __TASK_DATA_TO_HMI_H__
#define __TASK_DATD_TO_HMI_H__
#include <Arduino.h>
#include "config.h"



// Task for keep sending 指令到TC4
void TASK_data_to_HMI(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 500;
    uint8_t DATA_Buffer[BUFFER_SIZE];

    const TickType_t xIntervel = 200 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xQueueReceive(queue_data_to_HMI, &CMDBuffer, timeOut) == pdPASS)
        { // 从接收QueueCMD 接收指令
            Serial_in.print((char *)CMDBuffer);
            vTaskDelay(20);
        }
    }
}


#endif