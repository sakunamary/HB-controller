#ifndef __TASK_DATA_TO_HMI_H__
#define __TASK_DATD_TO_HMI_H__
#include <Arduino.h>
#include "config.h"

HardwareSerial Serial_HMI(2); // D16 RX_drumer  D17 TX_drumer

// Task for keep sending 指令到TC4
void TASK_data_to_HMI(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 200;
    uint8_t Serial_DATA_Buffer[BUFFER_SIZE];

    const TickType_t xIntervel = 250 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xQueueReceive(queue_data_to_HMI, &Serial_DATA_Buffer, timeOut) == pdPASS)
        { // 从接收QueueCMD 接收指令

            Serial_HMI.print((char *)Serial_DATA_Buffer);
            memset(Serial_DATA_Buffer, '\0', sizeof(Serial_DATA_Buffer));
            vTaskDelay(20);
        }
    }
}

#endif