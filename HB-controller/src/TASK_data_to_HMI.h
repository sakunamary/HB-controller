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
    uint8_t Serial_DATA_Buffer[BUFFER_SIZE];
    const TickType_t timeout = 200;
    const TickType_t xIntervel = 100 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xQueueReceive(queue_data_to_HMI, &Serial_DATA_Buffer, timeout) == pdPASS)
        { // 从接收QueueCMD 接收指令
            Serial_HMI.write(Serial_DATA_Buffer，sizeof(Serial_DATA_Buffer));
            vTaskDelay(20);
        }
    }
}

#endif