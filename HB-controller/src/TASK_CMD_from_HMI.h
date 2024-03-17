#ifndef __TASK_CMD_FROM_HMI_H__
#define __TASK_CMD_FROM_HMI_H__
#include <Arduino.h>
#include "config.h"

QueueHandle_t queueCMD = xQueueCreate(8, sizeof(char[BUFFER_SIZE])); // 发送到TC4的命令队列
SemaphoreHandle_t xSerialReadBufferMutex = NULL;

void TASK_CMD_From_HMI(void *pvParameters)
{
    (void)pvParameters;
    char HMI_ReadBuffer[BUFFER_SIZE];
    const TickType_t timeOut = 500;

    for (;;)
    {
        if (Serial_HMI.available())
        {
            if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
            {
                auto count = Serial_HMI.readBytesUntil(';', HMI_ReadBuffer, BUFFER_SIZE);
                Serial_HMI.println(String((char *)HMI_ReadBuffer));
                xQueueSend(queueCMD, &HMI_ReadBuffer, timeOut);
                memset(HMI_ReadBuffer, '\0', sizeof(HMI_ReadBuffer));
            }
            xSemaphoreGive(xSerialReadBufferMutex);
            vTaskDelay(20);
        }
    }
}



#endif