#ifndef __TASK_DATA_TO_HMI_H__
#define __TASK_DATD_TO_HMI_H__
#include <Arduino.h>
#include "config.h"
SemaphoreHandle_t xSerialReadBufferMutex = NULL;
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
            Serial_HMI.write(Serial_DATA_Buffer,sizeof(Serial_DATA_Buffer));
            vTaskDelay(20);
        }
    }
}



void TASK_CMD_HMI(void *pvParameters)
{
    (void)pvParameters;
    // uint8_t HMI_ReadBuffer[BUFFER_SIZE];
    // const TickType_t timeOut = 200;
    // TickType_t xLastWakeTime;
    // const TickType_t xIntervel = 100 / portTICK_PERIOD_MS;
    // xLastWakeTime = xTaskGetTickCount();
    // for (;;)
    // {
    //     if (Serial_HMI.available())
    //     {
    //         if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
    //         {
    //             Serial_HMI.readBytes(HMI_ReadBuffer, BUFFER_SIZE);
    //             // Serial_HMI.println(String((char *)HMI_ReadBuffer));
    //             // xQueueSend(queueCMD, &HMI_ReadBuffer, timeOut);
    //             memset(HMI_ReadBuffer, '\0', sizeof(HMI_ReadBuffer));
    //         }
    //         xSemaphoreGive(xSerialReadBufferMutex);
    //         vTaskDelay(20);
    //     }
    // }


}

#endif