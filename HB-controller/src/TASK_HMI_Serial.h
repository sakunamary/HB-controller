#ifndef __TASK_HMI_SERIAL_H__
#define __TASK_HMI_SERIAL_H__
#include <Arduino.h>
#include "config.h"

HardwareSerial Serial_HMI(2); // D16 RX_drumer  D17 TX_drumer

// Task for keep sending 指令到TC4
void TASK_data_to_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t Serial_DATA_Buffer[BUFFER_SIZE];
    const TickType_t timeOut = 500;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;

    while (1)
    {

        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待

        if (xResult == pdTRUE)
        {
            if (xQueueReceive(queue_data_to_HMI, &Serial_DATA_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令
                Serial_HMI.write(Serial_DATA_Buffer, BUFFER_SIZE);
                Serial.write(Serial_DATA_Buffer, BUFFER_SIZE);
                vTaskDelay(20);
            }
        }
    }
}

void TASK_CMD_FROM_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_ReadBuffer[BUFFER_SIZE];
    const TickType_t timeOut = 500;
    while (1)
    {
        if (Serial_HMI.available())
        {
            if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
            {
                Serial_HMI.readBytes(HMI_ReadBuffer, BUFFER_SIZE);
                xQueueSend(queueCMD, &HMI_ReadBuffer, timeOut);   // 串口数据发送至队列
                xTaskNotify(xTASK_HMI_CMD_handle, 0, eIncrement); // 通知处理任务干活
            }
            xSemaphoreGive(xSerialReadBufferMutex);
        }
        vTaskDelay(20);
    }
}

void TASK_HMI_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_CMD_Buffer[BUFFER_SIZE];
    const TickType_t timeOut = 500;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;

    while (1)
    {

        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待

        if (xResult == pdTRUE)
        {
            if (xQueueReceive(queueCMD, &HMI_CMD_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令
                Serial_HMI.write(HMI_CMD_Buffer, BUFFER_SIZE);
                vTaskDelay(20);
            }
        }
    }
}

#endif