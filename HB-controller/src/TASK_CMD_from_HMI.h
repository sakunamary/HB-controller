#ifndef __TASK_CMD_FROM_HMI_H__
#define __TASK_CMD_FROM_HMI_H__
#include <Arduino.h>
#include "config.h"

// SemaphoreHandle_t xSerialReadBufferMutex = NULL;

void TASK_CMD_From_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_ReadBuffer[BUFFER_SIZE];
    const TickType_t timeOut = 200;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 100 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (Serial_HMI.read() >= 0)
        ; // 清空串口缓冲区

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

    while (Serial_HMI.available() >= BUFFER_SIZE)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        // 从串口缓冲读取1个字节但不删除
        // HMI_ReadBuffer[0] = Serial_HMI.peek();
        Serial_HMI.readBytes(HMI_ReadBuffer, BUFFER_SIZE);
        // 当获取的数据是包头(0x67 FF )时
        if (HMI_ReadBuffer[0] == 0x67 && HMI_ReadBuffer[1] == 0xFF)
        {

        }
        else
        {
            Serial_HMI.read(); // 从串口缓冲读取1个字节并删除
            memset(HMI_ReadBuffer,'\0',BUFFER_SIZE);
        }
    }
}

// HB --> HMI的数据帧 FrameLenght = 14
// 帧头: 69 FF
// 类型: 01温度数据
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// 温度3: 00 00 // uint16
// 温度4: 00 00 // uint16
// 帧尾:FF FF FF

// HB --> HMI的控制状态帧 FrameLenght = 9
// 帧头: 67 FF
// 类型:02控制数据
// 火力: 00  // uint8
// 火力开关: 00 // uint8
// 冷却开关: 00 // uint8
// 帧尾:FF FF FF

// HMI --> HB的 命令帧 FrameLenght = 9
// 帧头: 67 FF
// 类型:03 控制数据
// 火力: 00  // uint8
// 火力开关: 00 // uint8
// 冷却开关: 00 // uint8
// 帧尾:FF FF FF

// 温度为小端模式   dec 2222  hex AE 08
#endif