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
   while (Serial_HMI.read() >= 0); //清空串口缓冲区

    for (;;)
    {
        if (Serial_HMI.available())
        {
            if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
            {
                Serial_HMI.readBytes(HMI_ReadBuffer, BUFFER_SIZE);
                // Serial_HMI.println(String((char *)HMI_ReadBuffer));
                // xQueueSend(queueCMD, &HMI_ReadBuffer, timeOut);
                memset(HMI_ReadBuffer, '\0', sizeof(HMI_ReadBuffer));
            }
            xSemaphoreGive(xSerialReadBufferMutex);
            vTaskDelay(20);
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

//温度为小端模式   dec 2222  hex AE 08
#endif