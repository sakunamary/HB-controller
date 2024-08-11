#ifndef __TASK_HMI_SERIAL_H__
#define __TASK_HMI_SERIAL_H__
#include <Arduino.h>
#include "config.h"
#include <pwmWrite.h>

// 发送指令到HMI
void TASK_data_to_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t Serial_DATA_Buffer[BUFFER_SIZE];
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
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
                vTaskDelay(20);
            }
        }
    }
}

void TASK_CMD_FROM_HMI(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_ReadBuffer[BUFFER_SIZE];
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    while (1)
    {
        if (Serial_HMI.available())
        {
            Serial_HMI.readBytes(HMI_ReadBuffer, BUFFER_SIZE);
            xQueueSend(queueCMD, &HMI_ReadBuffer, timeOut);   // 串口数据发送至队列
            xTaskNotify(xTASK_HMI_CMD_handle, 0, eIncrement); // 通知处理任务干活
            
        }
        delay(50);
    }
}

void TASK_HMI_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t HMI_CMD_Buffer[BUFFER_SIZE];
    uint8_t HMI_OUT_Buffer[BUFFER_SIZE];
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 500 / portTICK_PERIOD_MS;
    uint16_t temp_pwr = 0;
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
//Serial.write(HMI_CMD_Buffer, BUFFER_SIZE);
                if (HMI_CMD_Buffer[0] == 0x00 && HMI_CMD_Buffer[1] == 0x00 && HMI_CMD_Buffer[2] == 0x00 && HMI_CMD_Buffer[6] == 0x88)
                {

                    Serial.println("CMD form HMI");
                    Serial.write(HMI_CMD_Buffer, BUFFER_SIZE);
                    // 67 FF 01 00 00 00 00 00 00 00 00 25 37 00 FF FF FF
                    HMI_OUT_Buffer[0] = 0x67;
                    HMI_OUT_Buffer[1] = 0xff;
                    HMI_OUT_Buffer[2] = 0x00;
                    HMI_OUT_Buffer[3] = 0xff;
                    HMI_OUT_Buffer[4] = 0xff;
                    HMI_OUT_Buffer[5] = 0xff;
                    HMI_OUT_Buffer[6] = 0x69;
                    HMI_OUT_Buffer[7] = 0x69;
                    HMI_OUT_Buffer[8] = 0x69;
                    HMI_OUT_Buffer[9] = 0x67;
                    HMI_OUT_Buffer[10] = 0x67;
                    HMI_OUT_Buffer[11] = 0x67;
                    HMI_OUT_Buffer[12] = 0xff;
                    HMI_OUT_Buffer[13] = 0xff;
                    HMI_OUT_Buffer[14] = 0xff;
                    HMI_OUT_Buffer[15] = 0xff;

                    for (int loop = 0; loop < 3; loop++)
                    {
                        delay(1000);
                        xQueueSend(queue_data_to_HMI, &HMI_OUT_Buffer, timeOut);
                        // Serial_HMI.write(HMI_OUT_Buffer,HMI_BUFFER_SIZE);
                        xTaskNotify(xTASK_data_to_HMI, 0, eIncrement); // send notify to TASK_data_to_HMI
                    }
                    memset(HMI_OUT_Buffer, '\0', BUFFER_SIZE);
                }
                delay(20);
            }
        }
    }
}
#endif
// printh 00 00 00 ff ff ff 88 ff ff ff//输出上电信息到串口
// 69 ff 00 ff ff ff 69 69 69 67 67 67 ff ff ff ff  //握手协议
// HMI --> HB的 命令帧 FrameLenght = 16
// 帧头: 67 FF
// 类型:03 控制数据
// 火力: 00  00 // uint16
// 火力开关: 00 00// uint16
// 冷却开关: 00 00// uint16
// NULL: 00 00 // uint16
// NULL: 00 00 // uint16
// 帧尾:FF FF FF
