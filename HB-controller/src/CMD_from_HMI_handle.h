#ifndef __TASK_CMD_FROM_HMI_H__
#define __TASK_CMD_FROM_HMI_H__
#include <Arduino.h>
#include "config.h"
#include "SerialCommand.h"

// QueueHandle_t queueCMD = xQueueCreate(8, sizeof(char[BUFFER_SIZE])); // 发送到TC4的命令队列
// SemaphoreHandle_t xSerialReadBufferMutex = NULL;

// void TASK_CMD_From_HMI(void *pvParameters)
// {
//     (void)pvParameters;
//     char HMI_ReadBuffer[BUFFER_SIZE];
//     const TickType_t timeOut = 500;

//     for (;;)
//     {
//         if (Serial_HMI.available())
//         {
//             if (xSemaphoreTake(xSerialReadBufferMutex, timeOut) == pdPASS)
//             {
//                 auto count = Serial_HMI.readBytesUntil(';', HMI_ReadBuffer, BUFFER_SIZE);
//                 Serial_HMI.println(String((char *)HMI_ReadBuffer));
//                 xQueueSend(queueCMD, &HMI_ReadBuffer, timeOut);
//                 memset(HMI_ReadBuffer, '\0', sizeof(HMI_ReadBuffer));
//             }
//             xSemaphoreGive(xSerialReadBufferMutex);
//             vTaskDelay(20);
//         }
//     }
// }

//  HMI --> HB 的控制命令，主要从HMI模块传入的串口文本格式
//  火力开关：HEAT,1 / 0;
//  冷却开关：COOL,1 / 0;
//  火力值：PWR,0-100;
//  pid开关：PID_MODE,1 / 0 ;
//  PID_SV(X10): PID_SV,2345;
//  PID_P(X100): PID_P,0;
//  PID_I(X100): PID_I,0;
//  PID_D(X100): PID_D,0;

SerialCommand HMI_Commands(Serial_HMI);

// This is the default handler, and gets called when no other command matches.
void cmd_unrecognized()
{
    Serial.printf("HMI_serial command ERROR");
}

void cmd_HEAT()
{
    int val;
    char *arg;
    arg = HMI_Commands.next();
    if (arg != NULL)
    {
        val = atoi(arg);
        if (val != 0)
        {
            // 火力开关 ON
            digitalWrite(HEAT_RLY, HIGH); // 操作IO口，HMI 图标更新走内部处理
            Serial.printf("HMI turn heat ON fan \n");
        }
        else
        {                                // 火力开关 OFF
            digitalWrite(HEAT_RLY, LOW); ////操作IO口，HMI 图标更新走内部处理
            Serial.printf("HMI turn heat OFF fan \n");
        }
    }
}

void cmd_COOL()
{
    int val;
    char *arg;
    arg = HMI_Commands.next();
    if (arg != NULL)
    {
        val = atoi(arg);
        if (val != 0)
        {
            // 风扇开关 ON
            digitalWrite(FAN_RLY, HIGH); // 操作IO口，HMI 图标更新走内部处理
            mb.Hreg(FAN_HREG, 1);        // 更新 Modbus 的Herg
            Serial.printf("HMI turn fan ON fan \n");
        }
        else
        { // 风扇开关 OFF

            digitalWrite(FAN_RLY, LOW); ////操作IO口，HMI 图标更新走内部处理
            mb.Hreg(FAN_HREG, 0);       // 更新 Modbus 的Herg
            Serial.printf("HMI turn fan OFF fan \n");
        }
    }
}

void cmd_PID_MODE()
{
    int val;
    char *arg;
    arg = HMI_Commands.next();
    if (arg != NULL)
    {
        val = atoi(arg);
        if (val != 0)
        {
            // PID 模式开关 ON
        }
        else
        { // PID 模式开关 OFF
        }
    }
}

void cmd_PWR()
{
}

void cmd_PID_SV()
{
}

void cmd_PID_P()
{
}

void cmd_PID_I()
{
}
void cmd_PID_D()
{
}

#endif