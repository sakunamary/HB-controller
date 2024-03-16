#ifndef __TASK_CMD_FROM_HMI_H__
#define __TASK_CMD_FROM_HMI_H__
#include <Arduino.h>
#include "config.h"

#include <HardwareSerial.h>

QueueHandle_t queueCMD = xQueueCreate(8, sizeof(char[BUFFER_SIZE])); // 发送到TC4的命令队列



#endif