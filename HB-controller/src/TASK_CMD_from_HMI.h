#ifndef __TASK_CMD_FROM_HMI_H__
#define __TASK_CMD_FROM_HMI_H__
#include <Arduino.h>
#include "config.h"
// #include <cmndreader.h>



SemaphoreHandle_t xserialReadBufferMutex = NULL;                     // Mutex for TC4数据输出时写入队列的数据



#endif