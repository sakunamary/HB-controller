#ifndef __TASK_MODBUS_CONTROL_H__
#define __TASK_MODBUS_CONTROL_H__

#include <Arduino.h>
#include <config.h>

#include <pwmWrite.h>
// PWM Pins
const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-4096

Pwm pwm_heat = Pwm();

const uint16_t HEAT_HREG = 3005;
// const uint16_t SV_HREG = 3006;
// const uint16_t PID_HREG = 3007;
// const uint16_t PID_P_HREG = 3008;
// const uint16_t PID_I_HREG = 3009;
// const uint16_t PID_D_HREG = 3010;
const uint16_t FAN_HREG = 3011;

uint16_t last_PWR;
bool fan_status;
bool heat_status;
// uint16_t last_SV;
//  uint16_t last_PID_P;
//  uint16_t last_PID_I;
//  uint16_t last_PID_D;

int heat_pwr_to_SSR = 0;

bool init_status = true;
// bool pid_on_status = false;

void Task_modbus_control(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    uint8_t CMD_DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 100 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {        // for loop
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        fan_status = digitalRead(FAN_RLY);
        heat_status = digitalRead(HEAT_RLY);

        if (init_status)
        {
            //
            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) //
            {
                last_PWR = mb.Hreg(HEAT_HREG);
                heat_pwr_to_SSR = 0;
                make_frame_data(CMD_DATA_Buffer, 2, last_PWR, 3);
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
            }
            init_status = false;
        }
        else
        {
            if (mb.Hreg(HEAT_HREG) != last_PWR) // 火力pwr数值发生变动
            {
                if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                {
                    last_PWR = mb.Hreg(HEAT_HREG);                       // last 火力pwr数据更新
                    heat_pwr_to_SSR = last_PWR;                          // 发送新火力pwr数据到 SSR
                    make_frame_data(CMD_DATA_Buffer, 2, last_PWR, 3); // 将新火力pwr数据发送到HMI run_status 帧
                }
            }
        }

        if (digitalRead(FAN_RLY) != fan_status) // 风扇开关状态发生变动
        {
            fan_status = digitalRead(FAN_RLY); // 保存现状到fan_status
            mb.Hreg(FAN_HREG, fan_status);     // 更新FAN_HREG的值
        }

        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 整合数据帧到HMI
        {
            make_frame_head(CMD_DATA_Buffer, 2);                           // 帧头
            make_frame_data(CMD_DATA_Buffer, 2, digitalRead(HEAT_RLY), 5); // 加热管状态数据
            make_frame_data(CMD_DATA_Buffer, 2, fan_status, 7);            // 冷却扇状态数据
            make_frame_end(CMD_DATA_Buffer, 2);                            // 帧微
            xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
        // pwm_heat.write(HEAT_OUT_PIN, map(heat_pwr_to_SSR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSR
    }
}
#endif

// HB --> HMI的数据帧 FrameLenght = 16
// 帧头: 69 FF
// 类型: 01温度数据
// 温度1: 00 00 // uint16
// 温度2: 00 00 // uint16
// 温度3: 00 00 // uint16
// 温度4: 00 00 // uint16
// NULL: 00 00
// 帧尾:FF FF FF

// HB --> HMI的控制状态帧 FrameLenght = 16
// 帧头: 69 FF
// 类型:02控制数据
// 火力: 00 00 // uint16
// 火力开关: 00 00 // uint16
// 冷却开关: 00 00 // uint16
// NULL: 00 00 // uint16
// NULL: 00 00 // uint16
// 帧尾:FF FF FF

// HMI --> HB的 命令帧 FrameLenght = 16
// 帧头: 67 FF
// 类型:03 控制数据
// 火力: 00  00 // uint16
// 火力开关: 00 00// uint16
// 冷却开关: 00 00// uint16
// NULL: 00 00 // uint16
// NULL: 00 00 // uint16
// 帧尾:FF FF FF

// 温度为小端模式   dec 2222  hex AE 08