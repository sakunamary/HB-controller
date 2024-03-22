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

const uint16_t PWR_HREG = 3005;
// const uint16_t SV_HREG = 3006;
// const uint16_t PID_HREG = 3007;
// const uint16_t PID_P_HREG = 3008;
// const uint16_t PID_I_HREG = 3009;
// const uint16_t PID_D_HREG = 3010;
const uint16_t FAN_HREG = 3011;
const uint16_t HEAT_HREG = 3012;

uint16_t last_PWR;

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
    const TickType_t xIntervel = 500 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1) // A Task shall never return or exit.
    {         // for loop
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        // HEAT_HREG
        if (init_status)
        {
            //
            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) //
            {
                last_PWR = mb.Hreg(PWR_HREG);
                heat_pwr_to_SSR = 0;
                // 合成HMI数据帧
                make_frame_head(CMD_DATA_Buffer, 2); // 帧头
                make_frame_data(CMD_DATA_Buffer, 2, last_PWR, 3);
                make_frame_end(CMD_DATA_Buffer, 2); // 帧微
                xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                pwm_heat.write(HEAT_OUT_PIN, map(heat_pwr_to_SSR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
                xSemaphoreGive(xThermoDataMutex);                                                            // end of lock mutex
                xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
            }

            init_status = false;
        }
        else
        {
            if (mb.Hreg(PWR_HREG) != last_PWR) // 火力pwr数值发生变动
            {
                if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                {
                    last_PWR = mb.Hreg(PWR_HREG); // last 火力pwr数据更新
                    heat_pwr_to_SSR = last_PWR;   // 发送新火力pwr数据到 SSR
                    // 合成HMI数据帧
                    make_frame_head(CMD_DATA_Buffer, 2); // 帧头
                    make_frame_data(CMD_DATA_Buffer, 2, last_PWR, 3);
                    make_frame_end(CMD_DATA_Buffer, 2);
                    xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);                           // 帧微
                    pwm_heat.write(HEAT_OUT_PIN, map(heat_pwr_to_SSR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
                    xSemaphoreGive(xThermoDataMutex);
                    xTaskNotify(xTASK_data_to_HMI, 0, eIncrement); // end of lock mutex
                }
            }
        }
        // HEAT
        if (mb.Hreg(HEAT_HREG) != digitalRead(HEAT_RLY)) // 风扇开关状态发生变动
        {
            digitalWrite(HEAT_RLY, !digitalRead(HEAT_RLY));           // 将artisan的控制值控制开关
            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 整合数据帧到HMI
            {
                make_frame_head(CMD_DATA_Buffer, 2);                           // 帧头
                make_frame_data(CMD_DATA_Buffer, 2, digitalRead(HEAT_RLY), 5); // 冷却扇状态数据//pin status has changed ,so read directly
                make_frame_end(CMD_DATA_Buffer, 2);                            // 帧微
                xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
            }
        }
        // FAN
        if (mb.Hreg(FAN_HREG) != digitalRead(FAN_RLY)) // 风扇开关状态发生变动
        {
            digitalWrite(FAN_RLY, !digitalRead(FAN_RLY));              // 将artisan的控制值控制开关
            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 整合数据帧到HMI
            {
                make_frame_head(CMD_DATA_Buffer, 2);                          // 帧头
                make_frame_data(CMD_DATA_Buffer, 2, digitalRead(FAN_RLY), 7); // 冷却扇状态数据//pin status has changed ,so read directly
                make_frame_end(CMD_DATA_Buffer, 2);                           // 帧微
                xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
            }
        }
    }
    vTaskDelay(20);
}
#endif

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