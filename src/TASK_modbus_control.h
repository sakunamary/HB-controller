#ifndef __TASK_MODBUS_CONTROL_H__
#define __TASK_MODBUS_CONTROL_H__

#include <Arduino.h>
#include <config.h>
#include "ArduPID.h"
#include <pwmWrite.h>
#include <pidautotuner.h>

HardwareSerial Serial_HMI(2);      // D16 RX_drumer  D17 TX_drumer
const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-4096

Pwm pwm_heat = Pwm();
ArduPID Heat_pid_controller;
PIDAutotuner tuner = PIDAutotuner();

uint16_t last_PWR;
const uint16_t PWR_HREG = 3005;        // HEAT PWR
const uint16_t FAN_HREG = 3006;        // COOLING FAN SWITCH
const uint16_t HEAT_HREG = 3007;       // HEAT SWTICH
const uint16_t PID_SV_HREG = 3008;     // PID SV
const uint16_t PID_STATUS_HREG = 3009; // PID RUNNING STATUS

int heat_pwr_to_SSR = 0;
bool init_status = true;
bool pid_status = false;
long prevMicroseconds;
long microseconds;

double PID_output; // 取值范围 （0-255）
double pid_sv = 0;
double pid_tune_output;
double pid_out_max = PID_MAX_OUT; // 取值范围 （0-255）
double pid_out_min = PID_MIN_OUT; // 取值范围 （0-255）

void Task_modbus_control(void *pvParameters)
{ // function
    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    uint8_t CMD_DATA_Buffer[BUFFER_SIZE];
    const TickType_t xIntervel = 200 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (1) // A Task shall never return or exit.
    {         // for loop
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        // HEAT_HREG
        if (init_status) // 初始化状态
        {
            //
            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) //
            {
                last_PWR = mb.Hreg(PWR_HREG);
                heat_pwr_to_SSR = last_PWR;
                pid_sv = 0;
                mb.Hreg(PID_SV_HREG, 0);
                // 合成HMI数据帧
                // make_frame_head(CMD_DATA_Buffer, 2); // 帧头
                // make_frame_data(CMD_DATA_Buffer, 2, last_PWR, 3);
                // make_frame_end(CMD_DATA_Buffer, 2); // 帧微
                // xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                pwm_heat.write(HEAT_OUT_PIN, map(heat_pwr_to_SSR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
                xSemaphoreGive(xThermoDataMutex);                                                            // end of lock mutex
                // xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
            }

            init_status = false;
        }
        else
        {
            if (mb.Hreg(PID_STATUS_HREG) == 1) // pid 开启
            {
                if (pid_status == false)
                {                                                              // pid_status = false and pid_status_hreg =1
                    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                    {
                        pid_status = true; // update value
                        pid_sv = mb.Hreg(PID_SV_HREG) / 10;
                        Heat_pid_controller.start();
                        Heat_pid_controller.compute();                     // 计算pid输出
                        heat_pwr_to_SSR = map(PID_output, 0, 255, 0, 100); // 转换为格式 pid_output (0,255) -> (0,100)
                        last_PWR = heat_pwr_to_SSR;
                        xSemaphoreGive(xThermoDataMutex);
                    }
                }
                else
                {                                                              // pid_status = true and pid_status_hreg =1
                    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                    {
                        pid_sv = mb.Hreg(PID_SV_HREG) / 10;
                        Heat_pid_controller.compute();                     // 计算pid输出
                        heat_pwr_to_SSR = map(PID_output, 0, 255, 0, 100); // 转换为格式 pid_output (0,255) -> (0,100)
                        last_PWR = heat_pwr_to_SSR;
                        xSemaphoreGive(xThermoDataMutex);
                    }
                }
            }
            else
            {
                if (pid_status == true)
                {                                                              // pid_status = true and pid_status_hreg =0
                    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                    {
                        Heat_pid_controller.stop();
                        pid_sv = 0;
                        mb.Hreg(PID_SV_HREG, 0);
                        pid_status = false;
                        heat_pwr_to_SSR = last_PWR;
                        mb.Hreg(PWR_HREG, heat_pwr_to_SSR);
                        xSemaphoreGive(xThermoDataMutex);
                    }
                }
                else
                {                                      // pid_status = false and pid_status_hreg =0
                    if (mb.Hreg(PWR_HREG) != last_PWR) // 火力pwr数值发生变动
                    {
                        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                        {
                            last_PWR = mb.Hreg(PWR_HREG); // last 火力pwr数据更新
                            heat_pwr_to_SSR = last_PWR;   // 发送新火力pwr数据到 SSR
                            //  合成HMI数据帧
                            // make_frame_head(CMD_DATA_Buffer, 2); // 帧头
                            // make_frame_data(CMD_DATA_Buffer, 2, last_PWR, 3);
                            // make_frame_end(CMD_DATA_Buffer, 2);
                            // xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);                    // 帧微
                            xSemaphoreGive(xThermoDataMutex);
                            // xTaskNotify(xTASK_data_to_HMI, 0, eIncrement); // end of lock mutex
                        }
                    }
                }
            }
            pwm_heat.write(HEAT_OUT_PIN, map(heat_pwr_to_SSR, 0, 100, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
        }
        vTaskDelay(20);
        /////////////////////////////////////////////////////////////////////////////
        // HEAT
        if (mb.Hreg(HEAT_HREG) != digitalRead(HEAT_RLY)) // heater开关状态发生变动
        {

            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 整合数据帧到HMI
            {
                digitalWrite(HEAT_RLY, mb.Hreg(HEAT_HREG)); // 将artisan的控制值控制开关
                // make_frame_head(CMD_DATA_Buffer, 2);                            // 帧头
                // make_frame_data(CMD_DATA_Buffer, 2, digitalRead(HEAT_RLY), 5); // 冷却扇状态数据//pin status has changed ,so read directly
                // make_frame_end(CMD_DATA_Buffer, 2);                             // 帧尾
                // xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                // xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
            }
        }
        vTaskDelay(20);
        // FAN
        if (mb.Hreg(FAN_HREG) != digitalRead(FAN_RLY)) // 风扇开关状态发生变动
        {

            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 整合数据帧到HMI
            {
                digitalWrite(FAN_RLY, mb.Hreg(FAN_HREG)); // 将artisan的控制值控制开关
                // make_frame_head(CMD_DATA_Buffer, 2);                           // 帧头
                // make_frame_data(CMD_DATA_Buffer, 2, digitalRead(FAN_RLY), 7); // 冷却扇状态数据//pin status has changed ,so read directly
                // make_frame_end(CMD_DATA_Buffer, 2);                            // 帧尾
                // xQueueSendToFront(queue_data_to_HMI, &CMD_DATA_Buffer, xIntervel);
                xSemaphoreGive(xThermoDataMutex); // end of lock mutex
                // xTaskNotify(xTASK_data_to_HMI, 0, eIncrement);
            }
        }
        // PID auto tune triggered
        if (mb.Hreg(PID_STATUS_HREG) != 0)
        {
            xTaskNotify(xTask_PID_autotune, 0, eIncrement); // 通知处理任务干活
        }
    }
    vTaskDelay(20);
}

void Task_PID_autotune(void *pvParameters)
{
    (void)pvParameters;
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
            vTaskSuspend(xTask_modbus_control);
            vTaskSuspend(xTASK_HMI_CMD_handle);
            vTaskSuspend(xTASK_CMD_HMI);
            mb.Hreg(PID_STATUS_HREG, 0); // 关闭 pid
            Serial.printf("\nPID Auto Tune will be started in 5 seconde...\n");
            vTaskDelay(5000); // 让pid关闭有足够时间执行

            while (!tuner.isFinished()) // 开始自动整定
            {
                prevMicroseconds = microseconds;
                microseconds = micros();
                pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);

                pwm_heat.write(HEAT_OUT_PIN, map(pid_tune_output, round(PID_MIN_OUT * 255 / 100), round(PID_MAX_OUT * 255 / 100), 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
                // This loop must run at the same speed as the PID control loop being tuned
                while (micros() - microseconds < pid_parm.pid_CT)
                    delayMicroseconds(1);
            }

            // Turn the output off here.
            pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
            mb.Hreg(HEAT_HREG, 0);
            digitalWrite(HEAT_RLY, mb.Hreg(HEAT_HREG));
            // Get PID gains - set your PID controller's gains to these
            pid_parm.p = tuner.getKp();
            pid_parm.i = tuner.getKi();
            pid_parm.d = tuner.getKd();

            Serial.printf("\nPID Auto Tune Finished ...\n");
            Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
            Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
            Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);

            EEPROM.put(0, pid_parm);
            EEPROM.commit();
            Serial.printf("\nPID parms saved ...\n");
            vTaskResume(xTask_modbus_control);
            vTaskResume(xTASK_HMI_CMD_handle);
            vTaskResume(xTASK_CMD_HMI);
        }
    }
}

#endif

// HB --> HMI的控制状态帧 FrameLenght = 16
// 帧头: 69 FF
// 类型:02控制数据
// 火力: 00 00 // uint16
// 火力开关: 00 00 // uint16
// 冷却开关: 00 00 // uint16
// PID SV : 00 00 // uint16
// PID_STATUS: 00 // uint8
// PID_TUNE :00
// 帧尾:FF FF FF
// 温度为小端模式   dec 2222  hex AE 08
