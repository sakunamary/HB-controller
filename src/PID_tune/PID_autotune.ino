/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/
#include <Arduino.h>
#include "EEPROM.h"
#include "config.h"
#include <pwmWrite.h>
#include <pidautotuner.h>
#include <Adafruit_MAX31865.h>

const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-1023

double BT_TEMP;
long prevMicroseconds;
long microseconds;
double pid_tune_output;
double pid_out_max = PID_MAX_OUT; // 取值范围 （0-100）
double pid_out_min = PID_MIN_OUT; // 取值范围 （0-100）
pid_setting_t pid_parm;
double temp[5] = {0};
double temp_;
int i, j;

static TaskHandle_t xTask_PID_autotune = NULL;
SemaphoreHandle_t xThermoDataMutex = NULL;

Pwm pwm_heat = Pwm();
PIDAutotuner tuner = PIDAutotuner();
Adafruit_MAX31865 thermo_BT = Adafruit_MAX31865(SPI_CS_BT, SPI_MOSI, SPI_MISO, SPI_SCK); // CH3

void Task_Thermo_get_data(void *pvParameters);
void Task_PID_autotune(void *pvParameters);

void setup()
{

    // Prepare working .....
    xThermoDataMutex = xSemaphoreCreateMutex();
    pinMode(SYSTEM_RLY, OUTPUT);
    pinMode(HEAT_RLY, OUTPUT);

    digitalWrite(SYSTEM_RLY, LOW);  // 初始化电路启动；
    digitalWrite(HEAT_RLY, LOW);    // 初始化电路启动；
    digitalWrite(SYSTEM_RLY, HIGH); // 启动机器

    Serial.begin(BAUDRATE);
    thermo_BT.begin(MAX31865_2WIRE); // set to 2WIRE or 4WIRE as necessary
                                     // thermo_BT.enableBias(true);
    //  Init pwm output
    pwm_heat.pause();
    pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
    pwm_heat.resume();
    pwm_heat.printDebug();

    // part I :init setting

        Serial.println("Initialed EEPROM,data will be writen after 3s...");
        delay(3000);
        pid_parm.pid_CT = 3 * uS_TO_S_FACTOR; // 10s. uinit is micros
        pid_parm.p = 3.58;
        pid_parm.i = 0.65;
        pid_parm.d = 13.5;
        pid_parm.BT_tempfix = 0.0;

    xTaskCreatePinnedToCore(
        Task_Thermo_get_data, "Thermo_get_data" //
        ,
        1024 * 4 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
    Serial.printf("\nTASK=1:Thermo_get_data OK");

    xTaskCreatePinnedToCore(
        Task_PID_autotune, "PID autotune" //
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        &xTask_PID_autotune, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );

    Serial.printf("\nTASK=2:PID autotune OK");

    // end of part I

    // partII run the PID autotune

    // INIT PID AUTOTUNE

    Serial.printf("\nEEPROM value check ...\n");
    Serial.printf("\npid_CT:%ld\n", pid_parm.pid_CT);
    Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
    Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
    Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
    Serial.printf("\nBT fix:%4.2f\n", pid_parm.BT_tempfix);

    tuner.setTargetInputValue(PID_TUNE_SV);
    tuner.setLoopInterval(pid_parm.pid_CT);
    tuner.setOutputRange(map(pid_out_min, 0, 100, 0, 255), map(pid_out_max, 0, 100, 0, 255)); // 取值范围转换为（0-255）-> (76-205)
    tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);

    Serial.printf("\nPID Auto Tune will be started in 3 seconde...\n");
    vTaskDelay(3000);                               // 让pid关闭有足够时间执行
    xTaskNotify(xTask_PID_autotune, 0, eIncrement); // 通知处理任务干活

    // end of Part II
}

void loop()
{
}

void Task_Thermo_get_data(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 3000 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // setup for the the SPI library:

    while (1)
    { // for loop
        // Wait for the next cycle (intervel 2000ms).
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        for (i = 0; i < 5; i++)
        {
            vTaskDelay(100);
            temp[i] = thermo_BT.temperature(RNOMINAL, RREF); // CH3
        }
        i = 0;
        for (i = 0; i < 5; i++)
        {
            for (j = i + 1; j < 5; j++)
            {
                if (temp[i] > temp[j])
                {
                    temp_ = temp[i];
                    temp[i] = temp[j];
                    temp[j] = temp_;
                }
            }
        }
        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            BT_TEMP = temp[2]; // for PID AUTO TUNE
            Serial.println(BT_TEMP);
            xSemaphoreGive(xThermoDataMutex); // end of lock mutex
        }
    }
} // function

void Task_PID_autotune(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    const TickType_t xIntervel = 3000 / portTICK_PERIOD_MS;

    while (1)
    {
        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待

        if (xResult == pdTRUE)
        {
            // 开始 PID自动整定
            digitalWrite(HEAT_RLY, HIGH); // 启动发热丝

            pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution); // 输出新火力pwr到SSRÍ
            vTaskDelay(1000);
            while (!tuner.isFinished()) // 开始自动整定循环
            {
                prevMicroseconds = microseconds;
                microseconds = micros();

                if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
                {
                    pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);
                    pwm_heat.write(HEAT_OUT_PIN, map(pid_tune_output, 0, 255, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
                    xSemaphoreGive(xThermoDataMutex);                                                            // end of lock mutex
                }
                // Serial.printf("\nPID Auto Tuneing...OUTPUT:%4.2f BT_temp:%4.2f\n", pid_tune_output, BT_TEMP);
                //  This loop must run at the same speed as the PID control loop being tuned
                while (micros() - microseconds < pid_parm.pid_CT)
                    delayMicroseconds(1);
            }
            // Turn the output off here.
            pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
            digitalWrite(HEAT_RLY, LOW); // 启动发热丝
            // Get PID gains - set your PID controller's gains to these
            pid_parm.p = tuner.getKp();
            pid_parm.i = tuner.getKi();
            pid_parm.d = tuner.getKd();
            Serial.printf("\nPID Auto Tune Finished ...\n");
            Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
            Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
            Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
            Serial.printf("\nPID parms saved ...\n");
        }
    }
    vTaskDelete(NULL);
}