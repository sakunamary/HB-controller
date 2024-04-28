/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/

#define LOCATION_SETTINGS 0

#include <Arduino.h>
#include "config.h"
#include <MCP3424.h>
#include "DFRobot_AHT20.h"
#include <pwmWrite.h>
#include <pidautotuner.h>
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM

uint8_t MCP3424_address = 0x68;
long Voltage;                      // Array used to store results
const int HEAT_OUT_PIN = PWM_HEAT; // GPIO26
const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION; // pwm -0-1023

double BT_TEMP;
double AMB_RH;
double AMB_TEMP;

long prevMicroseconds;
long microseconds;
double pid_tune_output;
double pid_out_max = PID_MAX_OUT; // 取值范围 （0-100）
double pid_out_min = PID_MIN_OUT; // 取值范围 （0-100）

pid_setting_t pid_parm = {
    .pid_CT = 3,
    .p = 3.0,
    .i = 0.12,
    .d = 33.0,
    .BT_tempfix = 0.0,
    .ET_tempfix = 0.0,
    .inlet_tempfix = 0.0,
    .EX_tempfix = 0.0};

double temp[5] = {0};
double temp_;
int i, j;

static TaskHandle_t xTask_PID_autotune = NULL;
SemaphoreHandle_t xThermoDataMutex = NULL;

Pwm pwm_heat = Pwm();
PIDAutotuner tuner = PIDAutotuner();
ExternalEEPROM I2C_EEPROM;

void Task_Thermo_get_data(void *pvParameters);
void Task_PID_autotune(void *pvParameters);
void loadUserSettings();

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
    aht20.begin();
    ADC_MCP3424.NewConversion();
    I2C_EEPROM.setMemoryType(32);

    //  Init pwm output
    pwm_heat.pause();
    pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
    pwm_heat.resume();
    pwm_heat.printDebug();

    // part I :init setting
    Serial.println("start EEPROM setting ...");
    if (!I2C_EEPROM.begin())
    {
        Serial.println("failed to initialise EEPROM");
        delay(1000000);
    }
    else
    {
        Serial.println("Initialed EEPROM,load data will be writen after 3s...");
        delay(3000);
        loadUserSettings();

        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("\npid_CT:%d\n", pid_parm.pid_CT);
        Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
        Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
        Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
        Serial.printf("\nBT fix:%4.2f\n", pid_parm.BT_tempfix);
        Serial.printf("\nET fix:%4.2f\n", pid_parm.ET_tempfix);
        Serial.printf("\nInlet fix:%4.2f\n", pid_parm.inlet_tempfix);
        Serial.printf("\nEX fix:%4.2f\n", pid_parm.EX_tempfix);
    }

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
    // read pid data from EEPROM

    tuner.setTargetInputValue(PID_TUNE_SV);
    tuner.setLoopInterval(pid_parm.pid_CT * uS_TO_S_FACTOR); //interval in uS
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

        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // 给温度数组的最后一个数值写入数据
        {
            if (aht20.startMeasurementReady(/* crcEn = */ true))
            {
                AMB_TEMP = aht20.getTemperature_C();
                AMB_RH = aht20.getHumidity_RH();
            }
            ADC_MCP3424.Configuration(3, ADC_BIT, 1, 1);
            Voltage = ADC_MCP3424.Measure();
            BT_TEMP = ((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083); // CH1 3001

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

            I2C_EEPROM.put(LOCATION_SETTINGS, pid_parm);

            Serial.printf("\nPID parms saved ...\n");
        }
    }
    vTaskDelete(NULL);
}

// Load the current settings from EEPROM into the settings struct
void loadUserSettings()
{
    // Uncomment these lines to forcibly erase the EEPROM and see how the defaults are set
    // Serial.println("Erasing EEPROM");
    // myMem.erase();

    // Check to see if EEPROM is blank. If the first four spots are zeros then we can assume the EEPROM is blank.
    uint32_t testRead = 0;
    if (I2C_EEPROM.get(LOCATION_SETTINGS, testRead) == 0) // EEPROM address to read, thing to read into
    {
        // At power on, settings are set to defaults within the struct.
        // So go record the struct as it currently exists so that defaults are set.
        I2C_EEPROM.put(LOCATION_SETTINGS, pid_parm);
        Serial.println("Default settings applied");
    }
    else
    {
        // Read current settings
        I2C_EEPROM.get(LOCATION_SETTINGS, pid_parm);
    }
}
