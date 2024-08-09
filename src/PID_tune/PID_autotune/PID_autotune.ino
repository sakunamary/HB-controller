/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/

#include <Arduino.h>
#include "config.h"
#include <MCP3424.h>
#include "DFRobot_AHT20.h"
#include <ESP32Servo.h>
#include <pidautotuner.h>
#include "SparkFun_External_EEPROM.h"  // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM

uint8_t MCP3424_address = 0x68;
long Voltage;  // Array used to store results

const uint32_t frequency = PWM_FREQ;
const byte resolution = PWM_RESOLUTION;  // pwm -0-1023
const byte pwm_heat_out = PWM_HEAT;

double BT_TEMP;
double AMB_RH;
double AMB_TEMP;
float PID_TUNE_SV = 180.0;
long prevMicroseconds;
long microseconds;
double pid_tune_output;
double pid_out_max = PID_MAX_OUT;  // 取值范围 （0-100）
double pid_out_min = PID_MIN_OUT;  // 取值范围 （0-100）

pid_setting_t pid_parm = {
  .pid_CT = 2,
  .p = 25.41,
  .i = 1.81,
  .d = 99.74,
  .BT_tempfix = 0.0,
  .ET_tempfix = 0.0,
  .inlet_tempfix = 0.0,
  .EX_tempfix = 0.0
};

static TaskHandle_t xTask_PID_autotune = NULL;
static TaskHandle_t xTask_Thermo_get_data = NULL;
SemaphoreHandle_t xThermoDataMutex = NULL;

ESP32PWM pwm_heat;

PIDAutotuner tuner = PIDAutotuner();
ExternalEEPROM I2C_EEPROM;
DFRobot_AHT20 aht20;
MCP3424 ADC_MCP3424(MCP3424_address);  // Declaration of MCP3424 A2=0 A1=1 A0=0

void Task_Thermo_get_data(void *pvParameters);
void Task_PID_autotune(void *pvParameters);

void setup() {

  // Prepare working .....
  xThermoDataMutex = xSemaphoreCreateMutex();
  ESP32PWM::allocateTimer(0);
  pinMode(SYSTEM_RLY, OUTPUT);
  pinMode(HEAT_RLY, OUTPUT);

  digitalWrite(SYSTEM_RLY, LOW);   // 初始化电路启动；
  digitalWrite(HEAT_RLY, LOW);     // 初始化电路启动；
  digitalWrite(SYSTEM_RLY, HIGH);  // 启动机器

  Serial.begin(BAUDRATE);
  aht20.begin();
  ADC_MCP3424.NewConversion();
  I2C_EEPROM.setMemoryType(64);

  // Init pwm output
  pwm_heat.attachPin(pwm_heat_out, frequency, resolution);  // 1KHz 8 bit
  pwm_heat.writeScaled(0.0);

  // part I :init setting
  Serial.println("start EEPROM setting ...");
  if (!I2C_EEPROM.begin()) {
    Serial.println("failed to initialise EEPROM");
    delay(1000000);
  } else {
    Serial.println("Initialed EEPROM,load data will be writen after 3s...");
    delay(3000);
    //loadUserSettings();
    I2C_EEPROM.get(0, pid_parm);
    Serial.printf("\nEEPROM value check ...\n");
    Serial.printf("\npid_CT:%d", pid_parm.pid_CT);
    Serial.printf("\nPID kp:%4.2f", pid_parm.p);
    Serial.printf("\nPID ki:%4.2f", pid_parm.i);
    Serial.printf("\nPID kd:%4.2f", pid_parm.d);
    Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
    Serial.printf("\nET fix:%4.2f", pid_parm.ET_tempfix);
    Serial.printf("\nInlet fix:%4.2f", pid_parm.inlet_tempfix);
    Serial.printf("\nEX fix:%4.2f", pid_parm.EX_tempfix);
  }

  xTaskCreate(
    Task_Thermo_get_data, "Thermo_get_data"  //
    ,
    1024 * 4  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 3  // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    &xTask_Thermo_get_data  // Running Core decided by FreeRTOS,let core0 run wifi and BT
  );
  Serial.printf("\nTASK=1:Thermo_get_data OK");

  xTaskCreate(
    Task_PID_autotune, "PID autotune"  //
    ,
    1024 * 6  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 2  // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    &xTask_PID_autotune  // Running Core decided by FreeRTOS,let core0 run wifi and BT
  );

  Serial.printf("\nTASK=2:PID autotune OK");

  // end of part I

  // partII run the PID autotune

  // INIT PID AUTOTUNE
  // read pid data from EEPROM
  tuner.setTargetInputValue(PID_TUNE_SV);
  tuner.setTuningCycles(5);
  tuner.setLoopInterval(pid_parm.pid_CT * uS_TO_S_FACTOR);                                   // interval in uS
  tuner.setOutputRange(map(pid_out_min, 0, 100, 0, 255), map(pid_out_max, 0, 100, 0, 255));  // 取值范围转换为（0-255）-> (76-205)
  tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);

  Serial.printf("\nPID Auto Tune will be started in 3 seconde...\n");
  vTaskDelay(3000);                                // 让pid关闭有足够时间执行
  xTaskNotify(xTask_PID_autotune, 0, eIncrement);  // 通知处理任务干活

  // end of Part II
}

void loop() {
}

void Task_Thermo_get_data(void *pvParameters) {  // function

  /* Variable Definition */
  (void)pvParameters;
  TickType_t xLastWakeTime;
  const TickType_t xIntervel = 3000 / portTICK_PERIOD_MS;
  /* Task Setup and Initialize */
  // Initial the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  // setup for the the SPI library:

  while (1) {  // for loop
    // Wait for the next cycle (intervel 2000ms).
    vTaskDelayUntil(&xLastWakeTime, xIntervel);

    if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS)  // 给温度数组的最后一个数值写入数据
    {
      if (aht20.startMeasurementReady(/* crcEn = */ true)) {
        AMB_TEMP = aht20.getTemperature_C();
        AMB_RH = aht20.getHumidity_RH();
      }
      vTaskDelay(50);
      ADC_MCP3424.Configuration(3, ADC_BIT, 1, 1);
      Voltage = ADC_MCP3424.Measure();
      BT_TEMP = pid_parm.BT_tempfix + (((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083));  // CH1 3001
      // Serial.printf("AMB_TEMP:%4.2f,BT_TEMP:%4.2f\n", AMB_TEMP, BT_TEMP);
      xSemaphoreGive(xThermoDataMutex);  // end of lock mutex
    }
  }
}  // function

void Task_PID_autotune(void *pvParameters) {
  (void)pvParameters;
  uint32_t ulNotificationValue;  // 用来存放本任务的4个字节的notification value
  BaseType_t xResult;
  const TickType_t xIntervel = 3000 / portTICK_PERIOD_MS;

  while (1) {
    xResult = xTaskNotifyWait(0x00,                  // 在运行前这个命令之前，先清除这几位
                              0x00,                  // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                              &ulNotificationValue,  // 重置前的notification value
                              portMAX_DELAY);        // 一直等待

    if (xResult == pdTRUE) {
      // 开始 PID自动整定

      for (int loop = 0; loop < 3; loop++) {
        if (loop == 0) {
          PID_TUNE_SV = PID_TUNE_SV_1;
          tuner.setTargetInputValue(PID_TUNE_SV);
          digitalWrite(HEAT_RLY, HIGH);  // 启动发热丝
          pwm_heat.writeScaled(0);
          vTaskDelay(1000);
          while (!tuner.isFinished())  // 开始自动整定循环
          {
            prevMicroseconds = microseconds;
            microseconds = micros();

            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS)  // 给温度数组的最后一个数值写入数据
            {
              pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);
              pwm_heat.write(map(pid_tune_output, 0, 255, 230, 850));
              // pwm_heat.write(HEAT_OUT_PIN, map(pid_tune_output, 0, 255, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
              xSemaphoreGive(xThermoDataMutex);  // end of lock mutex
            }
            Serial.printf("PID Auto Pharse 1 Tuneing...OUTPUT:%4.2f BT_temp:%4.2f AMB_TEMP:%4.2f\n", pid_tune_output, BT_TEMP, AMB_TEMP);
            //  This loop must run at the same speed as the PID control loop being tuned
            while (micros() - microseconds < pid_parm.pid_CT * uS_TO_S_FACTOR)  // time units : us
              delayMicroseconds(1);
          }
          // Turn the output off here.
          pwm_heat.writeScaled(0);
          // pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
          digitalWrite(HEAT_RLY, LOW);  // 启动发热丝
          // Get PID gains - set your PID controller's gains to these
          pid_parm.p = tuner.getKp();
          pid_parm.i = tuner.getKi();
          pid_parm.d = tuner.getKd();
          Serial.printf("\nPID Auto Tune Pharse I  Finished ...\n");
          Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
          Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
          Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
          Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
          Serial.printf("\nET fix:%4.2f", pid_parm.ET_tempfix);
          Serial.printf("\nInlet fix:%4.2f", pid_parm.inlet_tempfix);
          Serial.printf("\nEX fix:%4.2f", pid_parm.EX_tempfix);

          I2C_EEPROM.put(128, pid_parm);

          Serial.printf("\nPID parms Pharse 1  saved ...\n");
        } else if (loop == 1) {

          PID_TUNE_SV = PID_TUNE_SV_2;
          tuner.setTargetInputValue(PID_TUNE_SV);
          digitalWrite(HEAT_RLY, HIGH);  // 启动发热丝
          pwm_heat.writeScaled(0);
          vTaskDelay(1000);
          tuner.startTuningLoop(micros());
          while (!tuner.isFinished())  // 开始自动整定循环
          {
            prevMicroseconds = microseconds;
            microseconds = micros();

            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS)  // 给温度数组的最后一个数值写入数据
            {
              pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);
              pwm_heat.write(map(pid_tune_output, 0, 255, 230, 850));
              // pwm_heat.write(HEAT_OUT_PIN, map(pid_tune_output, 0, 255, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
              xSemaphoreGive(xThermoDataMutex);  // end of lock mutex
            }
            Serial.printf("PID Auto Pharse 2 Tuneing...OUTPUT:%4.2f BT_temp:%4.2f AMB_TEMP:%4.2f\n", pid_tune_output, BT_TEMP, AMB_TEMP);
            //  This loop must run at the same speed as the PID control loop being tuned
            while (micros() - microseconds < pid_parm.pid_CT * uS_TO_S_FACTOR)  // time units : us
              delayMicroseconds(1);
          }
          // Turn the output off here.
          pwm_heat.writeScaled(0);
          // pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
          digitalWrite(HEAT_RLY, LOW);  // 启动发热丝
          // Get PID gains - set your PID controller's gains to these
          pid_parm.p = tuner.getKp();
          pid_parm.i = tuner.getKi();
          pid_parm.d = tuner.getKd();
          Serial.printf("\nPID Auto Tune Pharse 2  Finished ...\n");
          Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
          Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
          Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
          Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
          Serial.printf("\nET fix:%4.2f", pid_parm.ET_tempfix);
          Serial.printf("\nInlet fix:%4.2f", pid_parm.inlet_tempfix);
          Serial.printf("\nEX fix:%4.2f", pid_parm.EX_tempfix);

          I2C_EEPROM.put(1, pid_parm);

          Serial.printf("\nPID parms Pharse 2 saved ...\n");
        } else if (loop == 2) {
          PID_TUNE_SV = PID_TUNE_SV_3;
          tuner.setTargetInputValue(PID_TUNE_SV);
          digitalWrite(HEAT_RLY, HIGH);  // 启动发热丝
          pwm_heat.writeScaled(0);
          vTaskDelay(1000);
          tuner.startTuningLoop(micros());
          while (!tuner.isFinished())  // 开始自动整定循环
          {
            prevMicroseconds = microseconds;
            microseconds = micros();

            if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS)  // 给温度数组的最后一个数值写入数据
            {
              pid_tune_output = tuner.tunePID(BT_TEMP, microseconds);
              pwm_heat.write(map(pid_tune_output, 0, 255, 230, 850));
              // pwm_heat.write(HEAT_OUT_PIN, map(pid_tune_output, 0, 255, 230, 850), frequency, resolution); // 输出新火力pwr到SSRÍ
              xSemaphoreGive(xThermoDataMutex);  // end of lock mutex
            }
            Serial.printf("PID Auto Pharse 3  Tuneing...OUTPUT:%4.2f BT_temp:%4.2f AMB_TEMP:%4.2f\n", pid_tune_output, BT_TEMP, AMB_TEMP);
            //  This loop must run at the same speed as the PID control loop being tuned
            while (micros() - microseconds < pid_parm.pid_CT * uS_TO_S_FACTOR)  // time units : us
              delayMicroseconds(1);
          }
          // Turn the output off here.
          pwm_heat.writeScaled(0);
          // pwm_heat.write(HEAT_OUT_PIN, 0, frequency, resolution);
          digitalWrite(HEAT_RLY, LOW);  // 启动发热丝
          // Get PID gains - set your PID controller's gains to these
          pid_parm.p = tuner.getKp();
          pid_parm.i = tuner.getKi();
          pid_parm.d = tuner.getKd();
          Serial.printf("\nPID Auto Tune Pharse 3 Finished ...\n");
          Serial.printf("\nPID kp:%4.2f\n", pid_parm.p);
          Serial.printf("\nPID ki:%4.2f\n", pid_parm.i);
          Serial.printf("\nPID kd:%4.2f\n", pid_parm.d);
          Serial.printf("\nBT fix:%4.2f", pid_parm.BT_tempfix);
          Serial.printf("\nET fix:%4.2f", pid_parm.ET_tempfix);
          Serial.printf("\nInlet fix:%4.2f", pid_parm.inlet_tempfix);
          Serial.printf("\nEX fix:%4.2f", pid_parm.EX_tempfix);

          I2C_EEPROM.put(256, pid_parm);

          Serial.printf("\nPID parms saved ...\n");
        }
      }
    }
    vTaskDelete(xTask_Thermo_get_data);
    vTaskDelete(NULL);
  }
}

