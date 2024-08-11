/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/

#include <Arduino.h>
#include "config.h"
#include "Wire.h"
#include <MCP3424.h>
#include "TypeK.h"
#include "DFRobot_AHT20.h"
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM

// Need this for the lower level access to set them up.
uint8_t address = 0x68;
int i, j;
double temp_cal_tmp[5];
double temp_;
long Voltage; // Array used to store results

double BT_TEMP;
double ET_TEMP;
double INLET_TEMP;
double EX_TEMP;
double AMB_TEMP;

MCP3424 ADC_MCP3424(address); // Declaration of MCP3424 A2=0 A1=1 A0=0
DFRobot_AHT20 aht20;
pid_setting_t pid_parm = {
    .pid_CT = 2,
    .p = 7.42,
    .i = 0.46,
    .d = 79.48,
    .BT_tempfix = 0.0,
    .ET_tempfix = 0.0,
    .inlet_tempfix = 0.0,
    .EX_tempfix = 0.0};

ExternalEEPROM I2C_EEPROM;
TypeK temp_K_cal;

void setup()
{
    delay(1000);
    // Prepare working .....
    pinMode(SYSTEM_RLY, OUTPUT);
    digitalWrite(SYSTEM_RLY, LOW);  // 初始化电路启动；
    digitalWrite(SYSTEM_RLY, HIGH); // 启动机器
    Serial.begin(BAUDRATE);
    Wire.begin();
    I2C_EEPROM.setMemoryType(64);

    // Serial.println("start...\n");

    // ADC_MCP3424.NewConversion(); // New conversion is initiated
    // aht20.begin();

    // Serial.println("Pharse I:Sensor init\n");
    // if (aht20.startMeasurementReady(/* crcEn = */ true))
    // {
    //     AMB_TEMP = aht20.getTemperature_C();
    // }
    // Serial.println("Pharse I:OK\n");
    // Serial.println("Pharse II:cal temp fix data in 3s ....\n");
    // vTaskDelay(3000);
    // Serial.println("Temp raw Reading ....\n");
    // vTaskDelay(50);
    // ADC_MCP3424.Configuration(2, ADC_BIT, 1, 8);                            // MCP3424 is configured to channel i with 18 bits resolution, continous mode and gain defined to 8
    // Voltage = ADC_MCP3424.Measure();                                        // Measure is stocked in array Voltage, note that the library will wait for a completed conversion that takes around 200 ms@18bits
    // EX_TEMP = temp_K_cal.Temp_C(Voltage * 0.001, aht20.getTemperature_C()); // CH2

    // vTaskDelay(50);
    // ADC_MCP3424.Configuration(1, ADC_BIT, 1, 1);
    // Voltage = ADC_MCP3424.Measure();
    // INLET_TEMP = ((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083); // CH1  0.0039083

    // vTaskDelay(50);
    // ADC_MCP3424.Configuration(3, ADC_BIT, 1, 1);
    // Voltage = ADC_MCP3424.Measure();
    // BT_TEMP = ((Voltage / 1000 * RNOMINAL) / ((3.3 * 1000) - Voltage / 1000) - RREF) / (RREF * 0.0039083); // CH3

    // Serial.printf("Temp raw:: AMB_TEMP:%4.2f;BT:%4.2f; INLET:%4.2f;EX_TEMP:%4.2f\n", AMB_TEMP, BT_TEMP, INLET_TEMP, EX_TEMP);

    // pid_parm.BT_tempfix = AMB_TEMP - BT_TEMP;
    // pid_parm.inlet_tempfix = AMB_TEMP - INLET_TEMP;
    // pid_parm.EX_tempfix = AMB_TEMP - EX_TEMP;

    // Serial.printf("Temp fix::BT fix:%4.2f; inlet fix:%4.2f; ex fix:%4.2f\n", pid_parm.BT_tempfix, pid_parm.inlet_tempfix, pid_parm.EX_tempfix);
    // Serial.println("Pharse II:Done\n");

    // Serial.println("Pharse III: Write data into EEPROM...\n");

    // // part I :init setting
    // Serial.println("start EEPROM setting ...");
    // !I2C_EEPROM.begin();

    // Serial.println("Initialed EEPROM,load data will be writen after 3s...");
    // I2C_EEPROM.put(0, pid_parm);
    // I2C_EEPROM.put(128, pid_parm);
    // I2C_EEPROM.put(256, pid_parm);
    // Serial.println("EEPROM,load data for check after 3s...");
    // delay(3000);
    I2C_EEPROM.get(0, pid_parm);
    Serial.printf("\nEEPROM value 1 check ...\n");
    Serial.printf("pid_CT:%4.2f\n", pid_parm.pid_CT);
    Serial.printf("PID kp:%4.2f\n", pid_parm.p);
    Serial.printf("PID ki:%4.2f\n", pid_parm.i);
    Serial.printf("PID kd:%4.2f\n", pid_parm.d);
    Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
    Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
    Serial.printf("Inlet fix:%4.2f\n", pid_parm.inlet_tempfix);
    Serial.printf("EX fix:%4.2f\n", pid_parm.EX_tempfix);
   delay(1000);
    I2C_EEPROM.get(128, pid_parm);
    Serial.printf("\nEEPROM value 2 check ...\n");
    Serial.printf("pid_CT:%4.2f\n", pid_parm.pid_CT);
    Serial.printf("PID kp:%4.2f\n", pid_parm.p);
    Serial.printf("PID ki:%4.2f\n", pid_parm.i);
    Serial.printf("PID kd:%4.2f\n", pid_parm.d);
    Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
    Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
    Serial.printf("Inlet fix:%4.2f\n", pid_parm.inlet_tempfix);
    Serial.printf("EX fix:%4.2f\n", pid_parm.EX_tempfix);
   delay(1000);
    I2C_EEPROM.get(256, pid_parm);
    Serial.printf("\nEEPROM value 3 check ...\n");
    Serial.printf("pid_CT:%4.2f\n", pid_parm.pid_CT);
    Serial.printf("PID kp:%4.2f\n", pid_parm.p);
    Serial.printf("PID ki:%4.2f\n", pid_parm.i);
    Serial.printf("PID kd:%4.2f\n", pid_parm.d);
    Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
    Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
    Serial.printf("Inlet fix:%4.2f\n", pid_parm.inlet_tempfix);
    Serial.printf("EX fix:%4.2f\n", pid_parm.EX_tempfix);
}

void loop()
{
}
