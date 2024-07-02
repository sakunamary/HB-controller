/*
   EEPROM Write

   Stores random values into the EEPROM.
   These values will stay in the EEPROM when the board is
   turned off and may be retrieved later by another sketch.
*/

#define LOCATION_SETTINGS 0

#include <Arduino.h>
#include "config.h"
#include "Wire.h"
#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM

pid_setting_t pid_parm = {
    .pid_CT = 3 * uS_TO_S_FACTOR,
    .p = 3.023,
    .i = 0.12,
    .d = 33.0,
    .BT_tempfix = 1.0,
    .ET_tempfix = 1.5,
    .inlet_tempfix = 2.5,
    .EX_tempfix = 1.34};

ExternalEEPROM I2C_EEPROM;

void loadUserSettings();

void setup()
{
        delay(5000);
    // Prepare working .....
    Serial.begin(BAUDRATE);
    Wire.begin();
    I2C_EEPROM.setMemoryType(64);

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
        I2C_EEPROM.put(LOCATION_SETTINGS, pid_parm);
        Serial.println("EEPROM,load data for check after 3s...");

        delay(3000);
        loadUserSettings();
        Serial.printf("\nEEPROM value check ...\n");
        Serial.printf("pid_CT:%d\n", pid_parm.pid_CT);
        Serial.printf("PID kp:%4.2f\n", pid_parm.p);
        Serial.printf("PID ki:%4.2f\n", pid_parm.i);
        Serial.printf("PID kd:%4.2f\n", pid_parm.d);
        Serial.printf("BT fix:%4.2f\n", pid_parm.BT_tempfix);
        Serial.printf("ET fix:%4.2f\n", pid_parm.ET_tempfix);
        Serial.printf("Inlet fix:%4.2f\n", pid_parm.inlet_tempfix);
        Serial.printf("EX fix:%4.2f\n", pid_parm.EX_tempfix);
    }
}

void loop()
{
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
