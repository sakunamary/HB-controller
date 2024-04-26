#include <Wire.h>
#include "Arduino.h"
#include "I2C_EEPROM.h"

I2C_EEPROM::I2C_EEPROM(byte address){
  _address = address;
}

void I2C_EEPROM::init(){
  Wire.begin();
}

void I2C_EEPROM::write(unsigned int address, byte data){
  Wire.beginTransmission(_address);
  Wire.write((int)(address >> 8));
  Wire.write((int)(address & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  delay(EEPROM_DELAY);
}

void I2C_EEPROM::writeSerial(unsigned int address, byte* data, byte length){
  Wire.beginTransmission(_address);
  Wire.write((int)(address >> 8));
  Wire.write((int)(address & 0xFF));
  
  for(byte i = 0; i < length; i++){
    Wire.write(data[i]);
  }
  
  Wire.endTransmission();
  delay(EEPROM_DELAY);
}

byte I2C_EEPROM::read(unsigned int address){
  byte data = 0xFF;
  Wire.beginTransmission(_address);
  Wire.write((int)(address >> 8));
  Wire.write((int)(address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom((int)_address, 1);
  
  if(Wire.available()){
    data = Wire.read();
  }
  delay(EEPROM_DELAY);
  return data;
}

void I2C_EEPROM::readSerial(unsigned int address, byte length, byte *buffer){
  Wire.beginTransmission(_address);
  Wire.write((int)(address >> 8));
  Wire.write((int)(address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(_address, length);
  
  for(int i = 0; i < length; i++){
    if(Wire.available()){
      buffer[i] = Wire.read();
    }
  }
  delay(EEPROM_DELAY);
}
