#ifndef I2C_EEPROM_H
#define I2C_EEPROM_H

#define EEPROM_DELAY 3

class I2C_EEPROM{
  public:
    I2C_EEPROM(byte address);
    void init();
    void write(unsigned int address, byte data); //write single byte to EEPROM
    void writeSerial(unsigned int address, byte* data, byte length); //write multiple bytes to EEPROM
    byte read(unsigned int address); //read single byte from EEPROM
    void readSerial(unsigned int address, byte length, byte *buffer); //read multiple bytes to EEPROM
  private:
    byte _address;
};

#endif
