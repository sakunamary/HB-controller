## Arduino - I2C_EEPROM
Simple arduino library for reading and writing data from an I2C EEPROM chip (i.e. AT24C256). This library is based on the [arduino playground article](https://playground.arduino.cc/Code/I2CEEPROM)  about I2C EEPROM.

### Usage
1. Create instance
```c
#require <I2C_EEPROM.h>
I2C_EEPROM memory(0x50);
```

2. Initialize
```c
void setup()
{
  memory.init();
}
```

3. Use
  - Writing
```c
memory.write(0x01, 0x00);
byte data[] = {0x00, 0x01, 0x02, 0x04, 0x05, 0x06, 0x07};
memory.writeSerial(0x02, data, 7);
```
  - Reading
```c
byte readData = memory.read(0x01);
byte readDataSerial[7];
memory.readSerial(0x02, readDataSerial, 7);
```

## Functions
Constructor:
```c
I2C_EEPROM(byte address);
```
- ``byte address`` I2C address of chip

Write:
```c
void write(unsigned int address, byte data);
```
- ``unsigned int address`` address to write to
- ``byte data`` data to write

Write Serial:
```c
void writeSerial(unsigned int address, byte* data, byte length);
```
- ``unsigned int address`` address to write to
- ``byte* data`` data to write
- ``byte length`` data length

Read:
```c
byte read(unsigned int address);
```
- ``unsigned int address`` address to read from
- Returns ``byte``

Read Serial:
```c
void readSerial(unsigned int address, byte length, byte *buffer);
```
- ``unsigned int address`` address to read from
- ``byte length`` how many bytes
- ``byte *buffer`` write to this value

### License
This project is licensed under the Creative Commons license
