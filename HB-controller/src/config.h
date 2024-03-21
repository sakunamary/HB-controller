
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define VERSION "1.0.0"
#define DEBUG_MODE
// uncomment to make work for HB M6SE ,default is work for HB M2SE
// #define MODEL_M6S
// uncomment to switch to MODBUS RTU   ,default is work in MODBUS TCP MODE
// #define MODBUS_RTU
#define SLAVE_ID 1 // MODBUS RTU SLAVE ID

// pwm setting
#define PWM_FREQ 10000
#define PWM_RESOLUTION 10 // 0-1024

///////////////////////////////////////
//   DO NOT make any change below    //
///////////////////////////////////////
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate
#define BAUD_HMI 9600
// pinout define
#define ENC_BUTTON 34
#define ENC_CLK 32
#define ENC_DT 35

#define SPI_MISO 19
#define SPI_SCK 18
#define SPI_MOSI 23

#define SPI_CS_INLET 27 // ch1 OK
#define SPI_CS_EX 14    // ch2 OK
#define SPI_CS_BT 26    // ch3 OK
#define SPI_CS_ET 25    // ch4 OK

#define SYSTEM_RLY 22
#define FAN_RLY 21
#define HEAT_RLY 12
#define PWM_HEAT 33

#define HMI_TX 17
#define HMI_RX 16

const int BUFFER_SIZE = 16;

// parametsrs of MAX31865
#define RREF 430.0
#define RNOMINAL 100.0


// publc funciton

uint8_t make_frame_head(uint8_t data_array[BUFFER_SIZE], int cmd_type)
// pagkage the data frame end .cmd_type:1/data_frame;2/run_status;3/HMI_cmd
{
    data_array[0] = 0x69; // frame head
    data_array[1] = 0xff; // frame head

    switch (cmd_type)
    {
    case 1:                   // data_frame
        data_array[2] = 0x01; // data type
        break;
    case 2:                   // run_status
        data_array[2] = 0x02; // data type
        break;
    case 3:                   // HMI_cmd
        data_array[2] = 0x03; // data type
        break;
    default:
        break;
    }
    return data_array[BUFFER_SIZE];
}

uint8_t make_frame_end(uint8_t data_array[BUFFER_SIZE], int cmd_type)
// pagkage the data frame end .cmd_type:1/data_frame;2/run_status;3/HMI_cmd
{

    switch (cmd_type)
    {
    case 1:                    // data_frame
        data_array[11] = 0xff; // frame end
        data_array[12] = 0xff; // frame end
        data_array[13] = 0xff; // frame end
        break;
    case 2:                   // run_status
        data_array[6] = 0xff; // frame end
        data_array[7] = 0xff; // frame end
        data_array[8] = 0xff; // frame end
        break;
    case 3:                   // HMI_cmd
        data_array[6] = 0xff; // frame end
        data_array[7] = 0xff; // frame end
        data_array[8] = 0xff; // frame end
        break;
    default:
        break;
    }
    return data_array[BUFFER_SIZE];
}

uint8_t make_frame_data(uint8_t data_array[BUFFER_SIZE], int cmd_type, uint16_t in_val, int uBit)
// pagkage the data frame.cmd_type:1/data_frame;2/run_status;3/HMI_cmd
{
    uint8_t high = highByte(in_val);
    uint8_t low = lowByte(in_val);
    switch (cmd_type)
    {
    case 1:
        if (uBit > 2 && uBit < 13)
        {
            data_array[uBit] = low;      // frame end
            data_array[uBit + 1] = high; // frame end
        }

        break;
    case 2:
        if (uBit > 2 && uBit < 6)
        {
            data_array[uBit] = low; // frame end
        }
        break;
    case 3:
        if (uBit > 2 && uBit < 6)
        {
            data_array[uBit] = low; // frame end
        }
        break;

    default:
        break;
    }
    return data_array[BUFFER_SIZE];
}

#endif

// HB --> HMI的数据帧 FrameLenght = 14
// 帧头: 69 FF
// 类型: 01温度数据
// BT: 00 00 // uint16
// Inlet: 00 00 // uint16
// EX: 00 00 // uint16
// ET: 00 00 // uint16
// 帧尾:FF FF FF

// HB --> HMI的控制状态帧 FrameLenght = 9
// 帧头: 67 FF
// 类型:02控制数据
// 火力: 00  // uint8
// 火力开关: 00 // uint8
// 冷却开关: 00 // uint8
// 帧尾:FF FF FF

// HMI --> HB的 命令帧 FrameLenght = 9
// 帧头: 67 FF
// 类型:03 控制数据
// 火力: 00  // uint8
// 火力开关: 00 // uint8
// 冷却开关: 00 // uint8
// 帧尾:FF FF FF

// 温度为小端模式   dec 2222  hex AE 08