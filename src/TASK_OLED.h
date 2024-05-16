#ifndef __TASK_OLED_H__
#define __TASK_OLED_H__

#include <Arduino.h>
#include "config.h"

#include <Arduino.h>
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SSD1306Wire display(SCREEN_ADDRESS, SDA, SCL); // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h

static char buffer[64];
extern double BT_TEMP;
extern double ET_TEMP;
extern double INLET_TEMP;
extern double EX_TEMP;

void Task_OLED(void *pvParameters)
{ // function

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    String ver = VERSION;

    display.init();
    display.flipScreenVertically();
    display.setContrast(255);

    // Show initial display buffer contents on the screen --

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(86, 0 + 2, ver);
    display.display();

    for (;;) // A Task shall never return or exit.
    {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        display.clear();
        display.setFont(ArialMT_Plain_10);

        if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) // Mutex to make the data more clean
        {
            display.drawStringf(2 + 16, 0 + 2, buffer,  "BT:  %4.2f", BT_TEMP);
            display.drawStringf(2 + 16, 16 + 2, buffer, "IN:  %4.2f", INLET_TEMP);
            display.drawStringf(2 + 16, 32 + 2, buffer, "EX:  %4.2f", EX_TEMP);
            display.drawStringf(2 + 16, 48 + 2, buffer, "ET:  %4.2f", ET_TEMP);
            display.display();
            xSemaphoreGive(xThermoDataMutex);
        }

    }
}

#endif