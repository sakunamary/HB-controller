#ifndef __TASK_MODBUS_CONTROL_H__
#define __TASK_MODBUS_CONTROL_H__

// const uint16_t HEAT_HREG = 3005;
// const uint16_t SV_HREG = 3006;
// const uint16_t PID_HREG = 3007;
// const uint16_t PID_P_HREG = 3008;
// const uint16_t PID_I_HREG = 3009;
// const uint16_t PID_D_HREG = 30110;

uint16_t last_HEAT;
uint16_t last_SV;
uint16_t last_PID_P;
uint16_t last_PID_I;
uint16_t last_PID_D;



void Task_modbus_control(void *pvParameters)
{ //function 

    /* Variable Definition */
    (void)pvParameters;
    TickType_t xLastWakeTime;

    const TickType_t xIntervel = 100 / portTICK_PERIOD_MS;
    /* Task Setup and Initialize */
    // Initial the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    { //for loop





    }

}

#endif