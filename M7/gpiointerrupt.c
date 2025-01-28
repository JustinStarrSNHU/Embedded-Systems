/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// DISPLAY macro
#define DISPLAY(x) UART2_write(uart, &output, x, NULL);

// Timer Driver handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

// UART Global Variables
char output[64];
int bytesToSend;

// UART Driver Handles - Global variables
UART2_Handle uart;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                { 0x41, 0x0000, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global Variables
I2C_Handle i2c;

// task states and TickFct declarations
enum BUTTON_FLAG_States {BUTTON_FLAG_INIT, BUTTON_FLAG_0, BUTTON_FLAG_1};
int TickFct_CheckButtonState(int);

enum TEMPERATURE_States {TEMPERATURE_INIT, TEMPERATURE_READ};
int TickFct_CheckTemperatureState(int);

enum LED_states {LED_INIT, LED_ON, LED_OFF};
int TickFct_CheckLEDState(int);

enum OUTPUT_States {OUTPUT_INIT, OUTPUT_SEND};
int TickFct_CheckOutputState(int);

// task definition
typedef struct task {
    unsigned long taskPeriod;
    unsigned long elapsedTime;
    int state;
    int (*TickFct) (int);
} task;

// Global variables for tasks
task tasks[4];
const unsigned char tasksNum = 4;

const unsigned long tasksPeriodGCD = 100;

const unsigned long buttonPeriod = 200;
const unsigned long temperaturePeriod = 500;
const unsigned long ledPeriod = 500;
const unsigned long outputPeriod = 1000;

int16_t currentTemperature = 0;
int setPoint = 15;
unsigned char heat = 0;
int seconds = 0;

// function for the timer callback
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

// Function to initialize the timer
void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        // Failed to initialize timer
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        // Failed to start timer
        while (1) {}
    }
}

// function to initialize the UART
void initUART(void)
{
    UART2_Params uartParams;

    // Configure the driver
    UART2_Params_init(&uartParams);
    //uartParams.writeDataMode = UART_DATA_BINARY;
    //uartParams.readDataMode = UART_DATA_BINARY;
    //uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        // UART2_open() failed
        while (1);
    }
}

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    // Common I2C transaction setup
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;


    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if (found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

// function to read the temperature sensor
int16_t readTemp(void)
{
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        // Extract degrees c from the received data;
        // see TMP sensor datasheet
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        // If the MSB is set '1', then we have a 2's complement
        // negative value which needs to be sign extended
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d) \n\r", i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return abs(temperature);
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    tasks[0].state = BUTTON_FLAG_0; // Raise the flag for button 0
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* transitions for the SM */
    tasks[0].state = BUTTON_FLAG_1; // Raise the flag for button 1

}

// SM for the buttons task
int TickFct_CheckButtonState(int state)
{
    // SM transitions occur in the button callbacks

    // SM actions
    switch (state) {
        case BUTTON_FLAG_INIT:
            break;
        case BUTTON_FLAG_0:
            setPoint -= 1;            // decreases the temperature setPoint by 1
            state = BUTTON_FLAG_INIT; // lowers the button 0 flag
            break;
        case BUTTON_FLAG_1:
            setPoint += 1;            // increases the temperature setPoint by 1
            state = BUTTON_FLAG_INIT; // lowers the button 1 flag
            break;
        default:
            state = BUTTON_FLAG_INIT;
            break;
    }
    return state;
}

// SM for read temperature task
int TickFct_CheckTemperatureState(int state)
{
    // SM transitions
    switch (state) {
        case TEMPERATURE_INIT:
            state = TEMPERATURE_READ;
            break;
        case TEMPERATURE_READ:
            state = TEMPERATURE_READ;
            break;
        default:
            state = TEMPERATURE_INIT;
            break;
    }

    // SM actions
    switch (state) {
        case TEMPERATURE_INIT:
            state = TEMPERATURE_READ;
            break;
        case TEMPERATURE_READ:
            currentTemperature = readTemp();
            break;
        default:
            break;
    }
    return state;
}

// SM for LED task
int TickFct_CheckLEDState(int state)
{
    // SM transitions
    if (currentTemperature < setPoint) {
        state = LED_ON;
    }
    else {
        state = LED_OFF;
    }

    // SM actions
    switch(state) {
        case LED_INIT:
            break;
        case LED_ON:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            heat = 1;
            break;
        case LED_OFF:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            heat = 0;
            break;
        default:
            break;
    }
    return state;
}

// SM for server (UART) output task
int TickFct_CheckOutputState(int state)
{
    // SM transitions
    switch(state){
        case OUTPUT_INIT:
            state = OUTPUT_SEND;
            break;
        case OUTPUT_SEND:
            state = OUTPUT_SEND;
            break;
        default:
            state = OUTPUT_INIT;
    }

    // SM actions
    switch(state) {

        case OUTPUT_INIT:
            break;

        case OUTPUT_SEND:
            seconds++;
            DISPLAY(snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", currentTemperature, setPoint, heat, seconds))
            break;

        default:
            break;
    }
    return state;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART(); // The UART must be initialized before calling initI2C()
    initI2C();
    initTimer(); // initializes the timer

    // create each task
    unsigned char i = 0;

    // button task for when to check button states
    tasks[i].state = BUTTON_FLAG_INIT;
    tasks[i].taskPeriod = buttonPeriod;
    tasks[i].elapsedTime = tasks[i].taskPeriod;
    tasks[i].TickFct = &TickFct_CheckButtonState;
    ++i;

    // temperature task for when to read the temperature
    tasks[i].state = TEMPERATURE_INIT;
    tasks[i].taskPeriod = temperaturePeriod;
    tasks[i].elapsedTime = tasks[i].taskPeriod;
    tasks[i].TickFct = &TickFct_CheckTemperatureState;
    ++i;

    // task for setting the LED states
    tasks[i].state = LED_INIT;
    tasks[i].taskPeriod = ledPeriod;
    tasks[i].elapsedTime = tasks[i].taskPeriod;
    tasks[i].TickFct = &TickFct_CheckLEDState;
    ++i;

    // task for outputting data to server (UART)
    tasks[i].state = OUTPUT_INIT;
    tasks[i].taskPeriod = outputPeriod;
    tasks[i].elapsedTime = tasks[i].taskPeriod;
    tasks[i].TickFct = &TickFct_CheckOutputState;

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    //GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    while (1) {
        // task scheduler interates over tasks and calls the appropriate TickFnc if the elapsed time
        // is greater than or equal to each task's period.
        unsigned char i;
        for (i = 0; i < tasksNum; ++i) {
            if (tasks[i].elapsedTime >= tasks[i].taskPeriod) {
                tasks[i].state = tasks[i].TickFct(tasks[i].state);
                tasks[i].elapsedTime = 0;
            }
            tasks[i].elapsedTime += tasksPeriodGCD;
        }
        while(!TimerFlag) {}
        TimerFlag = 0;

    }

    return (NULL);
}
