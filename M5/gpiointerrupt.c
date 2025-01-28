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

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>

// both Message_Init and Message_State are initialized to 0 which is MESSAGE_SOS
// SOS will be the first message since it is in the first index of Message_States
enum Message_States {MESSAGE_SOS, MESSAGE_OK} Message_Init, Message_State;

unsigned char count = 0;           // used as a counter to move through each message array
unsigned char MESSAGE_LED_ACTION;  // used for SM actions

const int OFF = 0;  // Used for turning LEDs off
const int DOT = 1;  // Used when blinking the red LED
const int DASH = 2; // Used when blinking the green LED

// array for the SOS Message
char SOS_MESSAGE_LED_POSITIONS[] = {
                          DOT, OFF, DOT, OFF, DOT,                                        // blinks the red led three times for the first S in SOS (2500ms total)
                          OFF, OFF, OFF,                                                  // keeps the LEDs off for an additional 1500ms
                          DASH, DASH, DASH, OFF, DASH, DASH, DASH, OFF, DASH, DASH, DASH, // three long green LED for the O in SOS 5500ms total
                          OFF, OFF, OFF,                                                  // keeps the LEDs off for an additional 1500ms
                          DOT, OFF, DOT, OFF, DOT,                                        // blinks the red led three times for the last S in SOS (2500ms total)
                          OFF, OFF, OFF, OFF, OFF, OFF, OFF                               // keeps the LEDs off for an additional 3500ms
};

// array for the OK message
char OK_MESSAGE_LED_POSITIONS[] = {
                         DASH, DASH, DASH, OFF, DASH, DASH, DASH, OFF, DASH, DASH, DASH,  // three long green LED for the O in OK 5500ms total
                         OFF, OFF, OFF,                                                   // keeps the LEDs off for an additional 1500ms
                         DASH, DASH, DASH, OFF, DOT, OFF, DASH, DASH, DASH,               // three long green LED, then both off, red blink, both off, 3 long green for K in OK 4500ms total
                         OFF, OFF, OFF, OFF, OFF, OFF, OFF                                // keeps the LEDs off for an additional 3500ms
};

/* Timer Callback */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    /* state machine actions */

    // For the SOS message
    if (Message_Init == MESSAGE_SOS) {

        MESSAGE_LED_ACTION = SOS_MESSAGE_LED_POSITIONS[count]; // Because the timer callback is going to be called every 500ms, we can use an array to move through
                                                               // that tells the SM what LED action should or should not happen every 500ms.
        switch(MESSAGE_LED_ACTION) {

            // turns both LEDs off
            case OFF:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                break;
            // turns the red LED on and turns the green led off
            case DOT:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                break;
            // turns the green LED on and the red LED off
            case DASH:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                break;
        }

        count++; // moves us to the next index in the message array

        // reset the count to go back to the beginning of the current message array
        if(count == 34) {
            Message_Init = Message_State; // Message_Init only changes states if the end of the OK message has finished.
            count = 0;
        }
    }

    // for the OK message
    if (Message_Init == MESSAGE_OK) {
        MESSAGE_LED_ACTION = OK_MESSAGE_LED_POSITIONS[count]; // Moving through the OK message array

        switch(MESSAGE_LED_ACTION) {
            // turns both LEDs off
            case OFF:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                break;
            // turns the red LED on and the green LED off
            case DOT:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                break;
            // turns the green LED on and the red off
            case DASH:
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                break;
        }

        count ++; // moves us to the next index in the message array

        // reset the count to go back to the beginning of the current message array
        if (count == 30) {
            Message_Init = Message_State; // Message_Init only changes states if the end of the OK message has finished.
            count = 0;
        }
    }
}

/* function to initialize the timer */
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params. timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialize timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // Not using button 0

    /* Toggle an LED */
    //GPIO_toggle(CONFIG_GPIO_LED_0);
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
    /* switch between SOS and OK messages */
    /* transitions for the SM */
    if (Message_Init == MESSAGE_OK) {
        Message_State = MESSAGE_SOS;
    }
    else {
        Message_State = MESSAGE_OK;
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initTimer(); // initializes the timer

    /* Configure the LEDs and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    //GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

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

    return (NULL);
}
