/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
 *  ======== uart2echo.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/* State Machine */

enum LED_states {INIT, FIRST_KEY_PRESS, SECOND_KEY_PRESS, LED_ON, LED_OFF} LED_State;

void TickFct_LED(char input) {

    // transitions
    switch(LED_State) {

        // initial transition
        case INIT:
            if (input=='O') {
                LED_State = FIRST_KEY_PRESS;
            }
            else {
                LED_State = INIT;
            }
            break;

        // second transition
        case FIRST_KEY_PRESS:
            if (input == 'N') {
                LED_State = LED_ON;
            }
            else if (input == 'F') {
                LED_State = SECOND_KEY_PRESS;
            }
            else {
                LED_State = INIT;
            }
            break;

        // third transition
        case SECOND_KEY_PRESS:
            if (input == 'F') {
                LED_State = LED_OFF;
            }
            else {
                LED_State = INIT;
            }
            break;

        //default transition - transitions back to initial state if any other key is pressed
        default:
            LED_State = INIT;
            break;
    }

    //state actions
   switch(LED_State) {

       // turns the LED on if the LED_State is ON and resets the LED_State to INIT
       case LED_ON:
           GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
           LED_State = INIT;
           break;

       // turns the LED off if the LED_State is off and resets the LED_State to INIT
       case LED_OFF:
           GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
           LED_State = INIT;
           break;

       // breaks for any other LED_State
       default:
           break;
   }
}

/*
 *  ======== mainThread ========
 */

void *mainThread(void *arg0)
{
    char input;
    char const echoPrompt[] = "Echoing characters:\r\n";
    UART2_Handle uart;
    UART2_Params uartParams;
    size_t bytesRead;
    size_t bytesWritten = 0;

    // Call driver init functions
    GPIO_init();

    // Configure the LED pin
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // Create a UART with data processing off.
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        // UART2_open() failed
        while (1);
    }

    UART2_write(uart, echoPrompt, sizeof(echoPrompt), &bytesWritten);

    LED_State = INIT; // sets the initial LED_State

    // Loop forever echoing
    while (1) {

        UART2_read(uart, &input, 1, &bytesRead); // reads input from the user

        input = toupper(input); // changes user input to upper case characters

        TickFct_LED(input); // calls the SM Tick Function and passes the users input

        UART2_write(uart, &input, 1, &bytesWritten); // writes input from the user
    }

}


