/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 *  ======== pwmled2.c ========
 */
/* For usleep() */
#include <unistd.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/PWM.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*
 *  ======== mainThread ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
void *mainThread(void *arg0)
{
    // Period in microseconds
    uint16_t pwmPeriod = 3000;

    // Duty in microseconds
    // duty/period = duty cycle percentage
    uint16_t pwmDuty90 = 2700; // 2700/3000 = .9 or 90 percent
    uint16_t pwmDuty10 = 300;  //  300/3000 = .1 or 10 percent
    uint16_t pwmDuty0 = 0;     //    0/3000 = 0

    //uint16_t duty      = 0; no longer used

    //uint16_t dutyInc   = 100; no longer used

    /* Sleep time in microseconds */
    uint32_t time   = 1000000; // 1 second  = 1,000,000 microseconds
    PWM_Handle pwm1 = NULL;
    PWM_Handle pwm2 = NULL;
    PWM_Params params;

    /* Call driver init functions. */
    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits   = PWM_DUTY_US;
    params.dutyValue   = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1               = PWM_open(CONFIG_PWM_0, &params);
    if (pwm1 == NULL)
    {
        /* CONFIG_PWM_0 did not open */
        while (1) {}
    }

    PWM_start(pwm1);

    pwm2 = PWM_open(CONFIG_PWM_1, &params);
    if (pwm2 == NULL)
    {
        /* CONFIG_PWM_0 did not open */
        while (1) {}
    }

    PWM_start(pwm2);

    /* Loop forever blinking the yellow LED and alternating the duty of the green LED so that it goes from barely visible to bright */
    while (1)
    {
        PWM_setDuty(pwm1, pwmDuty90);  // sets the pwmDuty for the yellow LED to 90 percent - Bright
        PWM_setDuty(pwm2, pwmDuty10);  // sets the pwmDuty for the greed LED to 10 percent  - Dim

        usleep(time);                  // halts execution for one second

        PWM_setDuty(pwm1, pwmDuty0);   // sets the pwmDuty for the yellow LED to 0 percent  - Off
        PWM_setDuty(pwm2, pwmDuty90);  // sets the pwmDuty for the green LED to 90 percent  - Bright

        // this second usleep function is needed or the blink of the yellow LED or the change in the green LEDs duty will not be noticeable.
        usleep(time);                  // halts execution for one second


        // original code
        /*duty = (duty + dutyInc);

        if (duty == pwmPeriod || (!duty))
        {
            dutyInc = -dutyInc;
        }

        usleep(time);
        */
    }
}
