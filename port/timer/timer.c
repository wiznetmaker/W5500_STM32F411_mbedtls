/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <stdbool.h>

//#include "pico/stdlib.h"
#include "stm32f4xx_hal.h"

#include "timer.h"

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Timer */
//static struct repeating_timer g_timer;
void (*callback_ptr)(void);

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Timer */
void wizchip_1ms_timer_initialize(void (*callback)(void))
{
    callback_ptr = callback;
//    add_repeating_timer_us(-1000, wizchip_1ms_timer_callback, NULL, &g_timer);
		//bool add_repeating_timer_us(int32_t us, timer_callback_t callback, void *callback_arg, repeating_timer_t *timer);

}

//bool wizchip_1ms_timer_callback(struct repeating_timer *t)
//{
//    if (callback_ptr != NULL)
//    {
//        callback_ptr();
//    }
//}

/* Delay */
void wizchip_delay_ms(uint32_t ms)
{
		HAL_Delay(ms);
//    sleep_ms(ms);
}
