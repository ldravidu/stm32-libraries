/*
 * timers.h
 * 
 * Timer Control Library
 * for STM32F10x Microcontroller Family
 *
 * Used to control Timers and Counters
 * Delay functions with General Purpose Timers and SysTick 
 *
 * Version: 1.0 | Ravidu Liyanage
 */

#ifndef __TIMERS_H
#define __TIMERS_H

#include "stm32f10x.h"

/********************* Function Prototypes ********************/
void init_SysTick(void); // Initializes SysTick
void delay(int ms); // Delay Function, uses milli seconds

void init_timer(TIM_TypeDef *timer); // Initializes Timer
void delay_us(TIM_TypeDef *timer, int us); // Delay Function, uses micro seconds
void delay_ms(TIM_TypeDef *timer, int ms); // Delay Function, uses milli seconds

#endif /* __TIMERS_H */

