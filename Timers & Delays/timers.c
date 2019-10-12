/*
 * timers.c
 * 
 * Timer Control Library
 * for STM32F10x Microcontroller Family
 *
 * Used to control Timers and Counters
 * Delay functions with General Purpose Timers and SysTick 
 *
 * Version: 1.0 | Ravidu Liyanage
 */
 
#include "timers.h"

/********************** Global Variables **********************/
long int ticks = 0;
uint8_t SysTick_initFlag = 0;
uint8_t tim2_en = 0, tim3_en = 0, tim4_en = 0;

/************************* Functions **************************/
void init_SysTick(void)
{
	SysTick_initFlag = 1;
	
	SysTick_Config(SystemCoreClock / 1000);
}

void init_timer(TIM_TypeDef *timer)
{
	// Enable Timer 4 clock gating
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	
	// Configure Timer 4 with specific time base
	timer->PSC = 1; // Disable the prescalar
	timer->ARR = 72; // Divide the system clock by 72 to get 1 MHz
	timer->CR1 |= TIM_CR1_URS; // Only counter overflow/underflow generates an update interrupt
	timer->DIER |= TIM_DIER_UIE; // Update Interrupt Enable
	timer->EGR |= TIM_EGR_UG; // Update Generation
	
	if(timer == TIM2) 
	{
		NVIC_EnableIRQ(TIM2_IRQn);
		tim2_en = 1;
	}
	else if(timer == TIM3) 
	{
		NVIC_EnableIRQ(TIM3_IRQn);
		tim3_en = 1;
	}
	else if(timer == TIM4) 
	{
		NVIC_EnableIRQ(TIM4_IRQn);
		tim4_en = 1;
	}
}

void delay(int ms)
{
	if(SysTick_initFlag)
		init_SysTick();
	
	ticks = 0;
	
	while(ticks < ms);
}

void SysTick_Handler(void)
{
	ticks++;
}

void TIM2_IRQHandler(void)
{
	ticks++; // Increase the count variable at every overflow/underflow
	TIM2->SR &= ~TIM_SR_UIF; // Clear Interrupt Flag
}

void TIM3_IRQHandler(void)
{
	ticks++; // Increase the count variable at every overflow/underflow
	TIM3->SR &= ~TIM_SR_UIF; // Clear Interrupt Flag
}

void TIM4_IRQHandler(void)
{
	ticks++; // Increase the count variable at every overflow/underflow
	TIM4->SR &= ~TIM_SR_UIF; // Clear Interrupt Flag
}

void delay_us(TIM_TypeDef *timer, int us)
{
	if((timer == TIM2 && !tim2_en) || (timer == TIM3 && !tim3_en) || (timer == TIM4 && !tim4_en))
	{
		init_timer(timer);
	}
	
	timer->CR1 |= TIM_CR1_CEN; // Enable the counter
	
	ticks = 0; // Reset the count variable
	
	while(ticks < us); // Stuck in a while loop until the count becomes equal to us
	
	timer->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}

void delay_ms(TIM_TypeDef *timer, int ms)
{
	if((timer == TIM2 && !tim2_en) || (timer == TIM3 && !tim3_en) || (timer == TIM4 && !tim4_en))
	{
		init_timer(timer);
	}
	
	timer->CR1 |= TIM_CR1_CEN; // Enable the counter
	
	ticks = 0; // Reset the count variable
	
	while(ticks < ms*1000); // Stuck in a while loop until the count becomes equal to us
	
	timer->CR1 &= ~TIM_CR1_CEN; // Disable the counter
}

