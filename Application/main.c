/*
 * main.c
 *
 *  Created on: 12 дек. 2018 г.
 *      Author: admin
 */

#include "stm32f4xx.h"
#include "main.h"
#include "pid.h"

#define K_P     0.00001
#define K_I     0.0001
#define K_D     0.00000

struct PID_DATA pidData;

int32_t delayRef = -230;

volatile int32_t delayCCR2;
volatile int32_t delayCCR1;

static volatile uint16_t hTimebase_500us = 0;

volatile struct GLOBAL_FLAGS {
  //! True when PID control loop should run one time
  uint8_t pidTimer:1;
  uint8_t startTimer:1;
  uint8_t clearCC1IF:1;
  uint8_t clearCC2IF:1;
  uint8_t dummy:7;
} gFlags = {0, 0, 0, 0, 0};


// Prototypes

void TB_Wait(uint32_t time);
void setFreq(TIM_TypeDef* tim, uint16_t val);
void setPulseWidth(TIM_TypeDef* tim, uint16_t val);


// Functions

void TB_Wait(uint32_t time)
{
	hTimebase_500us = time;    // delay = 'time' value * 5ms
	while (hTimebase_500us != 0){} // wait and do nothing!
}

void setFreq(TIM_TypeDef* tim, uint16_t val)
{
    tim->ARR = val;
}

void setPulseWidth(TIM_TypeDef* tim, uint16_t val)
{
    tim->CCR1 = val;
}


int main(void)
{
	clockConfig();
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	TIM2_Init();
	TIM1_Init();

	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER6_0;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= GPIO_MODER_MODER1_0 | GPIO_MODER_MODER0_0;
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED0;



	while(1)
	{

	}
	return 0;
}


void TIM1_UP_TIM10_IRQHandler(void)
{

	TIM2->CNT = 0;

	TIM2->SR &=~ TIM_SR_CC2IF;
	TIM2->SR &=~ TIM_SR_CC1IF;

	while( ((TIM2->SR & TIM_SR_CC2IF) && (TIM2->SR & TIM_SR_CC1IF)) == 0 ){;}

	delayCCR2 = (TIM2->CCR2);
	delayCCR1 = (TIM2->CCR1);

	//int32_t delay = ((delayCCR2) - (delayCCR1)); // CCR1 -> PWM
	//int32_t delay = ((delayCCR1) - (delayCCR2)); // CCR1 -> PWM ???

	//int32_t delay = ((delayCCR2) - (delayCCR1)) * 11.9; // CCR1 -> PWM
	int32_t delay = ((delayCCR1) - (delayCCR2)) * 11.9; // CCR1 -> PWM ???
	//int32_t delay = (int32_t)((delayCCR2) - (delayCCR1)) * 11.9; // CCR1 -> PWM

	//PID regulator with feed forward from speed input.
	int32_t outputValue;
	outputValue = delayRef;
	outputValue += pid_Controller(delayRef, delay, &pidData);

		//int32_t error = delayRef - delay;
		//outputValue += error;

	if (outputValue < 40)
	{
		outputValue = 40;
	}
	else if (outputValue > 20000)
	{
		outputValue = 20000;
	}

	setFreq(TIM1, outputValue);
	setPulseWidth(TIM1, outputValue / 2);

	TIM1->SR &=~ TIM_SR_UIF;

}


void TIM1_CC_IRQHandler(void)
{
	TIM1->SR &=~ TIM_SR_CC1IF;
	//WRITE_REG(GPIOB->ODR, READ_REG(GPIOB->ODR) ^ GPIO_ODR_OD0);
}

void TIM2_IRQHandler(void)
{


	//TIM2->SR &=~ TIM_SR_CC2IF;
	//TIM2->SR &=~ TIM_SR_CC2IF;
	//TIM2->SR &=~ TIM_SR_UIF;


	//WRITE_REG(GPIOB->ODR, READ_REG(GPIOB->ODR) ^ GPIO_ODR_OD0);
	//WRITE_REG(GPIOB->ODR, READ_REG(GPIOB->ODR) ^ GPIO_ODR_OD1);


}


void SysTick_Handler(void)
{

	if (hTimebase_500us != 0)
	{
		hTimebase_500us --;
	}
}
