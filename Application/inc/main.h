/*
 * main.h
 *
 *  Created on: 12 дек. 2018 г.
 *      Author: admin
 */

#ifndef APPLICATION_INC_MAIN_H_
#define APPLICATION_INC_MAIN_H_

#include "stm32f4xx.h"

/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      4
#define PLL_N      168

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      4

void clockConfig(void);

void clockConfig(void)
{
	RCC->CR |= (uint32_t)RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY)){};

    /* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK / 1*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* PCLK1 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

    /* Configure the main PLL */
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0){}

    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL){};
}


void TIM2_Init(void);

void TIM2_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER15_1 | GPIO_MODER_MODER1_1;
	//GPIOA->PUPDR |= GPIO_PUPDR_PUPD15_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL1_0;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL15_0;


	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM2->CCMR1 |= TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0; // Input Capture 1 Filter
	//TIM2->CCMR1 |= TIM_CCMR1_IC1PSC_0; // Input Capture 1 Prescaler
	TIM2->CCER |= TIM_CCER_CC1E; // Capture enabled
	TIM2->CCER &=~ TIM_CCER_CC1P; // rising edge CC1P = 0; CC1NP = 0
	TIM2->CCER &=~ TIM_CCER_CC1NP;

	TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
	TIM2->CCMR1 |= TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_0; // Input Capture 2 Filter
	//TIM2->CCMR1 |= TIM_CCMR1_IC2PSC_0; // Input Capture 2 Prescaler
	TIM2->CCER |= TIM_CCER_CC2E; // Capture enabled
	TIM2->CCER &=~ TIM_CCER_CC2P; // rising edge CC2P = 0; CC2NP = 0
	TIM2->CCER &=~ TIM_CCER_CC2NP;

	//TIM2->CCMR1 &=~ TIM_CCMR1_IC1PSC;

	TIM2->PSC = 0; // fCK_PSC / (PSC[15:0] + 1)
	TIM2->ARR = 0xFFFFFFFF;
	//TIM2->ARR = 0xFFFF;
	TIM2->CNT = 0;

	//TIM2->DIER |= TIM_DIER_CC1IE; // Capture/Compare 1 interrupt enable
	//TIM2->DIER |= TIM_DIER_CC2IE; // Capture/Compare 2 interrupt enable
	//TIM2->DIER |= TIM_DIER_UIE; // Update interrupt enable

	//TIM2->EGR |= TIM_EGR_CC1G; // Capture/Compare 1 Generation
	//TIM2->EGR |= TIM_EGR_CC2G;


	TIM2->CR1 |= TIM_CR1_CEN;

	//NVIC_EnableIRQ(TIM2_IRQn);

}


void TIM1_Init(void);

void TIM1_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// GPIOA
	GPIOA->MODER |= GPIO_MODER_MODE8_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED8;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL8_0;


	// TIM1

	TIM1->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	//TIM1->CR1 |= TIM_CR1_CMS; // Center-aligned mode selection
	//TIM1->CR1 |= TIM_CR1_DIR; // Direction

	// Mode reg
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // Output Compare 1 Mode
	TIM1->CCMR1 &=~ TIM_CCMR1_CC1S; // Capture/Compare 1 Selection
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Output Compare 1 Preload enable

	// Enable reg
	//TIM1->CCER &=~ TIM_CCER_CC1P; //Capture/Compare 1 output Polarity
	TIM1->CCER |= TIM_CCER_CC1P;
	TIM1->CCER |= TIM_CCER_CC1E; //Capture/Compare 1 output enable


	TIM1->PSC = 10;
	TIM1->ARR = 40; //100
	TIM1->CCR1 = 20;

	//TIM1->BDTR |= TIM_BDTR_DTG_7 | TIM_BDTR_DTG_3; //DTG[0:7] bits (Dead-Time Generator set-up)
	TIM1->BDTR |= TIM_BDTR_MOE; //Main Output enable
	//TIM1->EGR |= TIM_EGR_UG;
	TIM1->RCR = 1;

	TIM1->DIER |= TIM_DIER_UIE;
	//TIM1->DIER |= TIM_DIER_CC1IE;

	//TIM1->CR1 |= TIM_CR1_CKD_0;
	TIM1->CR1 |= TIM_CR1_CEN;

	//NVIC_EnableIRQ(TIM1_CC_IRQn);
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

#endif /* APPLICATION_INC_MAIN_H_ */
