/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

typedef struct{

	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t LCKR;
	uint32_t AFRL;
	uint32_t AFRH;

}GPOI_typedef;

#define GPIOD ((GPOI_typedef*)(0x40020c00 + 0x00)) //GPIOD BASE
#define GPIOA ((GPOI_typedef*)(0x40020000 + 0x00)) //GPIOA BASE
#define RCC_AHB1ENR ((uint32_t *)(0x40023800 + 0x30)) //RCC CLOCK BASE

int main(void)
{
	uint32_t delay = 1000000;
	uint32_t lastButtonState = 0;

	*RCC_AHB1ENR |= 1 << 3; //GPIOD clock enable
	*RCC_AHB1ENR |= 1 << 0; //GPIOA clock enable

	GPIOD->MODER |= 0x55555000;
	GPIOA->MODER &= ~(0x3U << 0);

	for(int i = 3; i < 17; i++){
		//Flip delay
		uint32_t currentButtonState = (GPIOA->IDR & (1<<0));
		if(currentButtonState == 1 && lastButtonState == 0){
			if(delay == 1000000) delay /= 2;
			else delay *= 2;
			lastButtonState = currentButtonState;
		}else{
			lastButtonState = currentButtonState;
		}

		//Reset
		if(i >= 16){
			i = 2;
			continue;
		}

		//Turn on light
		if(5 <= i+3 && i+3 <= 15)
			GPIOD->ODR |= 1 << i+3;

		//Turn off light
		if(5 <= i && i <= 15)
			GPIOD->ODR &= ~(1 << i);

		//Delay
		for(int j = 0;j<delay;j++);
	}
}
