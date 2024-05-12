/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BRIGHTNESS 100 // Assuming Timer 4 ARR is configured for a max value of 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int tick = 0; // This will increment every time TIM3_IRQHandler is called
volatile int led1_brightness = 0;
volatile int led2_brightness = -25; // Offset by 1/4 period
volatile int led3_brightness = -50; // Offset by 1/2 period
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TIM3_IRQHandler(void) {
    if(TIM3->SR & TIM_SR_UIF) {
        // Update brightness for each LED
        led1_brightness = (tick < 50) ? tick * 2 : (100 - tick) * 2;
        led2_brightness = ((tick + 25) % 100 < 50) ? ((tick + 25) % 100) * 2 : (100 - ((tick + 25) % 100)) * 2;
        led3_brightness = ((tick + 50) % 100 < 50) ? ((tick + 50) % 100) * 2 : (100 - ((tick + 50) % 100)) * 2;

        // Update Timer 4 Compare Registers
        TIM4->CCR1 = led1_brightness;
        TIM4->CCR2 = led2_brightness;
        TIM4->CCR3 = led3_brightness;

        tick = (tick + 1) % 100; // Increment
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();

  /* USER CODE BEGIN Init */
	// EN Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// GPIOD Pins Alternate Function Mode	GPIOD->MODER |= 1 << 25;
	GPIOD->MODER |= 1 << 27;
	GPIOD->MODER |= 1 << 29;

	// Alternate Function to AF2 (output PWM)
	GPIOD->AFR[1] |= 2 << 16;
	GPIOD->AFR[1] |= 2 << 20;
	GPIOD->AFR[1] |= 2 << 24;

	// cikel nastavitve
	TIM4->ARR = MAX_BRIGHTNESS - 1;
	TIM4->PSC = 1600 - 1;
	TIM3->ARR = MAX_BRIGHTNESS - 1;
	TIM3->PSC = 1600 - 1;

	//In PWM mode 1, the output pin is set on a timer reset
	// (when the counter is zero) and reset when the
	// counter value matches the CCRx value.

	//In PWM mode 2, the output pin is reset on a timer reset
	// and set when the counter matches the CCRx value

	// Nastavi na PWM mode 2
	TIM4->CCMR1 |= 6 << 4; //PD12
	TIM4->CCMR1 |= 6 << 12; //PD13
	TIM4->CCMR2 |= 6 << 4; //PD14

	// Omogoči output kanale timer4
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CCER |= TIM_CCER_CC2E;
	TIM4->CCER |= TIM_CCER_CC3E;

	// Nastavimo update interrupt in generate interrupts on overflow
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_URS;
	TIM4->DIER |= TIM_DIER_UIE;
	TIM4->CR1 |= TIM_CR1_URS;

	// nastavi interrupt rutino
	NVIC_SetPriority(TIM4_IRQn, 2);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 1);
	NVIC_EnableIRQ(TIM3_IRQn);

	// Omogoči ure
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
