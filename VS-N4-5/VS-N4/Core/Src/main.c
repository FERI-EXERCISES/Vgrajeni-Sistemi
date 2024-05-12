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
#define ARR_SIZE 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t count;
uint8_t IS_FIRST_IT = 1;
uint8_t arr[ARR_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TIM2_IRQHandler(void){ // interrupt service routine
	if(TIM2->SR & 1){ // Preveri update interrupt flag
		if(IS_FIRST_IT == 1){ // ko prvic kliknes na gumb
			if (count < ARR_SIZE){ // ce se ni poln (ni se cas potekel)
				arr[count] = (GPIOA->IDR & 1) ? 1 : 0; // ce gumb prizgan
			}else{ // ce je konec casa
				IS_FIRST_IT = 0;
				count = 0;
				GPIOD->ODR &= ~(1 << 12);// prizge pin 14
			}
			count++;
		}else{ // playback
			if (count < ARR_SIZE){ // ce se ni na koncu playbacka
				if(arr[count] == 1)
						GPIOD->ODR |= 1 << 14;
				else
						GPIOD->ODR &= ~(1 << 14);
			}
			else{
				TIM2->CR1 &= ~TIM_CR1_CEN;
				GPIOD->ODR &= ~(1 << 14);
			}
			count++;
		}
		TIM2->SR &= ~(1 << 0); // pocisti flag (dopusti nadaljevanje)
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	// Timer tick frequency = clockspeed/(PSC+1)
	// PSC = (clockspeed/zeljena frekvenca)-1
	// ARR = (timer tick frequency * zeljena casovna dolzina (v ms))

  	RCC->AHB1ENR |= 1 << 3; // EN GPIOD
    RCC->AHB1ENR |= 1 << 0; // EN GPIOA
    RCC->APB1ENR |= 1 << 0; // EN TIM2

    GPIOD->MODER |= 1 << 24; // GPIOD OUTPUT P12
    GPIOD->MODER |= 1 << 28; // GPIOD OUTPUT P14

    TIM2->ARR = 1000 - 1; // Auto-Reload Register -> maximum count
    TIM2->PSC = 16 - 1; //(Prescaler) upočasni timer za določen skalar (1ms)
    TIM2->DIER |= 1 << 0; // EN timer interrupt
    TIM2->CR1 |= TIM_CR1_URS; // Update Request Source -> overflow/underflow triggera interrupt

    NVIC_SetPriority(TIM2_IRQn, 1); //Nested Vectored Interrupt Controller
    NVIC_EnableIRQ(TIM2_IRQn); // TIM2_IRQn interrupt request


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // check if button press
	  if(GPIOA->IDR & 1){
		  GPIOD->ODR |= 1 << 12; // Set Pin D12
		  TIM2->CR1 |= TIM_CR1_CEN; // Clock EN
	  }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
