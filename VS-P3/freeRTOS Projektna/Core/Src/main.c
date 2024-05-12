/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "esp32_manager.h"
#include "MQTT_dispatcher.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USED_UART USART2
//#define WIFI_SSID A1-8CB990
//#define WIFI_PASS NLMJGNENXN
//#define MQTT_BROKER_PORT 1883


#define TaskReadAccel_BIT (1UL << 0UL)
#define TaskSendAccel_BIT (1UL << 1UL)
#define TaskReadGyro_BIT (1UL << 2UL)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
EXTERN_TASK_HANDLES
QueueHandle_t QueueGyro;
QueueHandle_t QueueAccel;
EventGroupHandle_t xEventGroup;
uint8_t cip2 = 11;
int16_t meritev_test[6];
int ret;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void TaskReadGyro(void *pvParameters);
void TaskReadAccel(void *pvParameters);
void TaskSendGyro(void *pvParameters);
void TaskSendAccel(void *pvParameters);

void initGyro(void);
void initAccel(void);
uint8_t spi1_beriRegister(uint8_t);
void spi1_beriRegistre(uint8_t, uint8_t*, uint8_t);
void spi1_pisiRegister(uint8_t, uint8_t);
uint8_t i2c1_pisiRegister(uint8_t, uint8_t, uint8_t);
void i2c1_beriRegistre(uint8_t, uint8_t, uint8_t*, uint8_t);
void initAccel(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spi1_beriRegister(uint8_t reg) {
    uint16_t buf_out, buf_in;
    reg |= 0x80; // najpomembnejsi bit na 1
    buf_out = reg; // little endian, se postavi na pravo mesto ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&buf_out, (uint8_t*)&buf_in, 2, 2); // blocking posiljanje ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    return buf_in >> 8; // little endian...
}
void spi1_pisiRegister(uint8_t reg, uint8_t vrednost) {
    uint16_t buf_out;
    buf_out = reg | (vrednost<<8); // little endian, se postavi na pravo mesto ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&buf_out, 2, 2); // blocking posiljanje ....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
void spi1_beriRegistre(uint8_t reg, uint8_t* buffer, uint8_t velikost) {
    reg |= 0xC0; // najpomembnejsa bita na 1
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reg, 1, 10); // blocking posiljanje....
    HAL_SPI_Receive(&hspi1,  buffer, velikost, velikost); // blocking posiljanje....
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
void initGyro() { // ne pozabit klicat te funkcije
    uint8_t cip = spi1_beriRegister(0x0F);
    if (cip!=0xD4 && cip!=0xD3) {
        for (;;);
    }
    spi1_pisiRegister(0x20, 0x4F); // CTRL_REG1 PD Zen Yen Xen 200hz
    //TUKAJ NASTAVI Hz pa sensitivity +-500dps
    spi1_pisiRegister(0x22, 0x08); //DRDY
    spi1_pisiRegister(0x23, 0x10);
}
uint8_t i2c1_pisiRegister(uint8_t naprava, uint8_t reg, uint8_t podatek) {
    naprava <<= 1;
    return HAL_I2C_Mem_Write(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, &podatek, 1, 10);
}
void i2c1_beriRegistre(uint8_t naprava, uint8_t reg, uint8_t* podatek, uint8_t dolzina) {
    if ((dolzina>1)&&(naprava==0x19))  // ce je naprava 0x19 moramo postaviti ta bit, ce zelimo brati vec zlogov
        reg |= 0x80;
    naprava <<= 1;
    HAL_I2C_Mem_Read(&hi2c1, naprava, reg, I2C_MEMADD_SIZE_8BIT, podatek, dolzina, dolzina);
}
void initAccel() {
    i2c1_pisiRegister(0x19, 0x20, 0x27);
    i2c1_pisiRegister(0x19, 0x23, 0xA8);
}
void TaskReadGyro(void *pvParameters){
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;

	const EventBits_t xBitsToWaitFor = (TaskReadAccel_BIT | TaskSendAccel_BIT);
	while(1){
		xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);


		spi1_beriRegistre(0x28, (uint8_t*)&gyroX, 2);
		spi1_beriRegistre(0x2A, (uint8_t*)&gyroY, 2);
		spi1_beriRegistre(0x2C, (uint8_t*)&gyroZ, 2);

		xQueueSendToBack(QueueGyro, &gyroX, 0);
		xQueueSendToBack(QueueGyro, &gyroY, 0);
		xQueueSendToBack(QueueGyro, &gyroZ, 0);
	}
}
void TaskReadAccel(void *pvParameters){
	float accelX;
	float accelY;
	float accelZ;
	int16_t meritve[3];

	while(1){
		//i2c1_beriRegistre(0x19, 0x28,(uint8_t*)&meritve[0], 2);
		//i2c1_beriRegistre(0x19, 0x2A,(uint8_t*)&meritve[1], 2);
		//i2c1_beriRegistre(0x19, 0x2C,(uint8_t*)&meritve[2], 2);
		i2c1_beriRegistre(0x19, 0x28,(uint8_t*)&meritve, 6);

		meritve[0] = meritve[0] >> 4;
		meritve[1] = meritve[1] >> 4;
		meritve[2] = meritve[2] >> 4;

		accelX = meritve[0] * 0.0039;
		accelY = meritve[1] * 0.0039;
		accelZ = meritve[2] * 0.0039;

		xQueueSendToBack(QueueAccel, &accelX, 0);
		xQueueSendToBack(QueueAccel, &accelY, 0);
		xQueueSendToBack(QueueAccel, &accelZ, 0);

		xEventGroupSetBits(xEventGroup, TaskReadAccel_BIT);
	}

}
void TaskSendGyro(void *pvParameters){
	int16_t meritev[10];
	float meritev_send[5];

	meritev_send[0] = 0xaaaa;

	meritev[0] = 0xaaaa;
	const EventBits_t xBitsToWaitFor = (TaskReadAccel_BIT | TaskSendAccel_BIT | TaskReadGyro_BIT);
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	while(1){
		xEventGroupWaitBits(xEventGroup, xBitsToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);


		xQueueReceive(QueueGyro, &meritev[1], xTicksToWait);
		xQueueReceive(QueueGyro, &meritev[2], xTicksToWait);
		xQueueReceive(QueueGyro, &meritev[3], xTicksToWait);

//		meritev_send[1] =meritev[1] * 0.0175;
//		meritev_send[2] =meritev[2] * 0.0175;
//		meritev_send[3] =meritev[3] * 0.0175;

		meritev[4] = DWT->CYCCNT;
		//float clock = DWT->CYCCNT;

		//posli
		CDC_Transmit_FS((uint8_t*)&meritev, sizeof(meritev));
		//CDC_Transmit_FS((uint8_t*)&clock, sizeof(clock));
	}
}
void TaskSendAccel(void *pvParameters){
	float meritev[5];

	meritev[0] = 0xbbbb;
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	while(1){
		xQueueReceive(QueueAccel, &meritev[1], xTicksToWait);
		xQueueReceive(QueueAccel, &meritev[2], xTicksToWait);
		xQueueReceive(QueueAccel, &meritev[3], xTicksToWait);

		meritev[4] = DWT->CYCCNT;
		//posli
		//CDC_Transmit_FS((uint8_t*)&glava, sizeof(glava));
		CDC_Transmit_FS((uint8_t*)&meritev, sizeof(meritev));

		xEventGroupSetBits(xEventGroup, TaskSendAccel_BIT);

	}
}

void task_test(void *pvParameteres){
	int16_t meritev[8];
	meritev[0] = 0xaaaa;

	float meritev2[4];
	meritev2[0] = 0xbbbb;

	float accelX;
	float accelY;
	float accelZ;

	int16_t meritve[3];
	while(1){
		i2c1_beriRegistre(0x19, 0x28,(uint8_t*)&meritve[0], 2);
		i2c1_beriRegistre(0x19, 0x2A,(uint8_t*)&meritve[1], 2);
		i2c1_beriRegistre(0x19, 0x2C,(uint8_t*)&meritve[2], 2);

		meritve[0] = meritve[0] >> 4;
		meritve[1] = meritve[1] >> 4;
		meritve[2] = meritve[2] >> 4;

		accelX = meritve[0] * 0.0039;
		accelY = meritve[1] * 0.0039;
		accelZ = meritve[2] * 0.0039;

		meritev2[1] = accelX;
		meritev2[2] = accelY;
		meritev2[3] = accelZ;

		mqtt_pub_raw("AccelData", &meritev2, sizeof(meritev2), 1, 0, pdMS_TO_TICKS(100));

		int16_t gyroX;
		int16_t gyroY;
		int16_t gyroZ;

		spi1_beriRegistre(0x28, (uint8_t*)&gyroX, 2);
		spi1_beriRegistre(0x2A, (uint8_t*)&gyroY, 2);
		spi1_beriRegistre(0x2C, (uint8_t*)&gyroZ, 2);

//		gyroX *= 0.0175;
//		gyroY *= 0.0175;
//		gyroZ *= 0.0175;

		meritev[1] = gyroX;
		meritev[2] = gyroY;
		meritev[3] = gyroZ;

		mqtt_pub_raw("GyroData", &meritev, sizeof(meritev), 1, 0, pdMS_TO_TICKS(100));

		//vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  __HAL_I2C_ENABLE(&hi2c1);
  __HAL_SPI_ENABLE(&hspi1);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  initGyro();
  initAccel();


//  if(esp32_init(&huart2) != pdPASS){
//	  for(;;);
//  }
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//  if(mqtt_init() != pdPASS){
//	  for(;;);
//  }
//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  QueueGyro = xQueueCreate( 5, sizeof( int16_t ) );
  QueueAccel = xQueueCreate( 5, sizeof( float ) );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(TaskSendGyro, "CDC_Gyro", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  xTaskCreate(TaskSendAccel, "CDC_Accel", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  xTaskCreate(TaskReadGyro, "Data_Gyro", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(TaskReadAccel, "Data_Accel", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  //xTaskCreate(task_test, "running branje in pisanje accel", 500, NULL, 1, NULL);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  xEventGroup = xEventGroupCreate();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
