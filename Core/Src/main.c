/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = {
  .name = "Motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for showTask */
osThreadId_t showTaskHandle;
const osThreadAttr_t showTask_attributes = {
  .name = "showTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoder */
osThreadId_t encoderHandle;
const osThreadAttr_t encoder_attributes = {
  .name = "encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IR_sensor */
osThreadId_t IR_sensorHandle;
const osThreadAttr_t IR_sensor_attributes = {
  .name = "IR_sensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void motor(void *argument);
void showtask(void *argument);
void encoder_1(void *argument);
void ir_sensor(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[1];
int pwm = 0;
double max_rpm = 0;
double distance = 0; // later need to change to double
uint8_t completed = 1;
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
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  // initialise the pins needed

  HAL_UART_Receive_IT(&huart3,(uint8_t *)aRxBuffer,1);

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Motor */
  MotorHandle = osThreadNew(motor, NULL, &Motor_attributes);

  /* creation of showTask */
  showTaskHandle = osThreadNew(showtask, NULL, &showTask_attributes);

  /* creation of encoder */
  encoderHandle = osThreadNew(encoder_1, NULL, &encoder_attributes);

  /* creation of IR_sensor */
  IR_sensorHandle = osThreadNew(ir_sensor, NULL, &IR_sensor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 320;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 6000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED1_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	osDelay(100);
	completed = 0;
	distance = 0;
	UNUSED(huart);
    HAL_UART_Transmit(&huart3, aRxBuffer, 1, 100);
	HAL_UART_Receive_IT(&huart3, aRxBuffer, 1);


}

void Calculate_PWM(uint8_t right,uint8_t left)
{
	// use the following formula to calculate the relevnat pwm needed for different distance and change the 8399 to the preloadregister of the counter
//    25% duty cycle:     pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
//    50% duty cycle:     pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
//    75% duty cycle:     pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
//    100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
//	if(aRxBuffer[0]=='s')
//	{
//		// replace this with center
//		if( left>300)
//		{
//			pwm = 2800;
//		}
//		else if (  left>200)
//		{
//			pwm = 2500;
//		}
//		else if ( left>100)
//		{
//			pwm = 2000;
//		}
//		else
//		{
//			pwm = 0;
//		}
//	}
//	 if(aRxBuffer[0]=='r')
//	{
//		if(right>300)
//		{
//			pwm = 1800;
//		}
//		else if ( right>200)
//		{
//			pwm = 1500;
//		}
//		else if(right>150)
//		{
//			pwm = 1000;
//		}
//		else
//		{
//			pwm = 0;
//		}
//	}
//	else if(aRxBuffer[0]=="l")
//	{
//		if(left>300)
//		{
//			pwm = 1800;
//		}
//		else if(left>200)
//		{
//			pwm= 1500;
//		}
//		else if(left>150)
//		{
//			pwm = 1000;
//		}
//		else
//		{
//			pwm = 0;
//		}
//	}

	if(completed)
	{
		pwm = 0;
	}
	else
	{
		pwm = 2500;
	}


}

void CalculateDistance(double rpm)
{
	double local_distance ;

		local_distance = (rpm*2*M_PI*20) ; // minus 50 for longer distance ( more than 300 mm)
		if(!completed)
		{
			if(distance>=100)
			{
				completed =1;
			}
			else
			{
				distance = distance+local_distance;
			}

		}
}

void CalculateCurveDistance( double rpm)
{
	double local_distance ;

		local_distance = (rpm*2*M_PI*20) ; // minus 50 for longer distance ( more than 300 mm)
		if(!completed)
		{
			if(distance>=550) // u need 1100 for a 180 degree turn
			{
				completed =1;
			}
			else
			{
				distance = distance+local_distance;
			}

		}
}



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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for(;;)
  {

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	    osDelay(2000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  // change the pwm signal based on the distance

  /* Infinite loop */
  // for max turning use 50 and 110 and for center use 74 instead
  // use 68 and 78 for slight


  for(;;)
  {
	// backward direction

	  // come up with the states
		  switch(aRxBuffer[0])
		  {
		  case 's':
		      htim1.Instance->CCR4 = 74;
		      HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm);

		      HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
		      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm);
		      osDelay(10);
		      CalculateDistance(max_rpm);
		      osDelay(10);
		      break;
		  case 'r':
		      htim1.Instance->CCR4 = 110;
		      HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm);

		      HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
		      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm);
		      osDelay(500);
		      CalculateDistance(max_rpm);
		      osDelay(10);
		      break;
		  case 'l':
		      htim1.Instance->CCR4 = 50;
		      HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm);

		      HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
		      __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm);
		      osDelay(10);
		      CalculateCurveDistance(max_rpm);
		      osDelay(10);
		      break;
		  case 'b':
			  htim1.Instance->CCR4 = 74;
			  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm);

			  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
			  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwm);
			  osDelay(500);
		      CalculateDistance(max_rpm);
		      osDelay(10);
			  break;

		  default:
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
		      break;
		  }

  }





  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_showtask */
/**
* @brief Function implementing the showTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_showtask */
void showtask(void *argument)
{
  /* USER CODE BEGIN showtask */
	uint8_t hello[20] = "Hello World!\0";\
	uint8_t hello1[20];
	int local;
  /* Infinite loop */
  for(;;)
  {
//	  sprintf(hello,"Distance%d\0",distance);
//	  OLED_ShowNumber(10,10,distance,3,16);
//	  OLED_ShowString(10,10,hello);
	  local = (max_rpm*2*M_PI*33);
	  sprintf(hello1,"local:%d\0",  local);
	  OLED_ShowString(10, 20, hello1);
	  OLED_Refresh_Gram();
	  //osDelay(1000);
  }
  /* USER CODE END showtask */
}

/* USER CODE BEGIN Header_encoder_1 */
/**
* @brief Function implementing the encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_1 */
void encoder_1(void *argument)
{
  /* USER CODE BEGIN encoder_1 */
  /* Infinite loop */
		HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
		int cnt1, cnt2,cnt3,cnt4, diff1,diff2;
		uint32_t tick;
		cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		cnt3 = __HAL_TIM_GET_COUNTER(&htim3);
		tick = HAL_GetTick();
		uint8_t hello1[20];
		uint8_t hello2[20];
		double rpm_1 = 0; // the rpm is in per second
		double rpm_2 = 0;
	  for(;;)
	  {
		  if(HAL_GetTick()-tick > 200L){
			  // so what we are doing here is that for every 1s the counter value is 411
			  // when the duty cycle of the motor is 2000
			  cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
			  cnt4 = __HAL_TIM_GET_COUNTER(&htim3);
			  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
				  if(cnt2<cnt1)
				  {
					  diff1 = cnt1 - cnt2;
					  rpm_1 = ((diff1*5.0f)/(2.0f*330.0f*60.0f)) ;
				  }
				  else
				  {
					  diff1 = (65535 - cnt2)+cnt1;
					  rpm_1 = ((diff1*5.0f)/(2.0f*330.0f*60.0f)) ;
				  }


			  }
			  else {
			  				  if(cnt2>cnt1)
			  				  {
			  					  diff1 = cnt2 - cnt1;
			  					rpm_1 = ((diff1*5.0f)/(2.0f*330.0f*60.0f)) ;
			  				  }

			  				  else
			  				  {
			  					  diff1 = (65535 - cnt1) + cnt2;
			  					rpm_1 = ((diff1*5.0f)/(2.0f*330.0f*60.0f)) ;
			  				  }
			  			  }


			  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
			  {
				  if(cnt4<cnt3)
				  {
					  diff2 = cnt3 - cnt4;
					  rpm_2 = ((diff2*5.0f)/(2.0f*330.0f*60.0f)) ;
				  }
				  else
				  {
					  diff2 = (65535 - cnt4)+cnt3;
					  rpm_2 = ((diff2*5.0f)/(2.0f*330.0f*60.0f)) ;
				  }
			  }

			  else
			  {
				  if(cnt4>cnt3)
				  {
					  diff2 = cnt4 - cnt3;
					  rpm_2 = ((diff2*5.0f)/(2.0f*330.0f*60.0f)) ;
				  }

				  else
				  {
					  diff2 = (65535 - cnt2) + cnt4;
					  rpm_2 = ((diff2*5.0f)/(2.0f*330.0f*60.0f)) ;


				  }
			  }

				  if(rpm_1>rpm_2)
				  {
					  max_rpm = rpm_1;
				  }
				  else
				  {
					  max_rpm = rpm_2;
				  }
				  if(max_rpm>6)
				  {
					  max_rpm = 0;
				  }


			  sprintf(hello1,"encoder:%d\0",  diff1);
			  OLED_ShowString(10, 30, hello1);

//			  average_rpm_1 = average_rpm_1 + rpm_1;
//			  average_rpm_2 = average_rpm_2 + rpm_2;
//			  if(count==2)
//			  {
				  // the difference in tick is the period -> get the number of ticks passed ( in millisecond)
				  // then we have the tick frequency
//				  average_rpm_1 = average_rpm_1/ count ;
//				  average_rpm_2 = average_rpm_2/ count ;
//				  if(average_rpm_1<6.0f && average_rpm_2< 6.0f)
//				  {
//					  if(average_rpm_1>average_rpm_2)
//					  {
//						  max_rpm = average_rpm_1;
//					  }
//					  else
//					  {
//						  max_rpm = average_rpm_2;
//					  }
//				  }
//				  else
//				  {
//					  max_rpm = 0;
//				  }
//
//				  local = (max_rpm*2*M_PI*33);
//				  sprintf(hello1,"local:%d\0",  local);
//				  OLED_ShowString(10, 20, hello1);
////				  sprintf(hello2,"Speed2:%5d\0", max_rpm);
////				  OLED_ShowString(10, 30, hello2);
//				  count = 0;
//				  average_rpm_1 = 0;
//				  average_rpm_2 = 0;
//			  }


			 // dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
			  //sprintf(hello,"Dir1:%5d\0", dir);
			 // OLED_ShowString(10, 30, hello);

			  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
			  cnt3 = __HAL_TIM_GET_COUNTER(&htim3);
			  tick = HAL_GetTick();
		  }
	    //osDelay(1);
	  }
  /* USER CODE END encoder_1 */
}

/* USER CODE BEGIN Header_ir_sensor */
/**
* @brief Function implementing the IR_sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ir_sensor */
void ir_sensor(void *argument)
{
  /* USER CODE BEGIN ir_sensor */
  /* Infinite loop */

	uint32_t right_adc;
	uint32_t left_adc;
	double right_sensor;
	double left_sensor;
	uint32_t right_sensor_int;
	uint32_t left_sensor_int;
	uint32_t LPF_SUM_right = 0;
	uint32_t LPF_SUM_left = 0;
	uint32_t counter = 0;
	char buffer[100];
// hadc2 is the left sensor and hadc 1 is the right sensor
  for(;;)
  {
		HAL_ADC_Start(&hadc2);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
		left_adc = HAL_ADC_GetValue(&hadc2);
		right_adc = HAL_ADC_GetValue(&hadc1);

		LPF_SUM_right = LPF_SUM_right+right_adc;
		LPF_SUM_left = LPF_SUM_left+left_adc;
		counter++;
		if(counter>=100)
		{
			right_sensor = LPF_SUM_right/counter;
			left_sensor = LPF_SUM_left/counter;
			// come up with the equation to convert the sensor reading to distance
			right_sensor_int = (0.0000313111 *pow(right_sensor,2))-(0.230545* right_sensor)+493.974;
			left_sensor_int = (0.0000313111 *pow(left_sensor,2))-(0.230545* left_sensor)+493.974;
			Calculate_PWM(right_sensor_int,left_sensor_int);
			//sprintf(buffer, "right%dmm, left%dmm, completed=%d\r\n", right_sensor_int, left_sensor_int,completed);
			sprintf(buffer, "pwm%dmm, left%dmm, completed=%d\r\n", right_sensor_int, left_sensor_int,completed);
			HAL_UART_Transmit(&huart3,(uint32_t *) &buffer,strlen(buffer),0xFFFF);

			LPF_SUM_right = 0;
			LPF_SUM_left = 0;
			counter = 0;
		}
		osDelay(1);
  }
  /* USER CODE END ir_sensor */
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
  uint8_t hello[20] = "error!\0";
  while (1)
  {
	  OLED_ShowString(40,10,hello);
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

