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
#include <stdio.h>
#include <stdint.h>
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
int getLiDarRead();
float getSpeed();
void writeSPIchar(char c);
void writeSPIstring(char str[]);
void writeSPIfloat(float f);
void writeSPIlongfloat(float f);
void writeSPIint(int n);
void SPIinit();
void SPIreset();
void setServoPosition(uint16_t pulse_width);
float ball_travel_time(float velocity);
float linearEase(float t, float b, float c, float d);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int distances[2];
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
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  SPIinit();
  /*
  //Demonstrate distance data
  float dist = getLiDarRead();
  writeSPIlongfloat(dist);
  writeSPIstring("cm");
  */

  /*
  //Demonstrate speed capabilities
  writeSPIstring("Waiting...");
  float speed = getSpeed();
  SPIreset();
  HAL_Delay(10);
  writeSPIlongfloat(speed);
  writeSPIstring("mph");
  */

  /*
  //Demonstrate speed calculation is fast enough
  writeSPIstring("Waiting...");
  float speed = getSpeed();
  uint16_t timer_val = __HAL_TIM_GET_COUNTER(&htim16);
  SPIreset();
  HAL_Delay(10);
  writeSPIlongfloat(timer_val / 1000.0);
  writeSPIstring("sec");
  */

  // Start timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int LANECHOICE = 0;
  int distance;
  uint8_t splitDists[4];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  //button 1
//	  	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) { //power
//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
//	  	  }
//	  	  else{
//	  		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
//	  	  }

	  	  //button 2
	  	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) != GPIO_PIN_RESET){ //reset

	  	   }
	  	   else{
	  	      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	  	      if(LANECHOICE == 0) {
	  	    	  setServoPosition(1200); //left
	  	    	  SPIreset();
	  	      }
	  	      else {
	  	    	  setServoPosition(275); //right
	  	    	  SPIreset();
	  	      }
	  	   }

	  	  //button 3
	  	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) != GPIO_PIN_RESET){ //record

	  	   }
	  	   else{
	  	      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	  	      writeSPIstring("Waiting...");
	  	      float speed = getSpeed(); //mph
	  	      float travel_time = ball_travel_time(speed) * 1000;
	  	      float half_time = travel_time / 2;
	  	      if(LANECHOICE == 0) {
	  	    	  HAL_Delay(20);
	  	    	  setServoPosition(980);
				  distance = getLiDarRead();
				  while ((distance > 1800) || (distance < 6)){ //106.68 is width of bowling lane in cm, extra for some leeway //50 and 6
				  	distance = getLiDarRead();
				  }
				  distances[1] = distance;
	  	    	  HAL_Delay(20);
	  	    	  setServoPosition(760);
	  	      }
	  	      else {
	  	    	  HAL_Delay(20);
	  	    	  setServoPosition(517.5);
				  distance = getLiDarRead();
				  while ((distance > 1800) || (distance < 6)){ //106.68 is width of bowling lane in cm, extra for some leeway //50 and 6
				  	distance = getLiDarRead();
				  }
				  distances[1] = distance;
	  	    	  HAL_Delay(20);
	  	    	  setServoPosition(760);
	  	      }

	  	      SPIreset();
	  	      HAL_Delay(10);
	  	      writeSPIlongfloat(speed);
	  	      writeSPIstring("mph");
	  	      for (int i = 0; i < 2; i++){
	  	    	  splitDists[2*i] = distances[i] & 0xff;
	  	    	  splitDists[2*i+1] = (distances[i] & 0xff00) >> 8;
	  	      }
	  	      HAL_UART_Transmit (&huart1, splitDists, sizeof(uint8_t) * 4, 100);
	  	      HAL_Delay(1000);
	  	      //SPIreset();

	  	   }

	  	  //button 4
	  	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) != GPIO_PIN_RESET){ //lane switch

	  	  }
	  	   else{
	  	      //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
	  	      if(LANECHOICE == 0) {
	  	    	  LANECHOICE = 1;
	  	    	  writeSPIstring("RIGHT");
	  	    	  HAL_Delay(1000);
	  	    	  SPIreset();
	  	      }
	  	      else {
	  	    	  LANECHOICE = 0;
	  	    	  writeSPIstring("LEFT");
	  	    	  HAL_Delay(1000);
	  	    	  SPIreset();
	  	      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0000020B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_10BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int getLiDarRead(){
	  //Set buffers for data
	  uint8_t dataInBuffer = 0x04;
	  uint8_t dataOutBuffer = 0x01;
	  uint8_t dataHigh;
	  uint8_t dataLow;
	  //Request the LiDar to read a point
	  HAL_I2C_Mem_Write(&hi2c2, 0xC4, 0x00, I2C_MEMADD_SIZE_8BIT, &dataInBuffer, 1, HAL_MAX_DELAY);
	  HAL_Delay(5);
	  //Wait for the data from the LiDar
	  do {
	  	  HAL_I2C_Mem_Read(&hi2c2, 0xC5, 0x01, I2C_MEMADD_SIZE_8BIT, &dataOutBuffer, 1, HAL_MAX_DELAY);
	  } while ((dataOutBuffer % 2) == 1);
	  //Read in data from the Lidar
	  HAL_I2C_Mem_Read(&hi2c2, 0xC5, 0x0f, I2C_MEMADD_SIZE_8BIT, &dataHigh, 1, HAL_MAX_DELAY);
	  HAL_I2C_Mem_Read(&hi2c2, 0xC5, 0x10, I2C_MEMADD_SIZE_8BIT, &dataLow, 1, HAL_MAX_DELAY);
	  //Convert dataHigh & dataLow to int - distance in cm
	  return (int)(((uint16_t)dataHigh << 8) | dataLow);
}

float getSpeed(){
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	uint32_t timer_val;
	int distance = getLiDarRead();
	HAL_Delay(1000);
	while ((distance > 120) || (distance < 6)){ //106.68 is width of bowling lane in cm, extra for some leeway //50 and 6
		distance = getLiDarRead();
	}
	HAL_TIM_Base_Start(&htim7);
	while (distance < 120){
		distance = getLiDarRead();
	}
	distances[0] = distance;
	HAL_TIM_Base_Stop(&htim7);
	timer_val = __HAL_TIM_GET_COUNTER(&htim7);
	return ((486.809 / timer_val)); // Speed in mph, 21.83, width of bowling ball in cm
	//prev = 1000(clock rate) * 0.0223(conversion rate) * 26.67 (notebook width)
	//new = 1000/s * 0.0223(mile seconds / cm hours) * 21.83cm
}

void writeSPIchar(char c){
	  if (c == ' '){
		  return;
	  }
	  uint16_t transmit_val = (0b10 << 8) | c;
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &transmit_val, 1, HAL_MAX_DELAY);
	  HAL_Delay(20);
}

void writeSPIstring(char str[]){
    int count;
    for (count = 0; str[count] != '\0'; ++count);
    for (int i = 0; i < count; i++){
  	  writeSPIchar(str[i]);
    }
}

void writeSPIfloat(float f){
	  //Convert float to string
	  int nums = (f * 10);
	  char str[4];
      sprintf(str, "%3d", nums);

      for (int i = 0; i < 2; i++){
    	  writeSPIchar(str[i]);
      }
	  	  writeSPIchar('.');
		  writeSPIchar(str[2]);
}

void writeSPIlongfloat(float f){
	  //Convert float to string
	  int nums = (f * 100);
	  char str[7];
      sprintf(str, "%6d", nums);

      for (int i = 0; i < 4; i++){
    	  writeSPIchar(str[i]);
      }
	  	  writeSPIchar('.');
	  for (int i = 4; i < 6; i++){
		  writeSPIchar(str[i]);
	  }
}

void writeSPIint(int n){
	  //Convert float to string
	  char str[7];
      sprintf(str, "%6d", n);

      for (int i = 0; i < 6; i++){
    	  writeSPIchar(str[i]);
      }
}

void SPIinit(){
	  uint16_t my_value;
	  my_value = 0b0000001000; //Display off
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  my_value = 0b0000111000; //Function set
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  my_value = 0b0000000001; //Clear display
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  my_value = 0b0000000110; //Set entry mode
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  my_value = 0b0000000010; //To home
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  my_value = 0b0000001111; //Display on
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  HAL_Delay(20);
}

void SPIreset(){
	  uint16_t my_value;
	  my_value = 0b0000000001; //Clear display
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  my_value = 0b0000000010; //To home
	  HAL_SPI_Transmit(&hspi1, (uint8_t*) &my_value, 1, HAL_MAX_DELAY);
	  HAL_Delay(20);
}

void setServoPosition(uint16_t pulse_width) {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse_width;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

float ball_travel_time(float velocity) {
	//float mph = velocity / 0.44704;
	float lane_length = 60;
	float ft_psec = velocity * 1.467;
	float ball_tt = lane_length / ft_psec;
	return ball_tt;
}

float linearEase(float t, float b, float c, float d) {
    return c * (t / d) + b;
}
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
