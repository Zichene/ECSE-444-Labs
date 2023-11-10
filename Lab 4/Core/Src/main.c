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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01.h"
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4s5i_iot01_qspi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define true 1
#define false 0
#define WRITE_READ_ADDR_TEMP 0x0000
#define WRITE_READ_ADDR_PRESSURE 0x1000
#define WRITE_READ_ADDR_GYROX 0x2000
#define WRITE_READ_ADDR_GYROY 0x3000
#define WRITE_READ_ADDR_GYROZ 0x4000
#define WRITE_READ_ADDR_MAGNETOX 0x5000
#define WRITE_READ_ADDR_MAGNETOY 0x6000
#define WRITE_READ_ADDR_MAGNETOZ 0x7000
#define MAX_SAMPLES 1000 // maximum values for samples
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId read_sensorsHandle;
osThreadId checkButtonHandle;
osThreadId sendToUARTHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void OsTask_readSensors(void const * argument);
void OsTask_checkButton(void const * argument);
void OsTask_sendToUART(void const * argument);

/* USER CODE BEGIN PFP */
void writeSensorsToFlash(void);
void readSampleFlash(void);
void computeAvg(void);
void computeStats(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// global vars
// flags for interrupts
uint8_t tim2_it_flag = false;
uint8_t buttonPressed = false;
// global vars
float temperature = 0;
float pressure = 0;
int16_t magneto[3] = {0,0,0};
float gyro[3] = {0,0,0};
float tempVals[1000];
float pressureVals[1000];
int16_t magnetoXVals[1000];
int16_t magnetoYVals[1000];
int16_t magnetoZVals[1000];
float gyroXVals[1000];
float gyroYVals[1000];
float gyroZVals[1000];
uint16_t numSamples = 0;
float samplesAvg[8];
float samplesVariance[8];
char message[200] = "";
enum SENSOR_TYPE {
	TEMP,
	PRESSURE,
	MAGNETOMETER,
	GYRO,
	STATISTICS
};
enum SENSOR_VALS {
	TEMP_VAL,
	PRESSURE_VAL,
	MAG_X_VAL,
	MAG_Y_VAL,
	MAG_Z_VAL,
	GYRO_X_VAL,
	GYRO_Y_VAL,
	GYRO_Z_VAL
};
uint8_t currentSensor = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//uint8_t example_arr2[5] = {69,1,420,1,69};

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
  MX_USART1_UART_Init();
  MX_OCTOSPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  // init stuff
  // humidity & temp
  BSP_HSENSOR_Init();
  BSP_TSENSOR_Init();
  // pressure
  BSP_PSENSOR_Init();
  // accel & gyro
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  // magnetometer
  BSP_MAGNETO_Init();
  // qspi
  BSP_QSPI_Init();

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of read_sensors */
  osThreadDef(read_sensors, OsTask_readSensors, osPriorityNormal, 0, 256);
  read_sensorsHandle = osThreadCreate(osThread(read_sensors), NULL);

  /* definition and creation of checkButton */
  osThreadDef(checkButton, OsTask_checkButton, osPriorityNormal, 0, 128);
  checkButtonHandle = osThreadCreate(osThread(checkButton), NULL);

  /* definition and creation of sendToUART */
  osThreadDef(sendToUART, OsTask_sendToUART, osPriorityNormal, 0, 256);
  sendToUARTHandle = osThreadCreate(osThread(sendToUART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	// erase block
  if (BSP_QSPI_Erase_Block(0) != QSPI_OK)
	 Error_Handler();
  if (BSP_QSPI_Erase_Block(1) != QSPI_OK)
	 Error_Handler();
  // reset error LED
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  temperature = BSP_TSENSOR_ReadTemp();
	  BSP_GYRO_GetXYZ(gyro);
	  snprintf(message, 200, "GyroX: %f, GyroY: %f, GyroZ: %f\r\n", gyro[0], gyro[1], gyro[2]);
	  HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
	  */
	  /*

	  while (!tim2_it_flag) {

	  }
	  tim2_it_flag = false; // reset flag
*/
/*
	  if (BSP_QSPI_Erase_Block((uint32_t) WRITE_READ_ADDR) != QSPI_OK)
		  Error_Handler();
	  if (BSP_QSPI_Write(example_arr, (uint32_t) WRITE_READ_ADDR,sizeof(example_arr)) != QSPI_OK)
		  Error_Handler();
	  if (BSP_QSPI_Read(example_copy, (uint32_t) WRITE_READ_ADDR,sizeof(example_copy)) != QSPI_OK)
		  Error_Handler();
		  */


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c2.Init.Timing = 0x307075B1;
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
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 120000000;
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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB_Blue_Pin */
  GPIO_InitStruct.Pin = PB_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_Blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*------------------------- INTERRUPTS --------------------------*/
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	tim2_it_flag = (tim2_it_flag+1)%2; // toggle flag
}
*/
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
	// button
	if (GPIO_Pin == PB_Blue_Pin) {
		buttonPressed = true;
	}
}

/**
 * Write using sensor data using QSPI to flash
 */
void writeSensorsToFlash() {
	if (numSamples <= MAX_SAMPLES) {
		// writing to flash
		if (BSP_QSPI_Write(&temperature, WRITE_READ_ADDR_TEMP+numSamples*sizeof(float), sizeof(float)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write(&pressure, WRITE_READ_ADDR_PRESSURE+numSamples*sizeof(float), sizeof(float)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write(magneto, WRITE_READ_ADDR_MAGNETOX+numSamples*sizeof(int16_t), sizeof(int16_t)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write(&(magneto[1]), WRITE_READ_ADDR_MAGNETOY+numSamples*sizeof(int16_t), sizeof(int16_t)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write(&(magneto[2]), WRITE_READ_ADDR_MAGNETOZ+numSamples*sizeof(int16_t), sizeof(int16_t)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write(gyro, WRITE_READ_ADDR_GYROX+numSamples*sizeof(float), sizeof(float)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write(&(gyro[1]), WRITE_READ_ADDR_GYROY+numSamples*sizeof(float), sizeof(float)) != QSPI_OK)
			Error_Handler();
		if (BSP_QSPI_Write(&(gyro[2]), WRITE_READ_ADDR_GYROZ+numSamples*sizeof(float), sizeof(float)) != QSPI_OK)
			Error_Handler();
		// increment num of values
		numSamples++;
	} else {
		// we have reached a maximum amount of data, erase block and reset counter
		if (BSP_QSPI_Erase_Block(0) != QSPI_OK)
			Error_Handler();
		numSamples = 0;
	}
}

/**
 * Read values stored in flash
 */
void readSampleFlash() {
	// read all sensor values
	if (BSP_QSPI_Read(tempVals, WRITE_READ_ADDR_TEMP, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
	if (BSP_QSPI_Read(pressureVals, WRITE_READ_ADDR_PRESSURE, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
	if (BSP_QSPI_Read(magnetoXVals, WRITE_READ_ADDR_MAGNETOX, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
	if (BSP_QSPI_Read(magnetoYVals, WRITE_READ_ADDR_MAGNETOY, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
	if (BSP_QSPI_Read(magnetoZVals, WRITE_READ_ADDR_MAGNETOZ, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
	if (BSP_QSPI_Read(gyroXVals, WRITE_READ_ADDR_GYROX, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
	if (BSP_QSPI_Read(gyroYVals, WRITE_READ_ADDR_GYROY, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
	if (BSP_QSPI_Read(gyroZVals, WRITE_READ_ADDR_GYROZ, numSamples*sizeof(float)) != QSPI_OK)
		Error_Handler();
}

/**
 * Computes the averages of all the samples, stores them in samplesAvg global array
 */
void computeAvg() {
	float sumTemp;
	float sumPressure;
	float sumMagnetoX;
	float sumMagnetoY;
	float sumMagnetoZ;
	float sumGyroX;
	float sumGyroY;
	float sumGyroZ;
	for (int i = 0; i < numSamples; i++) {
		sumTemp += tempVals[i];
		sumPressure += pressureVals[i];
		sumMagnetoX += (float)magnetoXVals[i];
		sumMagnetoY += (float)magnetoYVals[i];
		sumMagnetoZ += (float)magnetoZVals[i];
		sumGyroX += gyroXVals[i];
		sumGyroY += gyroYVals[i];
		sumGyroZ += gyroZVals[i];

	}
	if (numSamples != 0) {
		samplesAvg[TEMP_VAL] = sumTemp/numSamples;
		samplesAvg[PRESSURE_VAL] = sumPressure/numSamples;
		samplesAvg[MAG_X_VAL] = sumMagnetoX/numSamples;
		samplesAvg[MAG_Y_VAL] = sumMagnetoY/numSamples;
		samplesAvg[MAG_Z_VAL] = sumMagnetoZ/numSamples;
		samplesAvg[GYRO_X_VAL] = sumGyroX/numSamples;
		samplesAvg[GYRO_Y_VAL] = sumGyroY/numSamples;
		samplesAvg[GYRO_Z_VAL] = sumGyroZ/numSamples;

	}
}

void computeStats(){
    computeAvg();
    float avgTemp;
    float avgPressure;
	float avgMagnetoX;
	float avgMagnetoY;
	float avgMagnetoZ;
	float avgGyroX;
	float avgGyroY;
	float avgGyroZ;
	float varianceSumTemp;
    float varianceSumPressure;
    float varianceSumMagnetoX;
    float varianceSumMagnetoY;
    float varianceSumMagnetoZ;
    float varianceSumGyroX;
    float varianceSumGyroY;
    float varianceSumGyroZ;
    if (numSamples != 0) {
        avgTemp=samplesAvg[TEMP_VAL];
        avgPressure=samplesAvg[PRESSURE_VAL];
        avgMagnetoX=(float)samplesAvg[MAG_X_VAL];
        avgMagnetoY=(float)samplesAvg[MAG_Y_VAL];
        avgMagnetoZ=(float)samplesAvg[MAG_Z_VAL];
        avgGyroX=samplesAvg[GYRO_X_VAL];
        avgGyroY=samplesAvg[GYRO_Y_VAL];
        avgGyroZ=samplesAvg[GYRO_Z_VAL];


        for (int i=0; i < numSamples;i++){
            varianceSumTemp += (tempVals[i]-avgTemp)*(tempVals[i]-avgTemp);
            varianceSumPressure += (pressureVals[i]-avgPressure)*(pressureVals[i]-avgPressure);
            varianceSumMagnetoX += (magnetoXVals[i]-avgMagnetoX)*(magnetoXVals[i]-avgMagnetoX);
            varianceSumMagnetoY += (magnetoYVals[i]-avgMagnetoY)*(magnetoYVals[i]-avgMagnetoY);
            varianceSumMagnetoZ += (magnetoZVals[i]-avgMagnetoZ)*(magnetoZVals[i]-avgMagnetoZ);
            varianceSumGyroX += (gyroXVals[i]-avgGyroX)*(gyroXVals[i]-avgGyroX);
            varianceSumGyroY += (gyroYVals[i]-avgGyroY)*(gyroYVals[i]-avgGyroY);
            varianceSumGyroZ += (gyroZVals[i]-avgGyroZ)*(gyroZVals[i]-avgGyroZ);
        }
        samplesVariance[TEMP_VAL] = varianceSumTemp/(numSamples-1);
        samplesVariance[PRESSURE_VAL] = varianceSumPressure/(numSamples-1);
        samplesVariance[MAG_X_VAL] = varianceSumMagnetoX/(numSamples-1);
        samplesVariance[MAG_Y_VAL] = varianceSumMagnetoY/(numSamples-1);
        samplesVariance[MAG_Z_VAL] = varianceSumMagnetoZ/(numSamples-1);
        samplesVariance[GYRO_X_VAL] = varianceSumGyroX/(numSamples-1);
        samplesVariance[GYRO_Y_VAL] = varianceSumGyroY/(numSamples-1);
        samplesVariance[GYRO_Z_VAL] = varianceSumGyroZ/(numSamples-1);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_OsTask_readSensors */
/**
* @brief Function implementing the read_sensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OsTask_readSensors */
void OsTask_readSensors(void const * argument)
{
  /* USER CODE BEGIN OsTask_readSensors */
  /* Infinite loop */
  for(;;)
  {
  temperature = BSP_TSENSOR_ReadTemp();
  pressure = BSP_PSENSOR_ReadPressure();
  BSP_MAGNETO_GetXYZ((int16_t*)magneto);
  BSP_GYRO_GetXYZ(gyro);
  writeSensorsToFlash();
  readSampleFlash();
  osDelay(1000);
  }
  /* USER CODE END OsTask_readSensors */
}

/* USER CODE BEGIN Header_OsTask_checkButton */
/**
* @brief Function implementing the checkButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OsTask_checkButton */
void OsTask_checkButton(void const * argument)
{
  /* USER CODE BEGIN OsTask_checkButton */
  /* Infinite loop */
  for(;;)
  {
	while(true) {
		if (buttonPressed)
			break;
	}
	buttonPressed = false;
	currentSensor = (currentSensor+1)%5; // toggle sensorType
    osDelay(1);
  }
  /* USER CODE END OsTask_checkButton */
}

/* USER CODE BEGIN Header_OsTask_sendToUART */
/**
* @brief Function implementing the sendToUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OsTask_sendToUART */
void OsTask_sendToUART(void const * argument)
{
  /* USER CODE BEGIN OsTask_sendToUART */
  /* Infinite loop */
  for(;;)
  {
	if (currentSensor == STATISTICS) {
		readSampleFlash();
		computeStats();
		snprintf(message, 200, "Current statistics over %d samples: \r\n \t\t AvgTemp: %f \t\t AvgPressure: %f \t\t\r\n", numSamples, samplesAvg[TEMP_VAL], samplesAvg[PRESSURE_VAL]);
		HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
		memset(message, 0, sizeof(message));
		snprintf(message, 200, "\r\n \t\t VarTemp: %f \t\t VarPressure: %f \t\t\r\n", samplesVariance[TEMP_VAL], samplesVariance[PRESSURE_VAL]);
		HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
		memset(message, 0, sizeof(message));
		snprintf(message, 200, "\r\n \t\t AvgMagnetoX: %f \t\t AvgMagnetoY: %f \t\t AvgMagnetoZ: %f \t\t\r\n", samplesAvg[MAG_X_VAL], samplesAvg[MAG_Y_VAL], samplesAvg[MAG_Z_VAL]);
		HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
		memset(message, 0, sizeof(message));
		snprintf(message, 200, "\r\n \t\t VarMagnetoX: %f \t\t VarMagnetoY: %f \t\t VarMagnetoZ: %f \t\t\r\n", samplesVariance[MAG_X_VAL], samplesVariance[MAG_Y_VAL], samplesVariance[MAG_Z_VAL]);
		HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
		memset(message, 0, sizeof(message));
		snprintf(message, 200, "\r\n \t\t AvgGyroX: %f \t\t AvgGyroY: %f \t\t AvgGyroZ: %f \t\t\r\n", samplesAvg[GYRO_X_VAL], samplesAvg[GYRO_Y_VAL], samplesAvg[GYRO_Z_VAL]);
		HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
		memset(message, 0, sizeof(message));
		snprintf(message, 200, "\r\n \t\t VarGyroX: %f \t\t VarGyroY: %f \t\t VarGyroZ: %f \t\t\r\n", samplesVariance[GYRO_X_VAL], samplesVariance[GYRO_Y_VAL], samplesVariance[GYRO_Z_VAL]);
		HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
		memset(message, 0, sizeof(message));
		while (currentSensor == STATISTICS) {
			// wait for button press
		}
		continue;
	}
	snprintf(message, 200, "currentSensor: %d\r\n",currentSensor);
	HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
	memset(message, 0, sizeof(message));
	switch (currentSensor) {
		case TEMP:
			snprintf(message, 200, "Temperature: %f\r\n",temperature);
			break;
		case PRESSURE:
			snprintf(message, 200, "Pressure: %f\r\n", pressure);
		    break;
		case MAGNETOMETER:
			snprintf(message, 200, "MagnetoX: %d, MagnetoY: %d, MagnetoZ: %d\r\n", magneto[0], magneto[1], magneto[2]);
			break;
		case GYRO:
			snprintf(message, 200, "GyroX: %f, GyroY: %f, GyroZ: %f\r\n", gyro[0], gyro[1], gyro[2]);
			break;
	}
	HAL_UART_Transmit(&huart1, (uint8_t *) message, sizeof(message), 1000);
	memset(message, 0, sizeof(message));
	osDelay(1000);
  }
  /* USER CODE END OsTask_sendToUART */
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
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
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
