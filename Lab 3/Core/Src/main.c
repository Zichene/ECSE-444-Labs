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
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.14
#define true 1
#define false 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
/* USER CODE BEGIN PFP */
void populateSineWave8Bit(uint8_t *array);
void changeWaveFrequency(int frequency);
void transformBufferToDAC(int32_t *buffer, uint32_t recording_buffer_length, uint32_t total_buffer_length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// init variables
uint8_t waveCounter = 0;
uint8_t sine[15];
const int sampleFrequency = 44100; // 44.1 KHz
const int samplePeriod = 120000000/sampleFrequency; // Clock freq / sampleFrequency
int waveFreqCountPeriod = 1;
int waveFreqCounter = 0;
// buffer for sound samples
const uint32_t RECORDING_BUFLEN = 65500; // buffer length purely for recording
const uint32_t VOICE_BUFLEN = 40000; // buffer length recording+chime
int32_t recordingBuffer[65500]; // sampling rate @ 20 kHz => 2 second of recording and 1 second of space for chime
// boolean flags
uint8_t DFSDM_finished = false;
uint8_t lightState = 0; //0 is off, 1 is on
uint8_t lightBlink = 0; //0 is normal, 1 is blinking
uint16_t blinkFreqReducerCount=0;
uint16_t blinkFreqReducerPeriod=8000;

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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
  // initiating things
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  // start the timer (TIM2) and associated interrupt
  HAL_TIM_Base_Start_IT(&htim2); // the _IT at the end of fn. means interrupt
  populateSineWave8Bit(sine);
  /* ------------- PART 3 OF LAB COMMENT OUT BELOW -------------------
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, recordingBuffer, RECORDING_BUFLEN);
  while (!DFSDM_finished) {
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
  transformBufferToDAC(recordingBuffer, RECORDING_BUFLEN);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, recordingBuffer, RECORDING_BUFLEN, DAC_ALIGN_8B_R);
  */
  /* ------------- PART 2 OF LAB COMMENTED OUT BELOW ------------------
  // initiating things
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, sine, 15, DAC_ALIGN_8B_R);
  // start the timer (TIM2) and associated interrupt
  HAL_TIM_Base_Start_IT(&htim2); // the _IT at the end of fn. means interrupt
  populateSineWave8Bit(sine);
  changeWaveFrequency(2000); // 2kHz
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (true)
  {

      if (lightState==1 && lightBlink==0){
          DFSDM_finished = false;
          lightBlink = 1;
          HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, recordingBuffer, VOICE_BUFLEN);
          while (!DFSDM_finished) {
          }
          HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
          lightBlink = 0;

        	  // make LED solid
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
          lightState = 1;

          transformBufferToDAC(recordingBuffer, VOICE_BUFLEN, RECORDING_BUFLEN);
          HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, recordingBuffer, RECORDING_BUFLEN, DAC_ALIGN_8B_R);
          while(lightState==1){

          }
          // stop playback on button press
          HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
          lightState = 1;
      }
	  // debug
	  /*
		int32_t cur;
		int32_t min = INT32_MAX;
		int32_t max = INT32_MIN;
		int32_t avg = 0;
		for (int i = 0; i < RECORDING_BUFLEN; i++) {
			cur = recordingBuffer[i];
			if (cur <= min) {
				min = cur;
			}

			if (cur >= max) {
				max = cur;
			}
		}
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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_FASTSINC_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 100;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 50;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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
  // THE PERIOD OF TIM2 HAS BEEN DETERMINED BY f_SAMPLE = 44.1 KHz AND f_CLK = 120 MHz
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB_BLUE_Pin */
  GPIO_InitStruct.Pin = PB_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* INTERRUPT FUNCTIONS*/
/* PART 1-2
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PB_BLUE_Pin) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}
*/
/* PART 1-2
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	if (htim == &htim2 && waveFreqCounter == waveFreqCountPeriod) {
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, sine[waveCounter]);
		waveCounter = (waveCounter+1)%15;
		// reset
		waveFreqCounter = 0;
	}
	waveFreqCounter++;
}
*/

/**
 * Part 3
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == PB_BLUE_Pin) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        if (lightState==0){
            lightState=1;
        }else{
            lightState=0;
        }
        blinkFreqReducerCount=0;
    }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	//Part 3
	if(lightBlink==1 && blinkFreqReducerCount==blinkFreqReducerPeriod){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        if (lightState==0){
            lightState=1;
        }else{
            lightState=0;
        }
        blinkFreqReducerCount=0;
	}

	blinkFreqReducerCount++;

}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter) {
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	DFSDM_finished = true;
}

void HAL_DAC_ConvCpltCallbackCh1 (DAC_HandleTypeDef * hdac) {
	int a = 1;
}

/* OTHER STUFF/ HELPER FUNCTIONS*/
/*
 * Populates an array with values of a sine wave for the DAC (8 bit right alligned)
 */
void populateSineWave8Bit(uint8_t *array) {
	const uint8_t ARRAY_LEN = 16;
	int val = 0;
	for (int i = 0; i < ARRAY_LEN; i++) {
		val = 90*arm_sin_f32((pi*i) / 8) + 90;
		array[i] = val;
		// debug
		// int debug = array[i];
	}
}


void changeWaveFrequency(int frequency) {
	waveFreqCounter = 0;
	waveFreqCountPeriod = sampleFrequency/frequency;
}

/**
 * Transforms a buffer's values into valid DAC 8bit right aligned values
 */
void transformBufferToDAC(int32_t *buffer, uint32_t recording_buffer_length, uint32_t total_buffer_length) {
	for (int i = 0; i < recording_buffer_length; i++) {
		int32_t val = buffer[i]; // 24-bit value
		val = val >> 8; // remove this for LOUDER but MORE SCUFFED NOISE
		// need to map buffer values to 8bit right alligned values (uint8_t)
		// from experimentation (screaming at the board): min values tend to be -3000 and max seems to be ~1000
		const int16_t MAX_VAL = 2000;
		const int16_t MIN_VAL = -1500;
		const float a = (255.0)/(MAX_VAL - MIN_VAL); // slope

		// clip buffer values to within [-MIN_VAL, MAX_VAL]
		if (val <= MIN_VAL) {
			val = MIN_VAL;
		}
		if (val >= MAX_VAL) {
			val = MAX_VAL;
		}
		// scale values up by [-MIN_VAL] to make sure no negatives
		if (MIN_VAL < 0) {
			val += (-MIN_VAL);
		}
		// now the range of val should be [0, MAX_VAL-MIN_VAL], apply linear function to get DAC val
		val = round(a*val);
		if (val >= 0 && val <= 255) {
			buffer[i] = val; // change the buffer
		} else {
			Error_Handler(); // should not happen
		}
	}
	// adding the chime at the end of the buffer

    //int32_t step = (total_buffer_length-recording_buffer_length)/16;
    int8_t sineIndex = 0;
    /*
    for (int i=0; i<15;i++){
        for (int j=recording_buffer_length+(step*i); j<recording_buffer_length+(step*(i+1));j++){
            int32_t val = sine[sineIndex];
            if (val >= 0 && val <= 255) {
                buffer[j] = val; // change the buffer
            } else {
                Error_Handler(); // should not happen
            }
            sineIndex = (sineIndex+1)%16;
        }
   }
   */
    /*
    for (int i=0; i<13;i++){
        for (int j=recording_buffer_length+(step*i); j<recording_buffer_length+(step*(i+1));j++){
            int32_t val = sine[sineIndex];
            if (val >= 0 && val <= 255) {
                if (i%2==1 && j%4>0){
                    val=0;
                } else{
                    val = sine[j%13];
                }
                buffer[j] = val; // change the buffer
            } else {
                Error_Handler(); // should not happen
            }
        }
    }
    */

    int32_t step = (total_buffer_length-recording_buffer_length)/14;
    for (int i=0; i<13;i++){
        for (int j=recording_buffer_length+(step*i); j<recording_buffer_length+(step*(i+1));j++){
            int32_t val = sine[sineIndex];
            if (i%2==1 && j%15>4 && j%15<13){
                val=0;
            } else{
                val = sine[j%15];
            }
            if (val >= 0 && val <= 255) {
                buffer[j] = val; // change the buffer
            } else {
                Error_Handler(); // should not happen
            }
        }
    }
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
