/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>
#include "sine_table.h"

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



#define PI 3.141592653589793
#define TAU 6.28318530717958647692
#define MAX_16_BIT 65535
#define MAX_16_BIT_SIDE 32767
#define MAX_32_BIT 4294967295
#define SAMPLE_FREQ 48000

#define DMA_BUFFER_SAMPLES 256
#define DMA_BUFFER_SIZE 2 * DMA_BUFFER_SAMPLES //2* accounts for double buffering

#define ADC_SIZE 2

#define A4 440
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch3;

/* USER CODE BEGIN PV */



//dma buffer used for pwm output
 uint16_t pwm_buffer[DMA_BUFFER_SIZE];

//dma buffer used for i2s output
int16_t i2s_buffer[2*DMA_BUFFER_SIZE]; //2* for left and right channels
uint8_t dma_buffer_processing = 0;
uint32_t dma_overflow = 0;

volatile float frequency = 220;
float frequency_range = 2; //double and half frequency
float phase = 0;
float phase_change = 0;


float angle = 0;


//master volume
float volume = 1;


uint16_t wavetable_index = 0; //sine table index


//potentiometer = [0], piezo = [1]
volatile uint16_t ADC_buffer[ADC_SIZE] = {0}; //ADC DMA readings go into this buffer.
volatile uint8_t ADC_id = 0;

uint32_t ADC_channels[ADC_SIZE] = {ADC_CHANNEL_5, ADC_CHANNEL_6};
ADC_ChannelConfTypeDef ADC_CH_Cfg = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int _write(int file, char *ptr, int len) {
    // Send data over USB CDC
    CDC_Transmit_FS((uint8_t*)ptr, len);

    // Optional: Add a brief delay to prevent buffer overflow on rapid prints
    HAL_Delay(1);

    return len;
}


//updates the step size of phase. turns frequency input into step size.
void update_phase_change(){
	float x = (frequency/(float)SAMPLE_FREQ);
	phase_change = ( x*TAU);
}


//this doesnt work and is shit, make it good monkey
void update_frequency(){

	float pot = ((ADC_buffer[0]>>20))/4096; //float between 0 and 1, >>4 as converting from 32 bit to 12 bit, which is the true ADC size
	pot = pot*2 -1;// -1 to 1

	float x = pot*frequency_range;

	frequency = ((float)A4)*x;

	update_phase_change();
}



/*
 *THIS IS HOW PWM AND TIMERS WORK AND HOW TO CHOOSE VALUES
 *
 * There is a tradeoff between high duty cycle resolution and high pwm frequency.
 * ARR basically splits up each pwm period into however many discrete steps, which controls duty resolution
 * system and timer clock is 96MHz. ARR is incremented each clock cycle.
 * this means that the length of a pwm frequency period is ~ system clock/ARR
 * therefore increasing ARR gives more duty resolution, but makes a pwm period take longer.
 *
 * ARR = 2047, gives carrier frequency of ~46k
 * duty = CRR/(ARR+1), ARR gives duty resolution of 11 bits (2047 steps).
 *
 * duty can be between 0 and 2047. dma_buffer stores the 16 bit output values.
 *
 */


//this function generates values for the pwm_buffer, which is fed to the DMA.
//Using a double buffered approach, so pwm_buffer input is only one half of the actual pwm_buffer
//pwm output is an unsigned 32 bit int.
void process_buffer_pwm(uint16_t *pwm_buffer){
	dma_buffer_processing = 1;

	//iterate through half the buffer
	for(int i = 0; i <DMA_BUFFER_SIZE/2; i++){

		wavetable_index = (uint16_t)(sizeof(sin2048)-1)*((float)phase/(float)TAU); //phase/tau returns value between 0 and 1

		float cur_sine = (float)sin2048[wavetable_index]/255; //sine as a float value 0 to 1. sine_table stored as uint8_t instead of float as cheaper to store

		//pwm duty cycle is from 1-2048. pwm_buffer values go to the CRR of the timer, which must be <= the ARR, which in this case is 2048.

		uint16_t buffer_out = (uint16_t)(volume*cur_sine*2048.0f);


		pwm_buffer[i] = buffer_out;

		phase += phase_change;
		if(phase >TAU){
			phase -=TAU;
		}
	}

	dma_buffer_processing = 0;
}


//same process as for pwm buffer, just formatted for i2s
void process_buffer_i2s(int16_t*i2s_buffer){
	dma_buffer_processing = 1;

	//left and right channels
	for(int i = 0; i < DMA_BUFFER_SAMPLES; i+=2){

		wavetable_index = (uint16_t)sizeof(sin2048)*(phase/TAU);

		float cur_sine = ((float)sin2048[wavetable_index] - 127)/128; //sine table is 0 to 255, converts to -1 to 1.

		int16_t buffer_out = (volume*cur_sine)*(float)MAX_16_BIT_SIDE; //converts to 16 bit int

		i2s_buffer[i] = buffer_out;
		i2s_buffer[i+1] = 0;//right channel not needed, mono output. I2S requires 2 channels

		phase += phase_change;
		if(phase > TAU){
			phase -= TAU;
		}

	}

	dma_buffer_processing = 0;
}



////// I2S Callback

//double buffered output
//once DMA buffer is completed, begin processing new second half while first half transmitting
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s){
	if(dma_buffer_processing){
		++dma_overflow;
	}
	else{
		process_buffer_i2s(&i2s_buffer[DMA_BUFFER_SIZE]); //DMA_BUFFER_SIZE is halfway through i2s buffer, i2s buffer is 2*DMA_BUFFER_SIZE long
	}
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if(dma_buffer_processing){
			++dma_overflow;
		}
		else{
			process_buffer_i2s(&i2s_buffer[0]);
		}
}



/////  PWM Callback

//double buffered output
//once DMA buffer is completed, begin processing new second half while first half transmitting
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim3){
	if(dma_buffer_processing){
		++dma_overflow;
	}
	else{
		process_buffer_pwm(&pwm_buffer[DMA_BUFFER_SIZE/2]); //halfway through buffer
	}
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim3) {
	if(dma_buffer_processing){
			++dma_overflow;
	}
	else{
		process_buffer_pwm(&pwm_buffer[0]);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  process_buffer_pwm(&pwm_buffer[0]);                    // Fill first half
  process_buffer_pwm(&pwm_buffer[DMA_BUFFER_SIZE / 2]);  // Fill second half

  //HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) &i2s_buffer,DMA_BUFFER_SAMPLES*2);
  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*) pwm_buffer, DMA_BUFFER_SIZE);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, ADC_SIZE);
  HAL_TIM_Base_Start(&htim2); // Start Timer2 (Trigger Source For ADC1)
  //HAL_ADC_Start_IT(&hadc1); // Start ADC Conversion

  //ADC DMA auto reads all analogue inputs
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  /* Example: Update duty cycle dynamically */
	      //for (int duty = 0; duty <= 2047; duty += 10)
	      //{
	        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);  // TIM1->CCR1 = duty;

	      //}


	  //update_frequency();
	  //update_phase_change();


	  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, sizeof(ADC_buffer));

	  printf("%u \r\n", ADC_buffer[0]);
	  printf("%u \r\n\n", ADC_buffer[1]);
	  printf("\r\n");
	  //printf("HELLO\r\n");
	  //uint8_t MSG[35] = "HELLO";
	  //HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
	  HAL_Delay(100);

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : lrclk_Pin */
  GPIO_InitStruct.Pin = lrclk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(lrclk_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : osc_Pin vol_Pin */
  GPIO_InitStruct.Pin = osc_Pin|vol_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
