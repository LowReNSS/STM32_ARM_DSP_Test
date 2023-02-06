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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "iwdg.h"
#include "lptim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SIZE	256
#define FFT_SIZE	512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcData[ADC_SIZE];

arm_rfft_fast_instance_f32 rfft_fast_instance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void enter_sleep_mode(void);
void enter_ready_mode(void);
void adc_get_sample(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPTIM2_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  arm_rfft_fast_init_f32(&rfft_fast_instance, FFT_SIZE);

  float32_t inputSignal[ADC_SIZE];
  float32_t complexFFT[FFT_SIZE];
  float32_t powerFFT[FFT_SIZE / 2];

  uint32_t uartTxData[3];

  /*МК переводится в режим сна. МК будет выведен из режима сна внешним прерыванием:
  импульсом длительность ~500 мкс на выводе LPTIM2_ETR*/
  enter_sleep_mode();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // После пробуждение МК запускает цикл измерений
	  adc_get_sample();

	  // На основании полученных измерений строится спект сигнала

	  for(uint32_t i = 0; i < ADC_SIZE; i++)
	  {
		  inputSignal[i] =  adcData[i];
	  }

	  arm_rfft_fast_f32(&rfft_fast_instance, inputSignal, complexFFT, 0);
	  arm_cmplx_mag_f32(complexFFT, powerFFT, FFT_SIZE / 2);


	  // В полученном спектре вычисляется наибольшая гармоника и ее амплитуда
	  float32_t maxValue = 0;
	  uint32_t maxIndex = 0;
	  arm_max_f32(&powerFFT[1], FFT_SIZE / 2 - 1, &maxValue, &maxIndex);
	  maxIndex += 1;

	  uartTxData[0] = (uint32_t)maxValue;
	  uartTxData[1] = (uint32_t)maxIndex;
	  uartTxData[3]	= (uint32_t)HAL_CRC_Calculate(&hcrc, uartTxData, 2);

	  // Результат вычисления отсылается по проводному интерфейсу
	  HAL_UART_Transmit(&huart1, (uint8_t*)&uartTxData, sizeof(uartTxData) / sizeof(uartTxData[0]), 10);

	  // МК переводится в режим сна
	  enter_sleep_mode();

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(4000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	if(hlptim == &hlptim2)
	{
		HAL_IWDG_Refresh(&hiwdg);
		enter_ready_mode();
		HAL_LPTIM_TimeOut_Stop_IT(&hlptim2);
		HAL_PWR_DisableSleepOnExit();
	}
}

void HAL_ADCEx_EndOfSamplingCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1)
	{
		HAL_IWDG_Refresh(&hiwdg);
		HAL_PWR_DisableSleepOnExit();
	}
}


void enter_sleep_mode(void)
{
	LL_RCC_MSI_Enable();
	/* Wait till MSI is ready */
	while(LL_RCC_MSI_IsReady() != 1)
	{
		__NOP();
	}
	LL_RCC_MSI_EnableRangeSelection();
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_3);
	LL_RCC_MSI_SetCalibTrimming(0);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
	{
		__NOP();
	}
	LL_SetSystemCoreClock(800000);

	/* Update the time base */
	if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_LPTIM_TimeOut_Start_IT(&hlptim2, 0xFFFF - 1, 96 - 1);

	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}


void enter_ready_mode(void)
{
	LL_RCC_MSI_Enable();
	/* Wait till MSI is ready */
	while(LL_RCC_MSI_IsReady() != 1)
	{
		__NOP();
	}
	LL_RCC_MSI_EnableRangeSelection();
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_8);
	LL_RCC_MSI_SetCalibTrimming(0);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
	{
		__NOP();
	}
	LL_SetSystemCoreClock(16000000);

	/* Update the time base */
	if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
	{
		Error_Handler();
	}



}


void adc_get_sample(void)
{
	extern TIM_HandleTypeDef htim3;
	extern DMA_HandleTypeDef hdma_adc1;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, sizeof(adcData) / sizeof(adcData[0]));
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	HAL_SuspendTick();

	if(HAL_TIM_Base_Start(&htim3) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	while(hadc1.State != HAL_ADC_STATE_READY)
	{
		__NOP();
		__NOP();
	}

	HAL_TIM_Base_Stop(&htim3);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ResumeTick();
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
