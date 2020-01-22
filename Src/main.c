
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_dac.h"

#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "slcan/slcan.h"
#include "slcan/slcan_additional.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
//static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);


ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac;

static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);

uint8_t CANTXcomm[100] = {0};
static void CANRQST_GetADC1(void);
static void CANRQST_GetADC2(void);
static void CANRQST_SetDAC(uint8_t ch1, uint8_t ch2);

uint16_t adc_raw[6];

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

CanTxMsgTypeDef CanTxBuffer;
CanRxMsgTypeDef CanRxBuffer;

#define UART_RX_FIFO_SIZE    1
uint8_t Uart2RxFifo;


typedef struct tcanRxFlags {
	union {
		struct {
			uint8_t fifo1 :1;
			uint8_t fifo2 :1;
		};
		uint8_t byte;
	} flags;
	uint8_t activefifo;
} tcanRx;

volatile tcanRx canRxFlags;
void bootloaderSwitcher();
#define TYPE_ID 0x16
volatile int32_t serialNumber;
const uint32_t *uid = (uint32_t *)(UID_BASE + 4);
/* USER CODE END 0 */



static void getADC();

//HAL_DACEx_DualSetValue(&hdac,DAC_ALIGN_8B_R, 0x88, 0x88);

/*
if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0xBB) != HAL_OK)
{
  Error_Handler();
}

if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, 0x88) != HAL_OK)
{
  Error_Handler();
}
*/
/*
if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK)
{
  Error_Handler();
}

if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
{
  Error_Handler();
}
*/



uint8_t commandtest1[LINE_MAXLEN] = "C";
uint8_t commandtest2[LINE_MAXLEN] = "S"; //3
uint8_t commandtest3[LINE_MAXLEN] = "o";

uint16_t test = 0;

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	serialNumber = TYPE_ID | (((*uid) << 8) & 0xFFFFFF00);
	//bootloaderSwitcher();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN_Init();
//  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();

  //MX_DMA_Init();
  MX_ADC_Init();
  MX_DAC_Init();

  /* USER CODE BEGIN 2 */

	hcan.pTxMsg = &CanTxBuffer;
	hcan.pRxMsg = &CanRxBuffer;

	canRxFlags.flags.byte = 0;
	// CAN RX init
	{
		slcanClearAllFilters();

		HAL_NVIC_SetPriority(CEC_CAN_IRQn, 2, 2);
		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	}
	// UART RX
	{
//		HAL_UART_Receive_IT(&huart2, &Uart2RxFifo, UART_RX_FIFO_SIZE);
//		HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);
//		NVIC_EnableIRQ(USART2_IRQn);
	}
	/* Enable USART1 global interrupt */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	//HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_raw, 6);


	HAL_Delay(100);
	StartCan();

	/*
	HAL_Delay(500);
	slCanCheckCommand(&commandtest1[0]);
	HAL_Delay(500);
	slCanCheckCommand(&commandtest2[0]);
	HAL_Delay(500);
	slCanCheckCommand(&commandtest3[0]);
*/

	while (1) {

		//transmitStd("t00181122334455667788");
		getADC();

		  if (adc_raw[0] > 1500)
			  test = 1;
		  else
			  test = 2;



		slCanCheckCommand(command);
		slcanOutputFlush();
		if (canRxFlags.flags.byte != 0) {
			//slcanReciveCanFrame(hcan.pRxMsg);


			if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x10))
			{
				CANRQST_GetADC1();
			}

			if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x11))
			{
				CANRQST_GetADC2();
			}
			if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x12))
			{
				CANRQST_SetDAC(hcan.pRxMsg->Data[2], hcan.pRxMsg->Data[3]);
			}


			canRxFlags.flags.fifo1 = 0;
			HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
		}









  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_2TQ;
  hcan.Init.BS1 = CAN_BS1_11TQ;
  hcan.Init.BS2 = CAN_BS2_4TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = ENABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_MOD_GPIO_Port, CAN_MOD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_MOD_Pin */
  GPIO_InitStruct.Pin = CAN_MOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_MOD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
	canRxFlags.flags.fifo1 = 1;
//    HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
//	slCanProccesInput(Uart2RxFifo);
//	__HAL_UART_FLUSH_DRREGISTER(huart);
//	HAL_UART_Receive_IT(huart, &Uart2RxFifo, UART_RX_FIFO_SIZE);
}





/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};

	  hadc.Instance = ADC1;
	  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	  hadc.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc.Init.LowPowerAutoWait = DISABLE;
	  hadc.Init.LowPowerAutoPowerOff = DISABLE;
	  hadc.Init.ContinuousConvMode = DISABLE;
	  hadc.Init.DiscontinuousConvMode = ENABLE;
	  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc.Init.DMAContinuousRequests = DISABLE;
	  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  if (HAL_ADC_Init(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure for the selected ADC regular channel to be converted.
	  */
	  sConfig.Channel = ADC_CHANNEL_0;
	  //sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }


	  sConfig.Channel = ADC_CHANNEL_1;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }


	  sConfig.Channel = ADC_CHANNEL_2;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfig.Channel = ADC_CHANNEL_3;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfig.Channel = ADC_CHANNEL_6;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfig.Channel = ADC_CHANNEL_7;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */


static void MX_DAC_Init(void)
{

	  DAC_ChannelConfTypeDef sConfig = {0};

	  hdac.Instance = DAC;
	  if (HAL_DAC_Init(&hdac) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


/**
  * Enable DMA controller clock
  */
/*
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}
*/

static void getADC()
{
	  if (HAL_ADC_Start(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
	  {
	  	  Error_Handler();
	  }

	  adc_raw[0] = HAL_ADC_GetValue(&hadc);


	  if (HAL_ADC_Start(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
	  {
	  	  Error_Handler();
	  }

	  adc_raw[1] = HAL_ADC_GetValue(&hadc);

	  if (HAL_ADC_Start(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
	  {
	  	  Error_Handler();
	  }

	  adc_raw[2] = HAL_ADC_GetValue(&hadc);

	  if (HAL_ADC_Start(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
	  {
	  	  Error_Handler();
	  }

	  adc_raw[3] = HAL_ADC_GetValue(&hadc);

	  if (HAL_ADC_Start(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
	  {
	  	  Error_Handler();
	  }

	  adc_raw[4] = HAL_ADC_GetValue(&hadc);

	  if (HAL_ADC_Start(&hadc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
	  {
	  	  Error_Handler();
	  }

	  adc_raw[5] = HAL_ADC_GetValue(&hadc);


}






static void CANRQST_GetADC1(void)
{
	//CANTXcomm

	hcan.pTxMsg->StdId = 1;
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 8;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x10;
    hcan.pTxMsg->Data[2] = (adc_raw[0] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[3] = adc_raw[0] & 0x00FF;
    hcan.pTxMsg->Data[4] = (adc_raw[1] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[5] = adc_raw[1] & 0x00FF;
    hcan.pTxMsg->Data[6] = (adc_raw[2] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[7] = adc_raw[2] & 0x00FF;


	transmitStdMy("t0018ABCDEFAB56677888");


}



static void CANRQST_GetADC2(void)
{
	//CANTXcomm

	hcan.pTxMsg->StdId = 1;
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 8;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x11;
    hcan.pTxMsg->Data[2] = (adc_raw[3] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[3] = adc_raw[3] & 0x00FF;
    hcan.pTxMsg->Data[4] = (adc_raw[4] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[5] = adc_raw[4] & 0x00FF;
    hcan.pTxMsg->Data[6] = (adc_raw[5] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[7] = adc_raw[5] & 0x00FF;


	transmitStdMy("t0018ABCDEFAB56677888");


}

static void CANRQST_SetDAC(uint8_t ch1, uint8_t ch2)
{

	if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, ch1) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, ch2) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
	{
	  Error_Handler();
	}

	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK)
	{
	  Error_Handler();
	}

	hcan.pTxMsg->StdId = 1;
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 8;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x12;
    hcan.pTxMsg->Data[2] = 0x01;
    hcan.pTxMsg->Data[3] = ch1;
    hcan.pTxMsg->Data[4] = 0x02;
    hcan.pTxMsg->Data[5] = ch2;
    hcan.pTxMsg->Data[6] = 0x00;
    hcan.pTxMsg->Data[7] = 0x00;

	transmitStdMy("t0018ABCDEFAB56677888");




	//HAL_DACEx_DualSetValue(&hdac,DAC_ALIGN_8B_R, 0x88, 0x88);

}











/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
