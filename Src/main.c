
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

/******************************************************************************/
/** Includes                                                                 **/
/******************************************************************************/

#include "main.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_dac.h"
#include "stm32f0xx_hal_spi.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "slcan/slcan.h"
#include "slcan/slcan_additional.h"

/******************************************************************************/
/** Version                                                                  **/
/******************************************************************************/
#define verH 0
#define verL 1
#define verD 6

#define CAN_ID 0x003

/******************************************************************************/
/** Local Variables                                                          **/
/******************************************************************************/
CAN_HandleTypeDef hcan;
IWDG_HandleTypeDef hiwdg;
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc;
DAC_HandleTypeDef hdac;
//DMA_HandleTypeDef hdma_adc;
SPI_HandleTypeDef hspi1;

/** CAN **/
CanTxMsgTypeDef CanTxBuffer;
CanRxMsgTypeDef CanRxBuffer;

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

/** ADC **/
#define ADC_CH_NR 8
uint16_t adc_raw[ADC_CH_NR];

/******************************************************************************/
/** Local Functions                                                          **/
/******************************************************************************/
/** clock configuration **/
void SystemClock_Config(void);

/** init functions **/
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
//static void MX_DMA_Init(void);

/** CAN communication **/
static void CANRQST_GetVer(void);
static void CANRQST_GetADC1(void);
static void CANRQST_GetADC2(void);
static void CANRQST_GetTemp(void);
static void CANRQST_SetDAC(uint16_t ch1, uint16_t ch2);
static void CANRQST_SetCtrlReg01(uint32_t ctrlReg);
static void CANRQST_ClrCtrlReg01(uint32_t ctrlReg);
static void CANRQST_GetCtrlReg01();
static void CANRQST_SetDigRes(uint8_t *data);

/** ADC **/
static void getADC();

/** UID **/
static void getUID(void);
uint32_t  UID[3];

/******************************************************************************/
/** Other Stuff                                                              **/
/******************************************************************************/
#define UART_RX_FIFO_SIZE    1
uint8_t Uart2RxFifo;
void bootloaderSwitcher();
#define TYPE_ID 0x16

/******************************************************************************/
/** CAN ctrl reg                                                             **/
/******************************************************************************/
typedef struct
{
    GPIO_TypeDef* bank;
    uint16_t      gpioNr;
} ctrlReg01_t;

#define CTRL_REG_01_LEN 4
static const ctrlReg01_t ctrlReg01_def[CTRL_REG_01_LEN] =
{
    { GPIOB,   GPIO_PIN_11 }, // EN_HV0
    { GPIOB,   GPIO_PIN_10 }, // EN_HV1
    { GPIOB,   GPIO_PIN_15 }, // EN_CAL_PULSE0
    { GPIOB,   GPIO_PIN_14 }, // EN_CAL_PULSE1
};

/******************************************************************************/
/** End Of Declarations                                                      **/
/******************************************************************************/


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/**
  * @brief  The application entry point.
  * @param  None
  * @return None
  */
int main(void)
{
    uint32_t tmp1;
    uint32_t tmp2;

    HAL_Init();

    SystemClock_Config();

    getUID();

    MX_GPIO_Init();
    MX_CAN_Init();
    //MX_USB_DEVICE_Init();
    MX_IWDG_Init();
    MX_ADC_Init();
    MX_DAC_Init();
    MX_SPI1_Init();

    hcan.pTxMsg = &CanTxBuffer;
    hcan.pRxMsg = &CanRxBuffer;

    canRxFlags.flags.byte = 0;

    HAL_NVIC_SetPriority(CEC_CAN_IRQn, 2, 2);
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

    HAL_Delay(100);
    StartCan();

    while (1)
    {

       getADC();
       HAL_Delay(200);
       HAL_Delay(500);

       //uint8_t spibuff[2];

       //spibuff[0] = 10;
       //spibuff[1] = 0;

       // test spi
       //HAL_Delay(500);
       //HAL_SPI_Transmit(&hspi1, spibuff, 1, 5000);
       //HAL_Delay(500);

       //slCanCheckCommand(command);
       //slcanOutputFlush();
       if (canRxFlags.flags.byte != 0)
       {

           if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x01))
           {
               CANRQST_GetVer();
           }

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
               tmp1 = 0;
               tmp1 = (hcan.pRxMsg->Data[2] << 8) | (hcan.pRxMsg->Data[3] << 0);

               tmp2 = 0;
               tmp2 = (hcan.pRxMsg->Data[4] << 8) | (hcan.pRxMsg->Data[5] << 0);

               CANRQST_SetDAC(tmp1, tmp2);
           }

           if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x13))
           {
               CANRQST_GetTemp();
           }

           if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x40))
           {
               tmp1 = 0;
               tmp1 = (hcan.pRxMsg->Data[2] << 24) | (hcan.pRxMsg->Data[3] << 16) | (hcan.pRxMsg->Data[4] << 8) | (hcan.pRxMsg->Data[5] << 0);
               CANRQST_SetCtrlReg01(tmp1);
           }

           if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x41))
           {
               tmp1 = 0;
               tmp1 = (hcan.pRxMsg->Data[2] << 24) | (hcan.pRxMsg->Data[3] << 16) | (hcan.pRxMsg->Data[4] << 8) | (hcan.pRxMsg->Data[5] << 0);
               CANRQST_ClrCtrlReg01(tmp1);
           }

           if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0x42))
           {
               CANRQST_GetCtrlReg01();
           }

           if ((hcan.pRxMsg->Data[0] == 0x00) && (hcan.pRxMsg->Data[1] == 0xA0))
           {
               uint8_t data[2];
               data[0] = hcan.pRxMsg->Data[3];
               data[1] = hcan.pRxMsg->Data[2];
               CANRQST_SetDigRes(data);
           }

           canRxFlags.flags.fifo1 = 0;
           HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
       }

       HAL_IWDG_Refresh(&hiwdg);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}

/******************************************************************************/
/** Local Conf & Init Functions                                              **/
/******************************************************************************/
/**
  * @brief  System Clock Configuration
  * @param  None
  * @return None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
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

    // Configure the Systick interrupt time
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    // Configure the Systick
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  CAN init function
  * @param  None
  * @return None
  */
static void MX_CAN_Init(void)
{

    CAN_FilterConfTypeDef  sFilterConfig;

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

    //slcanClearAllFilters();

    // Configure filters with mask
    /*
    sFilterConfig.BankNumber = 0;
    sFilterConfig.FilterNumber = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (0x01 << 5);
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = (0xFFFF << 5);
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    */

    // Configure filters with id list
    sFilterConfig.BankNumber = 0;
    sFilterConfig.FilterNumber = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (CAN_ID << 5);
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  IWDG init function
  * @param  None
  * @return None
  */
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

/**
  * @brief  GPIO init function
  * @param  None
  * @return None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(CAN_MOD_GPIO_Port, CAN_MOD_Pin, GPIO_PIN_RESET);

    // Configure GPIO pin: CAN_MOD_Pin
    GPIO_InitStruct.Pin = CAN_MOD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN_MOD_GPIO_Port, &GPIO_InitStruct);

    // Configure PIN B2: CAN_MOD
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    // Configure PIN B10: EN_HV1
    // Configure PIN B11: EN_HV0
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

    // Configure PIN B14: EN_CAL_PULSE1
    // Configure PIN B15: EN_CAL_PULSE0
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    // Configure SPI1



}

/**
  * @brief  ADC init function
  * @param  None
  * @return None
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

    // config for all channels
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    /** Configure for the selected ADC regular channel to be converted.
      * ADC_CHANNEL_0: DC_LEVEL_MEAS0
      * ADC_CHANNEL_1: DC_LEVEL_MEAS1
      * ADC_CHANNEL_2: U_SIPM_MEAS0
      * ADC_CHANNEL_3: U_SIPM_MEAS1
      * ADC_CHANNEL_6: I_SIPM_MEAS0
      * ADC_CHANNEL_7: I_SIPM_MEAS1
      * ADC_CHANNEL_8: TEMP_EXT
      * ADC_CHANNEL_9: TEMP_LOCAL
      */

    sConfig.Channel = ADC_CHANNEL_0;
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

    sConfig.Channel = ADC_CHANNEL_8;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_9;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  DAC init function
  * @param  None
  * @return None
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

static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_10BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
}

/******************************************************************************/
/** Other Functions                                                          **/
/******************************************************************************/

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
    canRxFlags.flags.fifo1 = 1;
}

static void getADC()
{
    for (int i = 0; i < ADC_CH_NR; i++)
    {
        if (HAL_ADC_Start(&hadc) != HAL_OK)
        {
            Error_Handler();
        }

        if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
        {
            Error_Handler();
        }

        adc_raw[i] = HAL_ADC_GetValue(&hadc);
    }
}

static void CANRQST_GetVer(void)
{
    hcan.pTxMsg->StdId = CAN_ID;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 8;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x01;
    hcan.pTxMsg->Data[2] = (((uint16_t) verH) & 0xFF00) >> 8;
    hcan.pTxMsg->Data[3] = ((uint16_t) verH) & 0x00FF;
    hcan.pTxMsg->Data[4] = (((uint16_t) verL) & 0xFF00) >> 8;
    hcan.pTxMsg->Data[5] = ((uint16_t) verL) & 0x00FF;
    hcan.pTxMsg->Data[6] = (((uint16_t) verD) & 0xFF00) >> 8;
    hcan.pTxMsg->Data[7] = ((uint16_t) verD) & 0x00FF;

    transmitStdMy("t0018ABCDEFAB56677888");

}

static void CANRQST_GetADC1(void)
{
    hcan.pTxMsg->StdId = CAN_ID;
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

    hcan.pTxMsg->StdId = CAN_ID;
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

static void CANRQST_GetTemp(void)
{
    hcan.pTxMsg->StdId = CAN_ID;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 6;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x13;
    hcan.pTxMsg->Data[2] = (adc_raw[6] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[3] = adc_raw[6] & 0x00FF;
    hcan.pTxMsg->Data[4] = (adc_raw[7] & 0xFF00) >> 8;
    hcan.pTxMsg->Data[5] = adc_raw[7] & 0x00FF;

    transmitStdMy("t0018ABCDEFAB56677888");
}

static void CANRQST_SetDAC(uint16_t ch1, uint16_t ch2)
{

    if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ch1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ch2) != HAL_OK)
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

    hcan.pTxMsg->StdId = CAN_ID;
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
}

static void CANRQST_SetCtrlReg01(uint32_t ctrlReg)
{
    for (int i = 0; i < CTRL_REG_01_LEN; i++)
    {
        if ((ctrlReg >> i) & 0x1)
        {
            HAL_GPIO_WritePin(ctrlReg01_def[i].bank, ctrlReg01_def[i].gpioNr, GPIO_PIN_SET);
        }
    }

    hcan.pTxMsg->StdId = CAN_ID;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 6;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x40;
    hcan.pTxMsg->Data[2] = (ctrlReg >> 24);
    hcan.pTxMsg->Data[3] = (ctrlReg >> 16);
    hcan.pTxMsg->Data[4] = (ctrlReg >>  8);
    hcan.pTxMsg->Data[5] = (ctrlReg >>  0);

    transmitStdMy("t0018ABCDEFAB56677888");

}

static void CANRQST_ClrCtrlReg01(uint32_t ctrlReg)
{
    for (int i = 0; i < CTRL_REG_01_LEN; i++)
    {
        if ((ctrlReg >> i) & 0x1)
        {
            HAL_GPIO_WritePin(ctrlReg01_def[i].bank, ctrlReg01_def[i].gpioNr, GPIO_PIN_RESET);
        }
    }

    hcan.pTxMsg->StdId = CAN_ID;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 6;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x41;
    hcan.pTxMsg->Data[2] = (ctrlReg >> 24);
    hcan.pTxMsg->Data[3] = (ctrlReg >> 16);
    hcan.pTxMsg->Data[4] = (ctrlReg >>  8);
    hcan.pTxMsg->Data[5] = (ctrlReg >>  0);

    transmitStdMy("t0018ABCDEFAB56677888");
}

static void CANRQST_GetCtrlReg01(void)
{
    // funkcja zadziala tylko w okreslonym przypadku:
    // https://community.st.com/s/question/0D50X00009XkfFTSAZ/the-hal-function-to-read-the-current-output-status-of-a-gpio
    //w przypadku wyjsc ustawionych jako OD nie zadziala !!!
    uint32_t tmp = 0;
    GPIO_PinState piState;

    for (int i = 0; i < CTRL_REG_01_LEN; i++)
    {
        piState = HAL_GPIO_ReadPin(ctrlReg01_def[i].bank, ctrlReg01_def[i].gpioNr);
        if (piState == GPIO_PIN_SET)
        {
            tmp |= (1 << i);
        }
    }

    hcan.pTxMsg->StdId = CAN_ID;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 6;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0x42;
    hcan.pTxMsg->Data[2] = (tmp >> 24);
    hcan.pTxMsg->Data[3] = (tmp >> 16);
    hcan.pTxMsg->Data[4] = (tmp >>  8);
    hcan.pTxMsg->Data[5] = (tmp >>  0);

    transmitStdMy("t0018ABCDEFAB56677888");

}

static void CANRQST_SetDigRes(uint8_t *data)
{
    HAL_SPI_Transmit(&hspi1, data, 1, 5000);

    hcan.pTxMsg->StdId = CAN_ID;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = 3;
    hcan.pTxMsg->Data[0] = 0x00;
    hcan.pTxMsg->Data[1] = 0xA0;
    hcan.pTxMsg->Data[2] = 0x01;

    transmitStdMy("t0018ABCDEFAB56677888");

}

static void getUID(void)
{
    UID[0] = HAL_GetUIDw0();
    UID[1] = HAL_GetUIDw1();
    UID[2] = HAL_GetUIDw2();
}

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
