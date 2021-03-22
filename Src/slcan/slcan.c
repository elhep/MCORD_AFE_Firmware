/*
 * slcan_interface.c
 *
 *  Created on: Apr 2, 2016
 *      Author: Vostro1440
 */

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "slcan.h"
#include "slcan_additional.h"

// internal slcan_interface state
static uint8_t state = STATE_CONFIG;

extern CAN_HandleTypeDef hcan;

void slcanClose()
{
	HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
	state = STATE_CONFIG;
}

uint8_t transmitStdMy(uint8_t* line) {
    HAL_StatusTypeDef tr;

    HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
    tr = HAL_CAN_Transmit(&hcan, 0);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
    return tr;
}

/**
 * @brief  get slcan state
 * @param  none
 * @retval slcan state
 */
/*
uint8_t slcan_getState()
{
	return state;
}
*/

void StartCan()
{

	HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
    state = STATE_CONFIG;


    HAL_Delay(500);

	if (state == STATE_CONFIG)
	slcanSetCANBaudRate(CAN_BR_100K);

	HAL_Delay(500);

	hcan.Init.Mode = CAN_MODE_NORMAL;

	if(CANInit() == HAL_OK)
	{
	HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
	state = STATE_OPEN;
	}

}






