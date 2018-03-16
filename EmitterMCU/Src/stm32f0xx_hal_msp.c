/**
  ******************************************************************************
  * File Name          : stm32f0xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "main.h"

extern void Error_Handler(void);
/* USER CODE BEGIN 0 */
CanTxMsgTypeDef        TxMessage;
CanRxMsgTypeDef        RxMessage;
/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
	
volatile uint8_t laser_type = 0;
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */
	__HAL_RCC_PWR_CLK_ENABLE();
  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* SVC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVC_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_Delay(100);
	
	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))
		laser_type |= 0x01;
	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
		laser_type |= 0x02;
	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
		laser_type |= 0x04;
	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6))
		laser_type |= 0x08;
	
	if (laser_type == 0x01)
	{
		can_device_id = 0x81; // Nd Qsw V1
		can_slot_filter = 1 << 15;
	}	
	if (laser_type == 0x02)
	{
		can_device_id = 0x82; // Long Pulse
		can_slot_filter = 1 << 15;
	}
	if (laser_type == 0x04)
	{
		can_device_id = 0x01; // Diode Laser
		can_slot_filter = 0 << 15;
	}
	if (laser_type == 0x08)
	{
		can_device_id = 0x83; // 1440 nm (Fractional Laser)
		can_slot_filter = 1 << 15;
	}
	if (laser_type == 0x03)
	{
		can_device_id = 0x85; // IPL (SS | LP)
		can_slot_filter = 1 << 15;
	}
	if (laser_type == 0x06)
	{
		can_device_id = 0x86; // 1340 nm (LP | LD)
		can_slot_filter = 1 << 15;
	}
	if (laser_type == 0x09)
	{
		can_device_id = 0x84; // Nd Qsw V2 (FL | SS)
		can_slot_filter = 1 << 15;
	}
	if (laser_type == 0x0c)
	{
		can_device_id = 0x87; // 2940 nm (LD | FL)
		can_slot_filter = 1 << 15;
	}
  /* USER CODE END MspInit 1 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hcan->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(CEC_CAN_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */
		hcan->pTxMsg = &TxMessage;
		hcan->pRxMsg = &RxMessage;
		
		TxMessage.StdId = 0x00;
		TxMessage.ExtId = can_slot_filter;
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 2;
		TxMessage.Data[0] = 0x01;
		TxMessage.Data[1] = 0x00;
		
		CAN_FilterConfTypeDef canFilter;
		canFilter.FilterNumber = 0;
		canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
		canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
		canFilter.FilterIdHigh = can_slot_filter >> 13;
		canFilter.FilterIdLow = (can_slot_filter << 3) | CAN_ID_EXT;
		canFilter.FilterMaskIdHigh = can_slot_mask >> 13;
		canFilter.FilterMaskIdLow = (can_slot_mask << 3) | CAN_ID_EXT;
		canFilter.FilterFIFOAssignment = CAN_FIFO0;
		canFilter.FilterActivation = ENABLE;
		canFilter.BankNumber = 0;
		
		HAL_CAN_ConfigFilter(hcan, &canFilter);
		
		//HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
  /* USER CODE END CAN_MspInit 1 */
  }

}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

  if(hcan->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);

  }
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PA9     ------> I2C1_SCL
    PA10     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(I2C1_IRQn);

  }
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
