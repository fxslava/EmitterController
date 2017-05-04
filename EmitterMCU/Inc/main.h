/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

// EEPROM, Temperature sensor IC
#define LASER_EEPROM_I2C_ADDRESS		0x50
#define LASER_TEMPSEN_I2C_ADDRESS		0x92
#define LASER_CNT_MEM_ADDRESS				0x0000

// CAN
#define CAN_DEVICE_ID								0x01			// Diode laser V1.0
#define SLOT_ID_FILTER							0 << 7		// Slot ID 0..1 << 7
#define SLOT_ID_MASK								1 << 7

// Device id groups
#define CAN_RECEIVER_DEVICE_ID_SLOT_mask			0x8000
#define CAN_RECEIVER_DEVICE_ID_PWM_mask				0x4000
#define CAN_RECEIVER_DEVICE_ID_PORT_mask			0x2000
#define CAN_RECEIVER_DEVICE_ID_SENS_mask			0x1000
#define CAN_RECEIVER_DEVICE_ID_GROUP_mask			0x7000

// CMD options
#define CAN_MESSAGE_TYPE_RW_mask							0x80 	// "0" - read, "1" - write
#define CAN_MESSAGE_TYPE_CMD_mask							0x40 	// "0" - register, "1" - command

// CMD types
#define CAN_MESSAGE_TYPE_CMD_RESETCNT					0x01
#define CAN_MESSAGE_TYPE_CMD_TAU							0x02
#define CAN_MESSAGE_TYPE_CMD_ENERGY						0x03
#define CAN_MESSAGE_TYPE_CMD_REQUESTID				0x04

// Registers
#define CAN_MESSAGE_TYPE_REGISTER_ID					0x01
#define CAN_MESSAGE_TYPE_REGISTER_CNT					0x02
#define CAN_MESSAGE_TYPE_REGISTER_TEMPERATURE	0x03
#define CAN_MESSAGE_TYPE_REGISTER_FLOW				0x04
#define CAN_MESSAGE_TYPE_REGISTER_LED					0x05
#define CAN_MESSAGE_TYPE_REGISTER_SWITCH			0x06

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

