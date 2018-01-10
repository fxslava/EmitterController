/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
I2C_HandleTypeDef hi2c1;

// CAN messages
extern CanTxMsgTypeDef        TxMessage;
extern CanRxMsgTypeDef        RxMessage;
uint8_t												can_device_id = 0x01;
uint32_t											can_slot_filter = 0 << 15;
uint32_t											can_slot_mask = 1 << 15;

typedef struct UUID_struct
{
	uint16_t X;
	uint16_t Y;
	uint8_t WAF_NUM;
	char LOT_NUM[7];
} UUID;

#define STM32_UUID ((uint32_t *)0x1FFFF7AC)

// Main Laser Pulse Counter
volatile static uint32_t LaserPulseCounter = 0;
volatile static uint32_t newLaserPulseCounter = 0;

// Temperature
volatile float temperature;

//Duration table
volatile float duration_tbl[ENERGY_TABLE_DURATION_NUM] = {
	0.002f,	0.004f, 0.006f, 0.008f, 0.010f, 0.012f, 0.014f, 0.016f, 0.018f, 0.020f,
	2.000f,	4.000f, 6.000f, 8.000f, 10.00f, 12.00f, 14.00f, 16.00f, 18.00f, 20.00f,
	22.00f,	24.00f, 26.00f, 28.00f, 30.00f, 32.00f, 34.00f, 36.00f, 38.00f, 40.00f	}; // reserved

//Energy table
volatile float energy_tbl[ENERGY_TABLE_DURATION_NUM * ENERGY_TABLE_VOLTAGES_NUM] = {
	
/*							220V,	240V,	260V,	280V, 300V, 320V, 340V, 360V, 380V, 400V, 420V, 440V, 450V - Voltages	*/
/*	200us		*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	400us		*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	600us		*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	800us		*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	1000us	*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	1200us	*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	1400us	*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	1600us	*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	1800us	*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	2000us	*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,	
	
/*							220V,	240V,	260V,	280V, 300V, 320V, 340V, 360V, 380V, 400V, 420V, 440V, 450V - Voltages	*/	
/*	2ms			*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	4ms			*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	6ms			*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	8ms			*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	10ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	12ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	14ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	16ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	18ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	20ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,

/*							220V,	240V,	260V,	280V, 300V, 320V, 340V, 360V, 380V, 400V, 420V, 440V, 450V - Voltages	*/
/*	12ms		*/	0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	14ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	16ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	18ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	20ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	22ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	24ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	26ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	28ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f,
/*	30ms		*/  0.0f,	1.0f,	2.0f,	3.0f,	4.0f,	5.0f,	6.0f,	7.0f,	8.0f,	9.0f,	10.f,	11.f,	12.f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
float Lerp(float X1, float X2, float Y1, float Y2, float x)
{
	float Y = Y1;
	if ((X2 - X1) > 0.0001f)
	{
		Y = ((Y2 - Y1) / (X2 - X1)) * (x - X1) + Y1;
	}
	return Y;
}

void rx_led_on()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

void rx_led_off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void tx_led_on()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void tx_led_off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

void LoadCounterFromEEPROM()
{
	HAL_I2C_Mem_Read(&hi2c1, LASER_EEPROM_I2C_ADDRESS << 1, LASER_CNT_MEM_ADDRESS, sizeof(uint16_t), (uint8_t*)&LaserPulseCounter, sizeof(uint32_t), 10);
	newLaserPulseCounter = LaserPulseCounter;
}

void StoreCounterToEEPROM()
{
	HAL_I2C_Mem_Write(&hi2c1, LASER_EEPROM_I2C_ADDRESS << 1, LASER_CNT_MEM_ADDRESS, sizeof(uint16_t), (uint8_t*)&LaserPulseCounter, sizeof(uint32_t), 10);
	HAL_Delay(10);
}

bool TestCounter(uint32_t counter)
{
	uint32_t counter_cmp;
	HAL_I2C_Mem_Write(&hi2c1, LASER_EEPROM_I2C_ADDRESS << 1, LASER_CNT_MEM_ADDRESS, sizeof(uint16_t), (uint8_t*)&counter, sizeof(uint32_t), 10);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, LASER_EEPROM_I2C_ADDRESS << 1, LASER_CNT_MEM_ADDRESS, sizeof(uint16_t), (uint8_t*)&counter_cmp, sizeof(uint32_t), 10);
	return counter == counter_cmp;
}

void TestEEPROM()
{
	uint32_t counter = 0;
	for( counter = 0; counter < 100000; counter++)
	{
		if (!TestCounter(counter))
		{
			rx_led_on();
			tx_led_on();
			__breakpoint(0);
			HAL_Delay(1000);
			rx_led_off();
			tx_led_off();
		}
		else
		{
			rx_led_on();
			tx_led_on();
			HAL_Delay(1);
			rx_led_off();
			tx_led_off();
			HAL_Delay(10);
		}
	}
}

void InitTemperatureSensor()
{
	uint8_t data[4];
	// Write to configuration register (continous convertion)
	data[0] = 0x0E;
	HAL_I2C_Mem_Write(&hi2c1, LASER_TEMPSEN_I2C_ADDRESS, 0xAC, 1, data, sizeof(uint8_t), 10);
	// Write +40 degrees to TH
	data[0] = 0x28; data[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, LASER_TEMPSEN_I2C_ADDRESS, 0xA1, 1, data, sizeof(uint16_t), 10);
	// Write +10 degrees to TL
	data[0] = 0x0A; data[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, LASER_TEMPSEN_I2C_ADDRESS, 0xA2, 1, data, sizeof(uint16_t), 10);
	// Start convertion
	data[0] = 0x51;
	HAL_I2C_Master_Transmit(&hi2c1, LASER_TEMPSEN_I2C_ADDRESS, data, 1, 10);
}

void ReadTemperatureData()
{
	uint8_t data[4];
	HAL_I2C_Mem_Read(&hi2c1, 0x92, 0xAA, 1, data, 2, 10);
	temperature = (float)data[0] + (float)(data[1] >> 4) / 16.0f;
	HAL_Delay(50);
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
	// Load Laser Pulse Counter
	//TestEEPROM();
	LoadCounterFromEEPROM();
	
	// Normal start "Blink"
	rx_led_on();
	tx_led_on();
	HAL_Delay(100);
	rx_led_off();
	tx_led_off();
	
	// Start CAN receiving messages
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	
	// Test temperature sensor
	InitTemperatureSensor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
		ReadTemperatureData();
		
		if (LaserPulseCounter != newLaserPulseCounter)
		{
			LaserPulseCounter = newLaserPulseCounter;
			StoreCounterToEEPROM();
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
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
  hcan.Init.Prescaler = 120;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_13TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{
	uint32_t counter = 0;
  if (CanHandle->pRxMsg->IDE == CAN_ID_EXT)
  {
		// EMMITER
		if ((CanHandle->pRxMsg->ExtId & CAN_RECEIVER_DEVICE_ID_GROUP_mask) == 0)
		{
			if ((CanHandle->pRxMsg->ExtId & CAN_MESSAGE_TYPE_CMD_mask) == 0)
			{
				// Read from register
				if ((CanHandle->pRxMsg->ExtId & CAN_MESSAGE_TYPE_RW_mask) == 0)
				{
					switch ((uint8_t)(CanHandle->pRxMsg->ExtId & CAN_MESSAGE_TYPE_REGISTERID_mask))
					{
						case CAN_MESSAGE_TYPE_REGISTER_ID:
							TxMessage.Data[0] = can_device_id;
							TxMessage.DLC = 1;
							break;
						case CAN_MESSAGE_TYPE_REGISTER_UID1:
							memcpy((void*)TxMessage.Data, (void*)&STM32_UUID[0], 4);
							TxMessage.DLC = 4;
							break;
						case CAN_MESSAGE_TYPE_REGISTER_UID2:
							memcpy((void*)TxMessage.Data, (void*)&STM32_UUID[1], 4);
							TxMessage.DLC = 4;
							break;
						case CAN_MESSAGE_TYPE_REGISTER_UID3:
							memcpy((void*)TxMessage.Data, (void*)&STM32_UUID[2], 4);
							TxMessage.DLC = 4;
							break;
						case CAN_MESSAGE_TYPE_REGISTER_CNT:
							memcpy((void*)TxMessage.Data, (void*)&LaserPulseCounter, 4);
							TxMessage.DLC = 4;
							break;
						case CAN_MESSAGE_TYPE_REGISTER_TEMPERATURE:
							memcpy((void*)TxMessage.Data, (void*)&temperature, 4);
							TxMessage.DLC = 4;
							break;
						case CAN_MESSAGE_TYPE_REGISTER_FLOW:
							// Not support
							break;
						case CAN_MESSAGE_TYPE_REGISTER_LED:
							// Not support
							break;
						case CAN_MESSAGE_TYPE_REGISTER_SWITCH:
							// Not support
							break;
					}
					HAL_CAN_Transmit_IT(&hcan);
				}
				// Write to register
				else
				{
					switch ((uint8_t)(CanHandle->pRxMsg->ExtId & CAN_MESSAGE_TYPE_REGISTERID_mask))
					{
						case CAN_MESSAGE_TYPE_REGISTER_ID:
							// Read only
							break;
						case CAN_MESSAGE_TYPE_REGISTER_CNT:
							memcpy((void*)&newLaserPulseCounter, (void*)RxMessage.Data, 4);
							break;
						case CAN_MESSAGE_TYPE_REGISTER_TEMPERATURE:
							// Read only
							break;
						case CAN_MESSAGE_TYPE_REGISTER_FLOW:
							// Not support
							break;
						case CAN_MESSAGE_TYPE_REGISTER_LED:
							// Not support
							break;
						case CAN_MESSAGE_TYPE_REGISTER_SWITCH:
							// Not support
							break;
					}
				}
			}
			else
			{
				// Perform commands
				switch ((uint8_t)(CanHandle->pRxMsg->ExtId & CAN_MESSAGE_TYPE_COMMANDID_mask))
				{
					case CAN_MESSAGE_TYPE_CMD_RESETCNT:
						LaserPulseCounter = 0;
						StoreCounterToEEPROM();
						break;
					case CAN_MESSAGE_TYPE_CMD_TAU:
						// Not support
						break;
					case CAN_MESSAGE_TYPE_CMD_ENERGY:
						// Not support
						break;
					case CAN_MESSAGE_TYPE_CMD_REQUESTID:
						// Not support
						break;
				}
			}
		}
		
		// PORT
		if ((CanHandle->pRxMsg->ExtId & CAN_RECEIVER_DEVICE_ID_PORT_mask) != 0)
		{
			if ((CanHandle->pRxMsg->Data[1] & 0x01) != 0)
				rx_led_on();
			else
				rx_led_off();
			
			if ((CanHandle->pRxMsg->Data[1] & 0x02) != 0)
				tx_led_on();
			else
				tx_led_off();
		}
  }

  /* Receive */
  if (HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    //Error_Handler();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
