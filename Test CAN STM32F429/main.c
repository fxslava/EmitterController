/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.4
  * @date    06-May-2016
  * @brief   Main program body
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

#ifdef _RTE_
#include "RTE_Components.h"             /* Component selection */
#endif
#ifdef RTE_CMSIS_RTOS                   // when RTE component CMSIS RTOS is used
#include "cmsis_os.h"                   // CMSIS RTOS header file
#endif

#include "Driver_CAN.h"
#include "Board_Buttons.h"
#include "Board_LED.h"

extern   ARM_DRIVER_CAN					Driver_CAN1;
extern   ARM_DRIVER_CAN					Driver_CAN2;

uint32_t                        rx_obj_idx1  = 0xFFFFFFFFU;
uint32_t                        rx_obj_idx2  = 0xFFFFFFFFU;
uint8_t                         rx_data[8];
ARM_CAN_MSG_INFO                rx_msg_info;
uint32_t                        tx_obj_idx1  = 0xFFFFFFFFU;
uint32_t                        tx_obj_idx2  = 0xFFFFFFFFU;
uint8_t                         tx_data[8];
ARM_CAN_MSG_INFO                tx_msg_info;
 
void CAN_SignalUnitEvent1 (uint32_t event) {}
 
void CAN_SignalObjectEvent1 (uint32_t obj_idx, uint32_t event) {
 
  if (obj_idx == rx_obj_idx1) {                  // If receive object event
    if (event == ARM_CAN_EVENT_RECEIVE) {       // If message was received successfully
      if (Driver_CAN1.MessageRead(rx_obj_idx1, &rx_msg_info, rx_data, 8U) > 0U) {
                                                // Read received message
        // process received message ...
				if (rx_data[0] == 0x01)
					LED_On(1);
				else
					LED_Off(1);
      }
    }
  }
  if (obj_idx == tx_obj_idx1) {                  // If transmit object event
    if (event == ARM_CAN_EVENT_SEND_COMPLETE) { // If message was sent successfully
      // acknowledge sent message ...
    }
  }
}

void CAN_SignalUnitEvent2 (uint32_t event) {}
 
void CAN_SignalObjectEvent2 (uint32_t obj_idx, uint32_t event) {
 
  if (obj_idx == rx_obj_idx2) {                  // If receive object event
    if (event == ARM_CAN_EVENT_RECEIVE) {       // If message was received successfully
      if (Driver_CAN1.MessageRead(rx_obj_idx2, &rx_msg_info, rx_data, 8U) > 0U) {
                                                // Read received message
        // process received message ...
				if (rx_data[0] == 0x01)
					LED_On(1);
				else
					LED_Off(1);
      }
    }
  }
  if (obj_idx == tx_obj_idx2) {                  // If transmit object event
    if (event == ARM_CAN_EVENT_SEND_COMPLETE) { // If message was sent successfully
      // acknowledge sent message ...
    }
  }
}

#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void) { 
  return os_time; 
}
#endif

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void Init_CAN1()
{
	ARM_CAN_CAPABILITIES     can_cap;
  ARM_CAN_OBJ_CAPABILITIES can_obj_cap;
  int32_t                  status;
  uint32_t                 i, num_objects;
 
  can_cap = Driver_CAN1.GetCapabilities (); // Get CAN driver capabilities
  num_objects = can_cap.num_objects;    // Number of receive/transmit objects
 
  status = Driver_CAN1.Initialize    (CAN_SignalUnitEvent1, CAN_SignalObjectEvent1);  // Initialize CAN driver
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN1.PowerControl  (ARM_POWER_FULL);                              // Power-up CAN controller
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN1.SetMode       (ARM_CAN_MODE_INITIALIZATION);                 // Activate initialization mode
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN1.SetBitrate    (ARM_CAN_BITRATE_NOMINAL,              // Set nominal bitrate
                                  100000U,                              // Set bitrate to 100 kbit/s
                                  ARM_CAN_BIT_PROP_SEG(5U)   |          // Set propagation segment to 5 time quanta
                                  ARM_CAN_BIT_PHASE_SEG1(1U) |          // Set phase segment 1 to 1 time quantum (sample point at 87.5% of bit time)
                                  ARM_CAN_BIT_PHASE_SEG2(1U) |          // Set phase segment 2 to 1 time quantum (total bit is 8 time quanta long)
                                  ARM_CAN_BIT_SJW(1U));                 // Resynchronization jump width is same as phase segment 2
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
	
	for (i = 0U; i < num_objects; i++) {                                          // Find first available object for receive and transmit
    can_obj_cap = Driver_CAN1.ObjectGetCapabilities (i);                            // Get object capabilities
    if      ((rx_obj_idx1 == 0xFFFFFFFFU) && (can_obj_cap.rx == 1U)) { rx_obj_idx1 = i; }
    else if ((tx_obj_idx1 == 0xFFFFFFFFU) && (can_obj_cap.tx == 1U)) { tx_obj_idx1 = i; break; }
  }
  if ((rx_obj_idx1 == 0xFFFFFFFFU) || (tx_obj_idx1 == 0xFFFFFFFFU)) { Error_Handler(); }
	
	// Set filter to receive messages with extended ID 0x12345678 to receive object
  status = Driver_CAN1.ObjectSetFilter(rx_obj_idx1, ARM_CAN_FILTER_ID_EXACT_ADD, ARM_CAN_EXTENDED_ID(0x12345678U), 0U);
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN1.ObjectConfigure(tx_obj_idx1, ARM_CAN_OBJ_TX);                 // Configure transmit object
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN1.ObjectConfigure(rx_obj_idx1, ARM_CAN_OBJ_RX);                 // Configure receive object
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
  
  status = Driver_CAN1.SetMode (ARM_CAN_MODE_NORMAL);                               // Activate normal operation mode
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
}

void Init_CAN2()
{
	ARM_CAN_CAPABILITIES     can_cap;
  ARM_CAN_OBJ_CAPABILITIES can_obj_cap;
  int32_t                  status;
  uint32_t                 i, num_objects;
 
  can_cap = Driver_CAN2.GetCapabilities (); // Get CAN driver capabilities
  num_objects = can_cap.num_objects;    // Number of receive/transmit objects
 
  status = Driver_CAN2.Initialize    (CAN_SignalUnitEvent2, CAN_SignalObjectEvent2);  // Initialize CAN driver
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN2.PowerControl  (ARM_POWER_FULL);                              // Power-up CAN controller
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN2.SetMode       (ARM_CAN_MODE_INITIALIZATION);                 // Activate initialization mode
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN2.SetBitrate    (ARM_CAN_BITRATE_NOMINAL,              // Set nominal bitrate
                                  100000U,                              // Set bitrate to 100 kbit/s
                                  ARM_CAN_BIT_PROP_SEG(5U)   |          // Set propagation segment to 5 time quanta
                                  ARM_CAN_BIT_PHASE_SEG1(1U) |          // Set phase segment 1 to 1 time quantum (sample point at 87.5% of bit time)
                                  ARM_CAN_BIT_PHASE_SEG2(1U) |          // Set phase segment 2 to 1 time quantum (total bit is 8 time quanta long)
                                  ARM_CAN_BIT_SJW(1U));                 // Resynchronization jump width is same as phase segment 2
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
	
	for (i = 0U; i < num_objects; i++) {                                          // Find first available object for receive and transmit
    can_obj_cap = Driver_CAN2.ObjectGetCapabilities (i);                            // Get object capabilities
    if      ((rx_obj_idx2 == 0xFFFFFFFFU) && (can_obj_cap.rx == 1U)) { rx_obj_idx2 = i; }
    else if ((tx_obj_idx2 == 0xFFFFFFFFU) && (can_obj_cap.tx == 1U)) { tx_obj_idx2 = i; break; }
  }
  if ((rx_obj_idx2 == 0xFFFFFFFFU) || (tx_obj_idx2 == 0xFFFFFFFFU)) { Error_Handler(); }
	
	// Set filter to receive messages with extended ID 0x12345678 to receive object
  status = Driver_CAN2.ObjectSetFilter(rx_obj_idx2, ARM_CAN_FILTER_ID_EXACT_ADD, ARM_CAN_EXTENDED_ID(0x12345678U), 0U);
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN2.ObjectConfigure(tx_obj_idx2, ARM_CAN_OBJ_TX);                 // Configure transmit object
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
 
  status = Driver_CAN2.ObjectConfigure(rx_obj_idx2, ARM_CAN_OBJ_RX);                 // Configure receive object
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
  
  status = Driver_CAN2.SetMode (ARM_CAN_MODE_NORMAL);                               // Activate normal operation mode
  if (status != ARM_DRIVER_OK ) { Error_Handler(); }
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	int32_t                  status;
  uint32_t                 i, num_objects;

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  osKernelInitialize();                 // initialize CMSIS-RTOS
#endif

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();
	Init_CAN1();
	Init_CAN2();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();


  /* Add your application code here
     */

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  // create 'thread' functions that start executing,
  // example: tid_name = osThreadCreate (osThread(name), NULL);

  osKernelStart();                      // start thread execution 
#endif

	memset(&tx_msg_info, 0U, sizeof(ARM_CAN_MSG_INFO));                           // Clear message info structure
  tx_msg_info.id = ARM_CAN_EXTENDED_ID(0x12345678U);                            // Set extended ID for transmit message
  tx_data[0]     = 0xFFU;                                                       // Initialize transmit data
  while (1) {
		if (Buttons_GetState() == 1)
		{
			tx_data[0] = 0x01;
			LED_On(0);
		}
		else
		{
			tx_data[0] = 0x00;
			LED_Off(0);
		}

    status = Driver_CAN2.MessageSend(tx_obj_idx1, &tx_msg_info, tx_data, 1U);
    if (status != 1U) { Error_Handler(); }
		HAL_Delay(10);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
