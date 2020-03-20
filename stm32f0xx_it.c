/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint8_t rxByte[1];
uint8_t rxData[200];
uint8_t rxCounter;
uint8_t readyGSM;
uint8_t readyReception;
uint8_t OK;
uint8_t rxFlag;
uint8_t setup1Flag;
uint8_t setup2Flag;
uint8_t wakeUpFlag;
uint8_t queue;
unsigned long fullCounter = 0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern int getMsg;
extern int firstMsg;
extern uint8_t atCommand[];
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
int slashCounter = 0;
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	  HAL_UART_Receive(&huart1,(uint8_t *)rxByte,1,100);
	  HAL_UART_IRQHandler(&huart1);
	  rxData[rxCounter++] = rxByte[0];
	  fullCounter++;

if(rxByte[0] == '\n'){
	slashCounter++;
	if(slashCounter == 6 && queue == 0){ // call and sms ready
		  memset(rxData,'\0',sizeof(rxData));
		  wakeUpFlag = 1;
		  rxCounter = 0;
		  queue++;
		  slashCounter = 0;
		  rxByte[0] = '\0';
	}

if(slashCounter == 4 && queue == 1){ // signal quality

	  if(rxData[15] == '1' || rxData[15] == '2'){
		  memset(rxData,'\0',sizeof(rxData));
		  readyReception = 1;
		  rxCounter = 0;
		  queue++;
		  slashCounter = 0;
		  rxByte[0] = '\0';
	  }
	  else if(rxData[15] == '9'){
		  // not know or not detectable
		  // tekrar aç kapa yapılmalı!!!!!!!!
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
			HAL_Delay(400);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_Delay(400);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
			HAL_Delay(400);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_Delay(400);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
			HAL_Delay(400);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_Delay(400);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
			HAL_Delay(400);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_Delay(400);

			  /****///wake up m66
			  // This part required for wake up to m66 so here is we do that push down to PIN_8 //

			  	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
			    HAL_Delay(1500);
			    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
			    HAL_Delay(3000);

			  /*****/


			/****/
			    // in here we're send at command cause of this first at command reqiured to detect baundrate
				 HAL_UART_Transmit(&huart1,(uint8_t *)atCommand,strlen(atCommand),100);
				 HAL_Delay(2000);
			/****/
				 queue = 0;

	  }
	}

if(slashCounter == 2 && queue == 2){

	if(rxData[12] == 'O' && rxData[13] == 'K'){
		memset(rxData,'\0',sizeof(rxData));
		setup1Flag = 1;
		rxCounter = 0;
		queue++;
		slashCounter = 0;
		rxByte[0] = '\0';
	}
	else{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
	    HAL_Delay(200);
	    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
	    HAL_Delay(200);
	}
	}
if(slashCounter == 2 && queue == 3){

	if(rxData[16] == 'O' && rxData[17] == 'K'){
		memset(rxData,'\0',sizeof(rxData));
		setup2Flag = 1;
		rxCounter = 0;
		queue++;
		slashCounter = 0;
		rxByte[0] = '\0';
	}
	else{
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
		HAL_Delay(200);
	}
	}
if(slashCounter == 5  && queue == 4){

	// OK mesajı gelmiyorsa mesaj gitmemiş olabilir bunu kontrol ettirmedim cünkü
	// mesaj boyutu değişince index değişir!
	if(rxData[0] == 'A' && rxData[1] == 'T'){
			memset(rxData,'\0',sizeof(rxData));
			rxCounter = 0;
			queue = 1;
			getMsg = 1;
			firstMsg = 1;
			slashCounter = 0;
			rxByte[0] = '\0';
		}
		else{
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
			HAL_Delay(200);
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
			HAL_Delay(200);
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);
			HAL_Delay(200);
		}
}
}
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 and USART4 global interrupts.
  */

void USART3_4_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_4_IRQn 0 */
  /* USER CODE END USART3_4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN USART3_4_IRQn 1 */
  /*
  rxData[rxCounter++] = rxByte[0];
  if(rxCounter == 10){

	  if(rxData[5] == 'O'){
		  readyGSM = 1;
		  memset(rxData,'\0',sizeof(rxData));
	  }
	  queue++;
	  rxFlag = 1;
	  rxCounter = 0;
  }


  if(rxCounter == 27 && queue == 1){

  	  if(rxData[15] == '1' && rxData[23] == 'O'){
  		  readyReception = 1;
		  memset(rxData,'\0',sizeof(rxData));
  	  }
	  queue++;
  	  rxFlag = 1;
  	  rxCounter = 0;
    }

  if(rxCounter == 54 && queue == 2){

  	  if(rxData[50] == 'O'){
  		  OK = 1;
		  memset(rxData,'\0',sizeof(rxData));
		  queue=0;
  	  }
  	  rxFlag = 1;
  	  rxCounter = 0;
    }
*/
  /* USER CODE END USART3_4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
