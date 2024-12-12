/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f3xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

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
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2;
extern uint64_t contadorTimerModo1;
extern uint64_t timerEsperaModo1;
extern uint64_t tiempoEsperaModo1;
extern uint8_t flagModo1;
extern uint64_t ledModo1;
extern uint8_t jugando;
extern uint8_t flagEmpezado;
// MODO 2
extern uint8_t flagModo2;
extern uint64_t contadorTimerModo2;
extern uint64_t timerEsperaModo2;
extern uint64_t tiempoEsperaModo2;
extern uint16_t contadorBuzzer;
extern uint8_t flagBuzzer;
extern uint16_t tiempoBuzzer;
uint8_t buzzerActivado = 0;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
	if (flagModo1 == 1 && ledModo1 == 0) {
		flagModo1 = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		char cadena[50];
		if (flagEmpezado == 0){
			sprintf(cadena, "%d\n\r", -1);  // -1 significa que todavia el temporizador no ha empezado a contar
			timerEsperaModo1 = 0;
		}else{
			sprintf(cadena, "%d\n\r", contadorTimerModo1);
		}
		HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000);
		contadorTimerModo1 = 0;
		jugando = 0;
		flagEmpezado = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	}else if (flagModo1 == 1 && ledModo1 == 1) {
		flagModo1 = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		char cadena[50];
		if (flagEmpezado == 0){
			sprintf(cadena, "%d\n\r", -1);
			HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000); // -1 significa que todavia el temporizador no ha empezado a contar
			timerEsperaModo1 = 0;
		}else{
			sprintf(cadena, "%d\n\r", -2);
			HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000); // -2 significa que ha pulsado el pulsador incorrecto
			timerEsperaModo1 = 0;
		}
		contadorTimerModo1 = 0;
		jugando = 0;
		flagEmpezado = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
//	} else if (flagModo2 == 1 && buzzerActivado == 1) {
	} else if (flagModo2 == 1) {

		flagModo2 = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		char cadena[50];
		if (flagEmpezado == 0){
			sprintf(cadena, "%d\n\r", -1);  // -1 significa que todavia el temporizador no ha empezado a contar
			timerEsperaModo2 = 0;
		}else{
			sprintf(cadena, "%d\n\r", contadorTimerModo2);
		}
		HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000);
		contadorTimerModo2 = 0;
//		buzzerActivado = 0;
		jugando = 0;
		flagEmpezado = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	}
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if (flagModo1 == 1 && ledModo1 == 1) {
//		flagModo1 = 0;
//		HAL_TIM_Base_Stop_IT(&htim3);
//		char cadena[50];
//		sprintf(cadena, "%d\n\r", contadorTimerModo1);
//		HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000);
//		HAL_TIM_Base_Stop_IT(&htim2);
//		contadorTimerModo1 = 0;
//		jugando = 0;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);

		flagModo1 = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		char cadena[50];
		if (flagEmpezado == 0){
			sprintf(cadena, "%d\n\r", -1);
			timerEsperaModo1 = 0;
		}else{
			sprintf(cadena, "%d\n\r", contadorTimerModo1);
		}
		HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000);
		contadorTimerModo1 = 0;
		jugando = 0;
		flagEmpezado = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	}else if (flagModo1 == 1 && ledModo1 == 0) {
		flagModo1 = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		char cadena[50];
		if (flagEmpezado == 0){
			sprintf(cadena, "%d\n\r", -1);
			HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000); // -1 significa que todavia el temporizador no ha empezado a contar
			timerEsperaModo1 = 0;
		}else{
			sprintf(cadena, "%d\n\r", -2);
			HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000); // -2 significa que ha pulsado el pulsador incorrecto
			timerEsperaModo1 = 0;
		}
		contadorTimerModo1 = 0;
		jugando = 0;
		flagEmpezado = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	}else if (flagModo2 == 1) {
		flagModo2 = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		char cadena[50];
		sprintf(cadena, "%d\n\r", -3);
		HAL_UART_Transmit(&huart2, (uint8_t*) cadena, strlen(cadena), 1000); // -3 significa que ha pulsado el pulsador incorrecto para jugar al modo del buzzer
		contadorTimerModo2 = 0;
		timerEsperaModo2 = 0;
		buzzerActivado = 0;
		jugando = 0;
		flagEmpezado = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
	}
//	HAL_UART_Transmit_IT(&huart2, (uint8_t *) cadena, strlen(cadena));
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (flagModo1 == 1) {
		contadorTimerModo1++;
	} else if (flagModo2 == 1) {
		contadorTimerModo2++;
	}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if (flagModo1 == 1) {
		if (timerEsperaModo1 == tiempoEsperaModo1) {
			if (ledModo1 == 0) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
			} else {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
			}
			timerEsperaModo1 = 0;
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_Base_Start_IT(&htim2);
			flagEmpezado=1;
		} else {
			timerEsperaModo1++;
		}
	} else if (flagModo2 == 1) {
		if (timerEsperaModo2 == tiempoEsperaModo2) {
			timerEsperaModo2 = 0;
			HAL_TIM_Base_Stop_IT(&htim3);
			startBuzzer(392);
//			buzzerActivado = 1;
			HAL_TIM_Base_Start_IT(&htim2);
			flagEmpezado=1;
		} else {
			timerEsperaModo2++;
		}
	}
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global and DAC1 underrun error interrupts.
  */
void TIM6_DAC1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC1_IRQn 0 */
	if (flagBuzzer == 1) {
		contadorBuzzer++;
		if (contadorBuzzer == tiempoBuzzer) {
			endBuzzer();
			contadorBuzzer = 0;
			flagBuzzer = 0;
		}
	}
  /* USER CODE END TIM6_DAC1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC1_IRQn 1 */

  /* USER CODE END TIM6_DAC1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
