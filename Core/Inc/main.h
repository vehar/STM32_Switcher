/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define SET(PORT, PIN) 		HAL_GPIO_WritePin(GPIO##PORT, GPIO_PIN_##PIN, GPIO_PIN_SET)
#define CLEAR(PORT, PIN) 	HAL_GPIO_WritePin(GPIO##PORT, GPIO_PIN_##PIN, GPIO_PIN_RESET)
#define TOGGLE(PORT, PIN) 	HAL_GPIO_TogglePin(GPIO##PORT, GPIO_PIN_##PIN)

#define A2_on				SET(A, 2)
#define A2_off				CLEAR(A, 2)

#define Button_pin	GPIOB, GPIO_PIN_2

#define led_off				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define led_on				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define led_Toggle			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)

#define PA1_power_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)	// ON/OFF supply voltage (28V) from PA
#define PA1_power_off		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)
#define PA1_Shutdown_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET) // logical shutdown
#define PA1_Shutdown_off	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)

#define PA2_power_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define PA2_power_off		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define PA2_Shutdown_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)
#define PA2_Shutdown_off	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)

#define PA3_power_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define PA3_power_off		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
#define PA3_Shutdown_on		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define PA3_Shutdown_off	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)

#define PA4_power_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)
#define PA4_power_off		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)
#define PA4_Shutdown_on		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define PA4_Shutdown_off	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)

#define PA5_power_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)
#define PA5_power_off		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)
#define PA5_Shutdown_on		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)
#define PA5_Shutdown_off	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)

#define PA6_power_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)
#define PA6_power_off		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)
#define PA6_Shutdown_on		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define PA6_Shutdown_off	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)

#define oct1_off			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define oct1_on				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)

#define oct2_off			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define oct2_on				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

#define oct3_off			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define oct3_on				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define supply_off			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define supplu_on			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define c28v_on				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define c28v_off			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)

#define TDD1_on				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define TDD1_off			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)

#define TDD2_on				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define TDD2_off			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)

#define TDD3_on				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define TDD3_off			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)

#define TDD4_on				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)
#define TDD4_off			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)

#define TDD5_on				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
#define TDD5_off			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)

#define TDD6_on				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)
#define TDD6_off			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)

#define fan_off				HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2)	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
#define fan_on				HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2)	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);


typedef enum 
{
	IDLE_CAPTURE = 0, 
	FIRST_CAPTURE,
	SECOND_CAPTURE
}CC_STATES_t;

typedef __packed struct 
{
	uint32_t CCVal;		
	uint32_t CCValPrev;		
	uint32_t Period;	
	uint32_t Cnt;	
	CC_STATES_t State_f;	
}CAPTURE_CHANNELS_t;

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
