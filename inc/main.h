/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l5xx_hal.h"
#include "stm32l5xx_ll_ucpd.h"
#include "stm32l5xx_ll_bus.h"
#include "stm32l5xx_ll_cortex.h"
#include "stm32l5xx_ll_rcc.h"
#include "stm32l5xx_ll_system.h"
#include "stm32l5xx_ll_utils.h"
#include "stm32l5xx_ll_pwr.h"
#include "stm32l5xx_ll_gpio.h"
#include "stm32l5xx_ll_dma.h"
#include "stm32l5xx_ll_exti.h"
#include <complex.h>


////////////////////
// Macros
////////////////////

#define bitset(word,   idx)  ((word) |=  (1<<(idx)))
#define bitclear(word, idx)  ((word) &= ~(1<<(idx)))
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx)))
#define bitcheck(word, idx)  ((word) &   (1<<(idx)))


////////////////////
// Constants
////////////////////

#define CLK_FREQ 110000000		//Hz
#define FREQ_TOLERANCE 2		//Hz
#define SAMPLE_FREQ 4000.0	  	//Hz
#define NUM_SAMPLES 1024

typedef float complex cplx;

////////////////////
// Function Headers
////////////////////

void ADC1init(void);
void blueLEDclear(void);
void blueLEDinit(void);
void blueLEDset(void);
void blueLEDtoggle(void);
void clearBoardLEDs(void);
void delayMs(int ms);
void getSample(void);
void GPIOAinit(void);
void GPIOCinit(void);
void GPIOEinit(void);
void greenLEDclear(void);
void greenLEDinit(void);
void greenLEDset(void);
void greenLEDtoggle(void);
void OPAMP1init(void);
void redLEDset(void);
void redLEDinit(void);
void redLEDclear(void);
void setClks(void);
void setBoardLEDs(float fund_freq, float note);
void setLEDBar(const float tuning [], int num_strings, float note);
void TIM2init(void);


void SystemClock_Config(void);
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port GPIOC
#define UCPD_FLT_Pin GPIO_PIN_14
#define UCPD_FLT_GPIO_Port GPIOB
#define ST_LINK_VCP_TX_Pin GPIO_PIN_7
#define ST_LINK_VCP_TX_GPIO_Port GPIOG
#define ST_LINK_VCP_RX_Pin GPIO_PIN_8
#define ST_LINK_VCP_RX_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOA
#define UCPD_DBN_Pin GPIO_PIN_5
#define UCPD_DBN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
