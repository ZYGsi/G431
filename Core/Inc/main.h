/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "arm_math.h"  
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void fftCalculate(void);
int moveAverageFilter();

// 函数声明
double calculateFrequency(int index_);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_TEST_Pin GPIO_PIN_13
#define LED_TEST_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOB
#define KEY1_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */
#define FFT_SIZE 1024

void get_sin_tab( unsigned int point );
void SineWave_Data( uint16_t num,uint16_t *D,float U);
void TriangleWave_Data(uint16_t num, uint16_t *D, float U);
double corr1000_200(uint32_t* data, short* mask);

extern float mag_1harmo,mag_3harmo,k;//一次和三次谐波上的能量及其比值
extern float mag_5harmo,k_1;//五次谐波上的能量
extern unsigned short harmo3_index,harmo5_index;//输入信号三次和五次谐波频率对应的索引值
extern float mag_1harmo1,mag_3harmo1,k1;//一次和三次谐波上的能量及其比值
extern float mag_5harmo1;//五次谐波上的能量
extern unsigned short harmo3_index1,harmo5_index1;//输入信号三次和五次谐波频率对应的索引值

extern float32_t inputSignal[FFT_SIZE*2];// FFT 输入信号数组
extern float32_t fftOutput[FFT_SIZE/2];// FFT 输出数组

extern float32_t vpp;

extern int index_;// 存放 FFT 输出中最大值的索引
extern int index1_;// 存放 FFT 输出中最大值的索引

extern uint8_t  MODE ;
extern uint8_t  MODE1 ;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
