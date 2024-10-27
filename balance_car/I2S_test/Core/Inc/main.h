/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mpu6050.h"
/* mpu6050 DMP�?  begin */
#include "eMPL_outputs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "log.h"
#include "mltypes.h"
#include "mpu.h"
#include "packet.h"

#include "encoder.h"
#include "control.h"

#include "stdlib.h"
#include "sccb.h"
#include "ov7670_set.h"
#include "dwt_delay.h"
#include "myDIP.h"
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern volatile uint8_t DCMI_OVER_FLAG;
extern volatile uint8_t	FLAG_MAIN_INITIAL;
extern volatile uint8_t FLAG_IMAGE_TRANSMITING;
extern volatile uint8_t FLAG_IMAGE_RGB;
extern volatile uint8_t	FLAG_IMAGE_BIN;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void get_tick_count (unsigned long *count);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define pic_length 200
#define pic_width 80

#define PI 3.141592653589793238
#define RED 0XF800
#define YELLOW 0XFFE0
#define GREEN 0X07E0
#define CYAN 0x07FF
#define BLUE 0X001F
#define PURPLE 0X780F
#define MAGENTA 0XF81F
#define BLACK 0x0000
#define WHITE 0XFFFF
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */