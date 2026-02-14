/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//-------------sbus program begin----------------

//-------------sbus program end------------------
//-------------sbus program begin----------------
#define SBUS_FRAME_SIZE 30
#define SBUS_HEADER     0x0F
#define SBUS_FOOTER     0x00
// SBUS���ݽṹ
typedef struct {
    uint8_t rx_buf[SBUS_FRAME_SIZE];    // ���ջ�����
    uint16_t channels[16];              // 16��ͨ��ֵ
    volatile uint8_t rx_len;            // �������ݳ���
    volatile uint8_t idle_flag;         // �����жϱ�־
    volatile uint8_t rx_index;          // ����λ������
    volatile uint8_t err_count;         // ��¼100���ź�����Ĵ������
} SBUS_HandleTypeDef;
extern SBUS_HandleTypeDef sbus;
//-------------sbus program end------------------
//-------------control program begin----------------
#define PI 3.1415926535f         // 2^31
typedef struct {
    float phi_max;
    float phi_min;
    uint16_t n;
    float phi_s;
    float phi_m;
} MotionParams_t;

// ״̬ö��
typedef enum {
    STATE_IDLE = 0,
	  STATE_HOMING,
    STATE_DOWNSTROKE,
    STATE_UPSTROKE,
    STATE_TRANSITION_UPSTROKE,  // �����ϳ��
    STATE_TRANSITION_DOWNSTROKE // �����³��
} SystemState_t;
//-------------control program end------------------


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//-------------control program begin------------------
// �ⲿ����ȫ�ֱ���
extern volatile MotionParams_t current_params;
extern volatile MotionParams_t new_params;
extern volatile MotionParams_t transition_params;
extern volatile SystemState_t system_state;
extern volatile uint16_t t_counter;
extern volatile uint8_t params_updated;
extern volatile uint8_t transition_active;
//-------------control program end------------------
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SV1_ADC_Pin GPIO_PIN_0
#define SV1_ADC_GPIO_Port GPIOA
#define SV2_ADC_Pin GPIO_PIN_1
#define SV2_ADC_GPIO_Port GPIOA
#define BLE_RX_Pin GPIO_PIN_2
#define BLE_RX_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_3
#define BLE_TX_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_4
#define IMU_INT_GPIO_Port GPIOA
#define IMU_INT_EXTI_IRQn EXTI4_IRQn
#define BAT_ADC_Pin GPIO_PIN_6
#define BAT_ADC_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define TX1_Pin GPIO_PIN_9
#define TX1_GPIO_Port GPIOA
#define SBUS_Pin GPIO_PIN_10
#define SBUS_GPIO_Port GPIOA
#define SV1_PWM_Pin GPIO_PIN_11
#define SV1_PWM_GPIO_Port GPIOA
#define SV2_PWM_Pin GPIO_PIN_12
#define SV2_PWM_GPIO_Port GPIOA
#define IMU_SCL_Pin GPIO_PIN_15
#define IMU_SCL_GPIO_Port GPIOA
#define BAT_IO_Pin GPIO_PIN_3
#define BAT_IO_GPIO_Port GPIOB
#define SV1_DIR_Pin GPIO_PIN_4
#define SV1_DIR_GPIO_Port GPIOB
#define SV2_DIR_Pin GPIO_PIN_5
#define SV2_DIR_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
