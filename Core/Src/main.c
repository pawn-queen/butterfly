/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>  // 用于比较标准数学函数
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//-------------control program begin---------------
// 全局变量定义
#define SERVO1_DIR 1.0f
#define SERVO2_DIR -1.0f
volatile MotionParams_t current_params;
volatile MotionParams_t new_params;
volatile MotionParams_t transition_params;
volatile SystemState_t system_state = STATE_IDLE;
volatile uint16_t t_counter = 0;
volatile uint8_t params_updated = 0;
volatile uint8_t transition_active = 0;
volatile uint32_t homing_delay_cnt = 0;
#define HOMING_DELAY_MS 200
volatile uint8_t servo_enable_flag = 0;//舵机使能标志位（上电默认关闭，CH5下拨后开启）
volatile uint8_t ch5_last_state = 0;  // 0=未下拨，1=已下拨
//-------------control program end------------------
//-------------sbus program begin----------------
SBUS_HandleTypeDef sbus = {0};
extern UART_HandleTypeDef huart1;  // 发送数据
extern UART_HandleTypeDef huart2;  // 接收数据
//-------------sbus program end------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//-------------control program begin---------------
// 函数声明
void CORDIC_Init(void);
void Range_to_2Pi(float *in, float *out);
float cordic_cos(float angle_rad);
float calculate_phi_value(uint16_t t, volatile MotionParams_t* params,float dir);
void update_derived_params(volatile MotionParams_t* params);
void Update_PWM_CCR(float phi_value);
void Motion_State_Machine_Handler(void);
void Update_Motion_Parameters(float phi_max, float phi_min, uint16_t n);
void Get_Current_Status(float* current_angle, uint16_t* current_t, SystemState_t* state, uint8_t* is_transition);
//-------------control program end------------------
void parse_sbus_frame(void);
uint16_t map_throttle_to_n(uint16_t throttle_value);
void process_sbus_throttle(void);
void process_ch5_switch(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CORDIC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  //-------------control program begin----------------
    // 初始化CORDIC
    CORDIC_Init();
    //初始化默认参数
    current_params.phi_max = 120.0f;//最大120度
    current_params.phi_min = 30.0f;  //最小30度
    current_params.n = 199;         // 200 means 周期约2s
    update_derived_params(&current_params);
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    //启动TIM4中断
    HAL_TIM_Base_Start_IT(&htim4);
		HAL_TIM_Base_Start_IT(&htim6);
  //-------------control program end------------------
  
//-------------sbus program begin----------------
    // Clear all relevant flags for UART and DMA
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_IDLE);
    __HAL_DMA_CLEAR_FLAG(huart1.hdmarx, DMA_FLAG_TC2);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);               // start usart1 idle
    HAL_UART_Receive_DMA(&huart1, sbus.rx_buf, SBUS_FRAME_SIZE);// start usart1 dma receive
//-------------sbus program end------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
            if ( sbus.idle_flag ) {
          sbus.idle_flag  = 0;
          sbus.rx_index++;
          parse_sbus_frame();          
//          printf("Received: %u bytes :", sbus.rx_len);
//          for (int i = 0; i < sbus.rx_len; i++) {
//              printf("%02X ", sbus.rx_buf[i]);
//          }
//          printf("\n\r");
          HAL_UART_Receive_DMA(&huart1, sbus.rx_buf, SBUS_FRAME_SIZE);
      }
      // 新增：先检测CH5状态，判断是否使能舵机
    process_ch5_switch();      
    // 仅当舵机使能后，才处理SBUS调速和状态机
    if (servo_enable_flag == 1) {
			//处理SBUS油门控制
        process_sbus_throttle();
		}
		HAL_Delay(1);	
//         // 可选：发送调试信息到串口
//        static uint32_t last_debug_time = 0;
//        if (HAL_GetTick() - last_debug_time > 1000) {  // ÿ���ӡһ��
//            printf("n: %d, throttle: %d\r\n", 
//                   current_params.n, sbus.channels[2]);
//            last_debug_time = HAL_GetTick();
//        }
//        
//        HAL_Delay(1);  // 适当延时

    //-------------control program begin----------------
//      
//              //  测试2：在运行过程中更新参数
//        HAL_Delay(10000); // 运行10秒后更新参数
//        Update_Motion_Parameters(180.0f, 90.0f, 199); // 2s����
////        printf("10s done update param...\r\n");
//      
//        HAL_Delay(10000); //  再运行1s后更新
//        Update_Motion_Parameters(150.0f, 30.0f, 99); //  1s����
////        printf("5s done update param again...\r\n");

    //-------------control program end------------------
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */	
  /* USER CODE END 3 */
}
	}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//-------------control program begin----------------
// 新增：TIM6中断回调函数（1ms触发一次）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        // 仅当舵机使能后，才执行状态机
        if (servo_enable_flag == 1) {
            Motion_State_Machine_Handler();
        } else {
            // 未使能时，保持舵机初始位置（防止扑动）
            float phi_servo_zero1 = 0.0f; 
            float phi_servo_zero2 = 140.0f;
            uint32_t ccr1_zero = (uint32_t)((phi_servo_zero1 * 2000.0f) / 180.0f + 500.0f);
            uint32_t ccr2_zero = (uint32_t)((phi_servo_zero2 * 2000.0f) / 180.0f + 500.0f);
            ccr1_zero = (ccr1_zero > 2500) ? 2500 : (ccr1_zero < 500) ? 500 : ccr1_zero;
            ccr2_zero = (ccr2_zero > 2500) ? 2500 : (ccr2_zero < 500) ? 500 : ccr2_zero;
            TIM4->CCR1 = ccr1_zero;
            TIM4->CCR2 = ccr2_zero;
        }
    }
}
// CORDIC初始化
void CORDIC_Init(void) {
    // 配置CORDIC为单输入余弦模式，6周期24圈精度
    CORDIC->CSR = CORDIC_FUNCTION_COSINE | CORDIC_PRECISION_6CYCLES;
//    // 巨简洁模式 
//    CORDIC->CSR = 0x00180060; //32bit、双in双out，模量为1、24cycles
}
void Range_to_2Pi(float *in, float *out) {// 角度范围限制函数
    float tempf = *in;
    if(tempf > PI) {
        tempf = fmodf(tempf, 2*PI);
        if(tempf > PI) tempf -= 2*PI;
    } 
    else if(tempf < -PI) {
        tempf = fmodf(tempf, 2*PI);
        if(tempf < -PI) tempf += 2*PI;
    }
    *out = tempf;
}
// CORDIC余弦计算
float cordic_cos(float angle_rad) {
    // 将角度转换为Q1.31格式
    int32_t angle_q31 = (int32_t)(angle_rad * 683565275.58f);
    // 写入角度值
    CORDIC->WDATA = angle_q31;
    // 等待计算完成
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0);
    // 读取结果并转换为浮点数
    int32_t result = CORDIC->RDATA;
    return (float)result / 2147483648.0f;
    
//        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
//        printf("CSR = 0x%08lX\n", (unsigned long)CORDIC->CSR);
	//        // 将角度转换为Q1.31格式
//        int32_t angle_q31 = (int32_t)(angle_rad * 683565275.58f);
//        CORDIC->WDATA = angle_q31;
//        CORDIC->WDATA = 0x7FFFFFFF;   // ģֵ=1.0     
//        while(!(CORDIC->CSR & CORDIC_CSR_RRDY)){ __NOP();}
//        int32_t cos_value = CORDIC->RDATA; // cos
//        int32_t sin_value = CORDIC->RDATA; // sin
//        return (float)cos_value / 2147483648.0f;

}

// 更新中间参数计算
void update_derived_params(volatile MotionParams_t* params) {
    params->phi_s = (params->phi_max - params->phi_min) / 2.0f;
    params->phi_m = (params->phi_max + params->phi_min) / 2.0f;
}
// 计算运动曲线角度计算
float calculate_phi_value(uint16_t t, volatile MotionParams_t* params, float dir) {
    // 计算角度参数：2π*(t/(2n+2))
    if (t == 0)
    {
      if (dir == SERVO1_DIR) return 20.0f;      // 舵机1归位90°
      if (dir == SERVO2_DIR) return 0.0f;     // 舵机2归位0°
    }
	  float angle = 2.0f * 3.1415926535f * ((float)t / (2.0f * params->n + 2.0f));
    float limited_angle;
    Range_to_2Pi(&angle, &limited_angle);
    // 使用CORDIC计算余弦
    float cos_val = cordic_cos(limited_angle);
//    printf("cos_vals=%.1f\r\n", cos_val);
//    printf("UP: phi_s=%.1f, phi_m=%.1f\r\n", params->phi_s, params->phi_m);
    // 计算最终角度值
    return params->phi_s * (dir*cos_val) + params->phi_m;
}
//更新PWM占空比

/*void Update_PWM_CCR(float phi_value) {
    // 将角度值线性映射到CCR值 (500-2500)
    volatile MotionParams_t* params = &current_params;
//    uint32_t ccr_value = (uint32_t)((phi_value - params->phi_min) * 
//                                   4999.0f / (params->phi_max - params->phi_min));
    uint32_t ccr_value = (uint32_t)((phi_value*2000)/180.0f+500);
    
     // 限制CCR值在有效范围内
    if (ccr_value > 2500) ccr_value = 2500;
    if (ccr_value < 500) ccr_value = 500;
    
    // 更新两个通道的CCR值
    TIM4->CCR1 = ccr_value;
    TIM4->CCR2 = ccr_value;
}*/

// // 更新PWM占空比(双通道反向)
void Update_PWM_CCR(float phi_value) {
    volatile MotionParams_t* params = &current_params;
    // 正向角度：传入的phi_value
    float phi_forward = phi_value;
    // 反向角度：基于正向角度取反（保持反向逻辑）
    float phi_reverse = params->phi_m * 2 - phi_value; // 等价于phi_m - (phi_forward - phi_m)
    
     // 角度→CCR映射（原逻辑不变）
    uint32_t ccr1_value = (uint32_t)((phi_forward * 2000.0f) / 180.0f + 500.0f);
    uint32_t ccr2_value = (uint32_t)((phi_reverse * 2000.0f) / 180.0f + 500.0f);
    
    //限幅保护
    ccr1_value = (ccr1_value > 2500) ? 2500 : (ccr1_value < 500) ? 500 : ccr1_value;
    ccr2_value = (ccr2_value > 2500) ? 2500 : (ccr2_value < 500) ? 500 : ccr2_value;
    
    //更新PWM
    TIM4->CCR1 = ccr1_value;
    TIM4->CCR2 = ccr2_value;
}
// 用户参数更新接口(优化后)
void Update_Motion_Parameters(float phi_max, float phi_min, uint16_t n) {
//    printf("param update request: max=%.1f, min=%.1f, n=%d\r\n", phi_max, phi_min, n);
    
    // 保存完整新参数
    new_params.phi_max = phi_max;
    new_params.phi_min = phi_min;
    new_params.n = n;
    update_derived_params(&new_params);
    
    // 根据当前状态创建相应的过渡参数
    if (system_state == STATE_DOWNSTROKE) {
       // 在downstroke期间更新：过渡参数使用原phi_min + 新phi_max和n
//        printf("DOWNSTROKE--,creat trans params\r\n");
        printf("1\r\n");
        transition_params.phi_max = phi_max;                    //新最大值
        transition_params.phi_min = current_params.phi_min;     // 原最小值（保持连续）
        transition_params.n = n;                                // 新n值
        update_derived_params(&transition_params);
        transition_active = 1;
        printf("trans param: max=%.1f, min=%.1f, n=%d\r\n", 
               transition_params.phi_max, transition_params.phi_min, transition_params.n);
        
    } 
    else if (system_state == STATE_UPSTROKE) {
         // 在upstroke期间更新：过渡参数使用原phi_max + 新phi_min和新n值
//        printf("UPSTROKE--,creat trans params\r\n");
        printf("2\r\n");
        transition_params.phi_max = current_params.phi_max;     // 原最大值（保持连续）
        transition_params.phi_min = phi_min;                    // 新最小值
        transition_params.n = n;                                // 新n值ֵ
        update_derived_params(&transition_params);
        transition_active = 1;
        printf("trans params: max=%.1f, min=%.1f, n=%d\r\n", 
               transition_params.phi_max, transition_params.phi_min, transition_params.n);
        
    } 
    else if (system_state == STATE_IDLE) {
         // 空闲状态：直接应用完整参数
//        printf("IDLE--,aplly new params\r\n");
//        printf("3\r\n");
        current_params = new_params;
        transition_active = 0;
    } 
    else if (system_state == STATE_TRANSITION_UPSTROKE) {
        // 已经在过渡状态中：直接更新new_params，当前过渡继续执行
//        printf("TRANSITION--,wait to update\r\n");
//        printf("4\r\n");
    }
    params_updated = 1;
}
//  状态机处理函数（改造后）
void Motion_State_Machine_Handler(void) {
    float phi_servo1, phi_servo2; // 舵机1/2的目标角度
       switch(system_state){ 
	     case STATE_IDLE:
				  if (servo_enable_flag == 1) {
					system_state = STATE_HOMING;
          homing_delay_cnt = 0;
		 }
            break;
		    case STATE_HOMING:			
		        // 1. 定义舵机0度目标值，复用原有角度→CCR的映射逻辑
          float phi_servo_zero1 = 20.0f; 
				  float phi_servo_zero2 = 0.0f;		//舵机目标归位角度
          uint32_t ccr1_zero = (uint32_t)((phi_servo_zero1 * 2000.0f) / 180.0f + 500.0f);
          uint32_t ccr2_zero = (uint32_t)((phi_servo_zero2 * 2000.0f) / 180.0f + 500.0f);
    
          // 2. 限幅保护（和其他状态保持一致，防止异常值）
          ccr1_zero = (ccr1_zero > 2500) ? 2500 : (ccr1_zero < 500) ? 500 : ccr1_zero;
          ccr2_zero = (ccr2_zero > 2500) ? 2500 : (ccr2_zero < 500) ? 500 : ccr2_zero;
     
          // 3. 更新PWM寄存器，让双舵机归位到0度
          TIM4->CCR1 = ccr1_zero;
          TIM4->CCR2 = ccr2_zero;
				// 第二步：延时计数，等待舵机到位
            homing_delay_cnt++;
            if (homing_delay_cnt >= HOMING_DELAY_MS) {
                // 归位完成：切换到下冲程，清零步数计数器
                system_state = STATE_DOWNSTROKE;
                t_counter = 0;
                homing_delay_cnt = 0;   // 清零延时计数器，方便下次归位
            }
            break;
                 
        case STATE_DOWNSTROKE:
            if (t_counter <= current_params.n) {
                 // 1. 计算两个舵机的角度（方向系数分别为SERVO1_DIR/SERVO2_DIR）
                phi_servo1 = calculate_phi_value(t_counter, &current_params, SERVO1_DIR);
                phi_servo2 = calculate_phi_value(t_counter, &current_params, SERVO2_DIR);
                
                // 2. 分别计算两个舵机的CCR值（复用原有PWM映射逻辑）
                uint32_t ccr1 = (uint32_t)((phi_servo1 * 2000.0f) / 180.0f + 500.0f);
                uint32_t ccr2 = (uint32_t)((phi_servo2 * 2000.0f) / 180.0f + 500.0f);
                
                // 3. 限幅保护
                ccr1 = (ccr1 > 2500) ? 2500 : (ccr1 < 500) ? 500 : ccr1;
                ccr2 = (ccr2 > 2500) ? 2500 : (ccr2 < 500) ? 500 : ccr2;
                
                // 4. 更新PWM通道
                TIM4->CCR1 = ccr1;
                TIM4->CCR2 = ccr2;
                
                t_counter++;
                  // 原有状态切换逻辑（完全不变）
                if (t_counter > current_params.n) {
                    if (transition_active) {   
                        system_state = STATE_TRANSITION_UPSTROKE;
                        current_params = transition_params;
                    } 
                    else if (params_updated) {  
                        current_params = new_params;
                        params_updated = 0;
                        system_state = STATE_UPSTROKE;
                    } 
                    else {  
                        system_state = STATE_UPSTROKE;
                    }
                    transition_active = 0;
                    t_counter = current_params.n+1;
                }
            }
            break;
            
        case STATE_UPSTROKE:
            if (t_counter <= 2*current_params.n+1) {
                 // 同上：计算双舵机角度→CCR→更新PWM
                phi_servo1 = calculate_phi_value(t_counter, &current_params, SERVO1_DIR);
                phi_servo2 = calculate_phi_value(t_counter, &current_params, SERVO2_DIR);
                
                uint32_t ccr1 = (uint32_t)((phi_servo1 * 2000.0f) / 180.0f + 500.0f);
                uint32_t ccr2 = (uint32_t)((phi_servo2* 2000.0f) / 180.0f + 500.0f);
                
                ccr1 = (ccr1 > 2500) ? 2500 : (ccr1 < 500) ? 500 : ccr1;
                ccr2 = (ccr2 > 2500) ? 2500 : (ccr2 < 500) ? 500 : ccr2;
                
                TIM4->CCR1 = ccr1;
                TIM4->CCR2 = ccr2;
                
                t_counter++;
                 // 原有状态切换逻辑（完全不变）
                if (t_counter > 2*current_params.n+1) {
                    if (transition_active) {
                        system_state = STATE_TRANSITION_DOWNSTROKE;
                        current_params = transition_params;
                    } else if (params_updated) {
                        current_params = new_params;
                        params_updated = 0;
                        system_state = STATE_DOWNSTROKE;
                    } else {
                        system_state = STATE_DOWNSTROKE;
                    }
                    t_counter = 0;
                    transition_active = 0;
                }
            }
            break;
            
         // 过渡状态（TRANSITION_DOWNSTROKE/TRANSITION_UPSTROKE）逻辑完全复用
        // 仅将角度计算改为双舵机方向版
        case STATE_TRANSITION_DOWNSTROKE:
            if (t_counter <= current_params.n) {
                phi_servo1 = calculate_phi_value(t_counter, &current_params, SERVO1_DIR);
                phi_servo2 = calculate_phi_value(t_counter, &current_params, SERVO2_DIR);
                
                uint32_t ccr1 = (uint32_t)((phi_servo1* 2000.0f) / 180.0f + 500.0f);
                uint32_t ccr2 = (uint32_t)((phi_servo2* 2000.0f) / 180.0f + 500.0f);
                
                ccr1 = (ccr1 > 2500) ? 2500 : (ccr1 < 500) ? 500 : ccr1;
                ccr2 = (ccr2 > 2500) ? 2500 : (ccr2 < 500) ? 500 : ccr2;
                
                TIM4->CCR1 = ccr1;
                TIM4->CCR2 = ccr2;
                
                HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
                t_counter++;
                
                if (t_counter > current_params.n) {
                    current_params = new_params;
                    system_state = STATE_UPSTROKE;
                    t_counter = current_params.n+1;
                    transition_active = 0;
                    params_updated = 0;
                }
            }
            break;
            
        case STATE_TRANSITION_UPSTROKE:
            if (t_counter <= 2*current_params.n+1) {
                phi_servo1 = calculate_phi_value(t_counter, &current_params, SERVO1_DIR);
                phi_servo2 = calculate_phi_value(t_counter, &current_params, SERVO2_DIR);
                
                uint32_t ccr1 = (uint32_t)((phi_servo1 * 2000.0f) / 180.0f + 500.0f);
                uint32_t ccr2 = (uint32_t)((phi_servo2 * 2000.0f) / 180.0f + 500.0f);
                
                ccr1 = (ccr1 > 2500) ? 2500 : (ccr1 < 500) ? 500 : ccr1;
                ccr2 = (ccr2 > 2500) ? 2500 : (ccr2 < 500) ? 500 : ccr2;
                
                TIM4->CCR1 = ccr1;
                TIM4->CCR2 = ccr2;
                
                HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
                t_counter++;
                
                if (t_counter > 2*current_params.n+1) {
                    current_params = new_params;
                    system_state = STATE_DOWNSTROKE;
                    t_counter = 0;
                    transition_active = 0;
                    params_updated = 0;
                }
            }
            break;
    }
}
//
//-------------control program end------------------

//-------------sbus program begin----------------
void parse_sbus_frame(void){
    if (sbus.rx_buf[0] == SBUS_HEADER && sbus.rx_buf[24] == SBUS_FOOTER) {

//  if (sbus.rx_len == 25) {
      sbus.channels[0] = ((int16_t)sbus.rx_buf[1] >> 0 | ((int16_t)sbus.rx_buf[2] << 8 )) & 0x07FF;
      sbus.channels[1] = ((int16_t)sbus.rx_buf[2] >> 3 | ((int16_t)sbus.rx_buf[3] << 5 )) & 0x07FF;
      sbus.channels[2] = ((int16_t)sbus.rx_buf[3] >> 6 | ((int16_t)sbus.rx_buf[4] << 2 ) | (int16_t)sbus.rx_buf[5] << 10 ) & 0x07FF;
      sbus.channels[3] = ((int16_t)sbus.rx_buf[5] >> 1 | ((int16_t)sbus.rx_buf[6] << 7 )) & 0x07FF;
      sbus.channels[4] = ((int16_t)sbus.rx_buf[6] >> 4 | ((int16_t)sbus.rx_buf[7] << 4 )) & 0x07FF;
      sbus.channels[5] = ((int16_t)sbus.rx_buf[7] >> 7 | ((int16_t)sbus.rx_buf[8] << 1 ) | (int16_t)sbus.rx_buf[9] << 9 ) & 0x07FF;
      sbus.channels[6] = ((int16_t)sbus.rx_buf[9] >> 2 | ((int16_t)sbus.rx_buf[10] << 6 )) & 0x07FF;
      sbus.channels[7] = ((int16_t)sbus.rx_buf[10] >> 5 | ((int16_t)sbus.rx_buf[11] << 3 )) & 0x07FF;
      sbus.channels[8] = ((int16_t)sbus.rx_buf[12] << 0 | ((int16_t)sbus.rx_buf[13] << 8 )) & 0x07FF;
      sbus.channels[9] = ((int16_t)sbus.rx_buf[13] >> 3 | ((int16_t)sbus.rx_buf[14] << 5 )) & 0x07FF;
      sbus.channels[10] = ((int16_t)sbus.rx_buf[14] >> 6 | ((int16_t)sbus.rx_buf[15] << 2 ) | (int16_t)sbus.rx_buf[16] << 10 ) & 0x07FF;
      sbus.channels[11] = ((int16_t)sbus.rx_buf[16] >> 1 | ((int16_t)sbus.rx_buf[17] << 7 )) & 0x07FF;
      sbus.channels[12] = ((int16_t)sbus.rx_buf[17] >> 4 | ((int16_t)sbus.rx_buf[18] << 4 )) & 0x07FF;
      sbus.channels[13] = ((int16_t)sbus.rx_buf[18] >> 7 | ((int16_t)sbus.rx_buf[19] << 1 ) | (int16_t)sbus.rx_buf[20] << 9 ) & 0x07FF;
      sbus.channels[14] = ((int16_t)sbus.rx_buf[20] >> 2 | ((int16_t)sbus.rx_buf[21] << 6 )) & 0x07FF;
      sbus.channels[15] = ((int16_t)sbus.rx_buf[21] >> 5 | ((int16_t)sbus.rx_buf[22] << 3 )) & 0x07FF;
      
      memset(sbus.rx_buf, 0, sizeof(sbus.rx_buf));

////    HAL_UART_Receive_DMA(&huart2, &sbus.rx_buf[0], 25);
//    HAL_UART_Receive_DMA(&huart2, sbus.rx_buf, SBUS_FRAME_SIZE);

    static char msg[128];  // 静态缓冲区，避免生命周期问题
    static uint32_t last_send_time = 0;
    
    // 限制发送频率，避免DMA过载
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_send_time < 20) {  // 50Hz最大频率
        return;
    }
    
    int len = snprintf(msg, sizeof(msg), 
                      "CH1:%4d CH2:%4d CH3:%4d CH4:%4d CH5:%4d CH6:%4d CH7:%4d CH8:%4d \r\n", 
                      sbus.channels[0], sbus.channels[1], 
                      sbus.channels[2], sbus.channels[3],
                      sbus.channels[4], sbus.channels[5], 
                      sbus.channels[6], sbus.channels[7]);
    
    // 安全检查
    if (len <= 0 || len >= sizeof(msg)) {
        return;  // 格式化错误
    }
    
    // 检查DMA传输状态
    if (huart2.gState == HAL_UART_STATE_READY) {
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
        if (status == HAL_OK) {
            last_send_time = current_time;
        }
         // 如果DMA忙，静默跳过，下次再试
    }

  } else {
     // 帧错误处理
    sbus.err_count++;
    // 发送错误信息到串口
    char error_msg[64];
    int len = snprintf(error_msg, sizeof(error_msg), 
                      "SBUS Frame Error! Header:0x%02X Footer:0x%02X\r\n", 
                      sbus.rx_buf[0], sbus.rx_buf[24]);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)error_msg, len);
//    // LED指示 - 信号异常
//    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  }

}

// 空闲中断回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    
    if (huart->Instance == USART1) { 
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);    // clear the flag of the IDLE
        HAL_UART_DMAStop(&huart1);             // stop the dma receive mode firstly
       sbus.rx_len = __HAL_DMA_GET_COUNTER(huart->hdmarx);
       sbus.rx_len =  SBUS_FRAME_SIZE;
        sbus.rx_len =  SBUS_FRAME_SIZE -__HAL_DMA_GET_COUNTER(huart1.hdmarx); 
       sbus.idle_flag = 1;
  }
}
// 全局变量记录上次的n值，避免频繁更新
static uint16_t last_n_value = 199;
static uint32_t last_update_time = 0;

// SBUS处理函数
void process_ch5_switch(void) {
    // CH5通道值：0~1023，下拨通常对应小值（<512），上拨对应大值（>=512）
    uint16_t ch5_val = sbus.channels[4];
    uint8_t ch5_current_state = (ch5_val < 512) ? 1 : 0;  // 1=下拨状态，0=未下拨状态

    // 检测CH5从“未下拨”变为“下拨”（边沿触发，防止重复使能）
    if (ch5_current_state == 1 && ch5_last_state == 0) {
        servo_enable_flag = 1;  // 开启舵机使能
        ch5_last_state = 1;     // 更新CH5上次状态
        // 可选：发送使能提示
        char enable_msg[] = "Servo Enabled (CH5 Down)\r\n";
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)enable_msg, strlen(enable_msg));
    }
    // CH5上拨时关闭舵机
     if (ch5_current_state == 0 && ch5_last_state == 1) {
         servo_enable_flag = 0;
        ch5_last_state = 0;
        char disable_msg[] = "Servo Disabled (CH5 Up)\r\n";
       HAL_UART_Transmit_DMA(&huart2, (uint8_t*)disable_msg, strlen(disable_msg));
     }
}

void process_sbus_throttle(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 限制更新频率，避免过于频繁的参数变化
    if (current_time - last_update_time < 100) {  // 每100ms更新一次
        return;
    }
    
    // 获取油门通道值并映射到n值
    uint16_t throttle = sbus.channels[2];  // 第三通道
    uint16_t new_n = map_throttle_to_n(throttle);
    
    //只有当n值变化较大时才更新，避免微小抖动
    if (abs((int16_t)new_n - (int16_t)last_n_value) >= 5) {
//        printf("SBUs: throttle=%d -> n=%d\r\n", throttle, new_n);
        
        // 调用参数更新函数
        // 保持原有的phi_max和phi_min，只更新n值
        Update_Motion_Parameters(current_params.phi_max, current_params.phi_min, new_n);
        
        last_n_value = new_n;
        last_update_time = current_time;
    }
}
// 将SBUS油门通道值[200-1800]映射到n值[99-399]
uint16_t map_throttle_to_n(uint16_t throttle_value) {
    // 输入范围限制
    if (throttle_value < 200) throttle_value = 200;
    if (throttle_value > 1800) throttle_value = 1800;
    
     // 线性映射: [200-1800] -> [49-199]
    // 公式: n = (throttle - 200) * (199-49) / (1800-200) + 49
    uint16_t n_value = (uint16_t)((throttle_value - 200) * 150.0f / 1600.0f + 49);
    
    //确保在范围内
    if (n_value < 49) n_value = 49;
    if (n_value > 199) n_value = 199;
    
    return n_value;
}
//-------------sbus program end------------------
// 获取当前状态信息
void Get_Current_Status(float* current_angle, uint16_t* current_t, 
                       SystemState_t* state, uint8_t* is_transition) {
    if (system_state == STATE_DOWNSTROKE || system_state == STATE_UPSTROKE || 
        system_state == STATE_TRANSITION_UPSTROKE) {
        *current_angle = calculate_phi_value(t_counter, &current_params,1.0f);
    } else {
        *current_angle = 0.0f;
    }
    *current_t = t_counter;
    *state = system_state;
    *is_transition = transition_active;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
