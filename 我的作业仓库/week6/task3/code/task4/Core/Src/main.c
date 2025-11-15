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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void Set_Motor_PWM(uint32_t pwm);
void Button_Scan(void);
void PID_Control(void);
#include<stdio.h>
#include"key_timer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_PULSES_PER_REV 13.0f 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct {
    float Kp, Ki, Kd;
    float last_error;
    float prev_error;
    float integral;
} PID_t;

typedef enum {
    MOTOR_MODE1 = 0,  
    MOTOR_MODE2,      
    MOTOR_MODE3       
} Motor_ModeTypeDef;

volatile static Motor_ModeTypeDef motor_mode = MOTOR_MODE1;  

PID_t speed_pid = {1.5f, 0.4f, 0.1f, 0.0f, 0.0f, 0.0f};
volatile float target_speed = 1.0f;
volatile float actual_speed = 0.0f;
volatile uint32_t pwm_value = 400;

volatile int32_t encoder_total = 0;
volatile int16_t encoder_last = 0;

volatile uint32_t speed_calc_time = 0;
volatile int32_t last_encoder_total = 0;
volatile uint32_t vofa_send_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Update_Encoder_Count(void);
void Calculate_Speed(void);
float Incremental_PID_Calculate(float target, float current) ;
void Set_Motor_PWM(uint32_t pwm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 重定向printf到串口，确保每次只发�?1个字节[citation:1]
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // 注意&huart1�?与您实际使用的串口一�?
    return ch;
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);       
HAL_TIM_Base_Start_IT(&htim1);                   
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
Set_Motor_PWM(pwm_value);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Key_StateTypeDef key = Key_GetState();
	  if (key == KEY_PRESSED) 
		{
			motor_mode = (Motor_ModeTypeDef)( ( (int)motor_mode + 1 ) & 3 );
            switch (motor_mode) 
			{
                case MOTOR_MODE1:
                    target_speed=1.0f;
                    break;
                case MOTOR_MODE2:
                   target_speed=2.0f;
                    break;
                case MOTOR_MODE3:
                    target_speed=3.0f;
                    break;
                default:
                   target_speed=1.0f;
                    break;
			}
		}	

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Update_Encoder_Count(void)
{
    int16_t current_count = TIM3->CNT;
    int16_t diff = current_count - encoder_last;
    if(diff > 32767) diff -= 65536;
    else if(diff < -32767) diff += 65536;
    
    encoder_total += diff;
    encoder_last = current_count;
}

void Calculate_Speed(void)
{
    static uint32_t last_calc_time = 0;
    uint32_t current_time = HAL_GetTick();
    if(current_time - last_calc_time >= 100)
    {
        int32_t count_diff = encoder_total - last_encoder_total;
        
        actual_speed = (count_diff / ENCODER_PULSES_PER_REV) / (0.1f); // 100ms = 0.1s
        
        last_encoder_total = encoder_total;
        last_calc_time = current_time;
    }
}
// 增量式PID计算
float Incremental_PID_Calculate(float target, float current) 
{
    float error = target - current;
    float delta_p = speed_pid.Kp * (error - speed_pid.last_error);
    float delta_i = speed_pid.Ki * error;
    float delta_d = speed_pid.Kd * (error - 2*speed_pid.last_error + speed_pid.prev_error);
    
    float increment = delta_p + delta_i + delta_d;
  
    speed_pid.prev_error = speed_pid.last_error;
    speed_pid.last_error = error;
    
    return increment;
}

uint32_t Constrain(uint32_t value, uint32_t min_val, uint32_t max_val)
{
    if(value < min_val) return min_val;
    if(value > max_val) return max_val;
    return value;
}

void Set_Motor_PWM(uint32_t pwm) 
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1, GPIO_PIN_RESET);
    pwm = Constrain(pwm, 0, 999);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
    pwm_value = pwm;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1) 
	{
		 static uint8_t pid_counter = 0;  
        Key_TimerScan();
        pid_counter++;
        if(pid_counter >= 1) 
		{ 
            PID_Control();
            pid_counter = 0;
        }
    }
}

void VOFA_SendData(float target, float actual, uint32_t pwm, float error)
{
    printf("%.2f,%.2f,%.2f,%.2f\n", target, actual, (float)pwm, error);
}
void PID_Control(void)
{
    Update_Encoder_Count();
    Calculate_Speed();
    float pid_adjust = Incremental_PID_Calculate(target_speed, actual_speed);
    int32_t new_pwm = pwm_value + (int32_t)pid_adjust;
    Set_Motor_PWM(Constrain(new_pwm, 0, 999));
   vofa_send_counter++;
    if (vofa_send_counter >= 5) 
	{
        VOFA_SendData(target_speed, actual_speed, pwm_value, target_speed - actual_speed);
        vofa_send_counter = 0;
    }
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
