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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#include <string.h>
#include "uart_printf.h"
#include "mpu6050.h"
#include "pwm.h"
#include "pid.h"
#include "fs-i6x.h"

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//if FS_I6A define to 1, means control by receive FS_I6A
#define FS_I6A 0
int throttle = 1150;
int count=0;
int isReady=0;
float dt_tim1_intr=0;

//TIM_CHANNEL_ALL
#define FUSION_RATE_PID 1.0
//#define FUSION_RATE_PID 0.20

void show_parameters()
{
	printf("###################################\r\n");
	printf("Kp:%f, Ki:%f, Kd:%f\r\n\r\n", Kp, Ki, Kd);
}

int ch_cnt=0;
void update_ch3(int val, float pid)
{
	int pwm_1;
	int pwm_2;
	float pid_adjust=0, _pid_adjust=0;
	//_pid_adjust = compensatePIDOutput(pid, curAngle);
	_pid_adjust = pid;
	pid_adjust = (1-FUSION_RATE_PID)*prev_pidAdjust + FUSION_RATE_PID*_pid_adjust;
	//printf("_adjust:%f adjust:%f\r\n", _pid_adjust, pid_adjust);
	
	pwm_1 = (int)pwm_filter(val+pid_adjust-(A2212_CH2_Offset/2));
	pwm_2 = (int)pwm_filter(val-pid_adjust+(A2212_CH2_Offset/2));
	
	if(ch_cnt++ % 100 == 0)
		printf("pwm_1:%d , pwm2:%d\r\n", pwm_1, pwm_2);
	
	set_pwm(TIM_CHANNEL_1, pwm_1);
	set_pwm(TIM_CHANNEL_2, pwm_2);
	//set_pwm(TIM_CHANNEL_2, (A2212_CH2_RATE*pwm_2));
}

float ftimer_2ms(float ftimer)
{
	return (1000/ftimer);
}

//SysClk=72MHz, Prescaler=49, ARR=7199
float calc_clk(int sysclk, int prescaler, int arr)
{
	float f_timer = sysclk/((prescaler+1)*(arr+1));
	return ftimer_2ms(f_timer);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
		if(isReady)
		{
			// update CH3: throttle
		#if FS_I6A
			throttle = pwm_filter(cur_channels[2]);
		#endif
			angle_calculate();
			pid_calculate();
			update_ch3(throttle, pidOutput);//pidOutput
		}
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == USART2){
		
	}else if (huart->Instance == USART6){
		
		if(count % 100 == 0){
			IBUS_Parse();
			IBUS_update_ch();
		}
		
		memset(&ibus_buffer, 0, sizeof(ibus_buffer));
		IBUS_Init();
		count++;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	show_parameters();
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
	HAL_TIM_Base_Start_IT(&htim1);
	dt_tim1_intr = calc_clk(72000000, 7199, 49);
	
  /* USER CODE BEGIN 2 */
	printf("### init: pwm\r\n");
	start_pwm(TIM_CHANNEL_1);
	start_pwm(TIM_CHANNEL_2);
	
	set_pwm(TIM_CHANNEL_1, 1000);
	set_pwm(TIM_CHANNEL_2, 1000);
	
	HAL_Delay(5000);
	printf("### init: MPU\r\n");
	MPU6250_Init();
	Calibrate_Gyro_Offset();
	
	printf("### init: IBUS\r\n");
	//IBUS_Init();
	
	isReady = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		count++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MPU6250_ReadData();
		Convert_MPU_Data();
		
		if(count % 200 == 0){
			//show_MPU_Data();
			//IBUS_show_ch();
		}
		
		HAL_Delay(10);
  }
	
	stop_pwm(TIM_CHANNEL_1);
	stop_pwm(TIM_CHANNEL_2);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
