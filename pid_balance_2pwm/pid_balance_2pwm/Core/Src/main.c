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

int count=0;

//TIM_CHANNEL_ALL


float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int compensatePIDOutput(float pid_output, float angle) {
    int compensation = 0;

    // +++ : 0 ~ +20 => 0 ~ +27
    if (angle > 0.0f) {
        if (angle > 20.0f) angle = 19.3f;
        compensation = map_float((angle * 100.0f), 0, 1930, 0, PID_CH1_ADJUST);
    }
    // --- : 0 ~ -15 => 0 ~ -11
    else if (angle < 0.0f) {
        if (angle < -15.0f) angle = -15.0f;
        compensation = map_float((-angle * 100.0f), 0, 1500, 0, PID_CH2_ADJUST);
    }
		
		//printf("compensation:%d\n", compensation);
    // PID
    int compensated_output = pid_output + compensation;
    return compensated_output;
}

void update_ch3(int val, float pid)
{
	//int pwm_l = pwm_filter(val+(pid/2.0));
	//int pwm_r = pwm_filter(val-(pid/2.0));
	int pwm_l;
	int pwm_r;
	int pid_adjust, _pid_adjust;
	_pid_adjust = compensatePIDOutput(pid, curAngle);
	_pid_adjust = _pid_adjust - (A2212_CH2_Offset/2);
	pid_adjust = (1-FUSION_RATE_PID)*prev_pidAdjust + FUSION_RATE_PID*_pid_adjust;
	//printf("_pid_adjust:%d adj:%d\n", _pid_adjust, pid_adjust);
	prev_pidAdjust=pid_adjust;
	pwm_l = pwm_filter(val+pid_adjust);
	pwm_r = pwm_filter(val-pid_adjust);
	
	set_pwm(TIM_CHANNEL_1, pwm_l);
	//set_pwm(TIM_CHANNEL_2, pwm_r);
	set_pwm(TIM_CHANNEL_2, pwm_r);
	//set_pwm(TIM_CHANNEL_2, (A2212_CH2_RATE*pwm_r));
}




void IBUS_2pwm() {
	int throttle;
	
	// update CH3: throttle
#if FS_I6A
	throttle = pwm_filter(cur_channels[2]);
#else
	throttle = 1350;
#endif
	angle_calculate();
	pid_calculate();
	
	update_ch3(throttle, pidOutput);//pidOutput
}

void IBUS_Parse() {
    if (ibus_buffer[0] != 0x20 || ibus_buffer[1] != 0x40) {
        return;
    }
		
    uint8_t checksum = 0;
    for (int i = 0; i < 31; i++) {
        checksum += ibus_buffer[i];
			  //printf("%d: %02x ", i, ibus_buffer[i]);
    }
		//printf("\n");
		
		//printf("CRC = %d, b[31] = %d \n", checksum, ibus_buffer[31]);
    //if (checksum == ibus_buffer[31]) {
        for (int i = 0; i < 10; i++) {  // ??? 10 ???
            ibus_channels[i] = ibus_buffer[4 + (i * 2)] | (ibus_buffer[3 + (i * 2)] << 8);
        }

        //printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d  CH5:%d  CH6:%d\r\n",
        //    ibus_channels[0], ibus_channels[1], ibus_channels[2],
        //    ibus_channels[3], ibus_channels[4], ibus_channels[5]);
    //}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == USART2){
		idx = size;
	}else if (huart->Instance == USART6){
		ibus_idx = size;
		
		if(count % 100 == 0){
			IBUS_Parse();
			IBUS_update_ch();
		}
		
		memset(&ibus_buffer, 0, sizeof(ibus_buffer));
		IBUS_Init();
		count++;
	}
}

int tim1_cnt=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
		if(tim1_cnt % 200 == 0)
			printf("AAAAAAAAAA\r\n");
		
		tim1_cnt++;
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
  /* USER CODE BEGIN 2 */
	MPU6250_Init();
	Calibrate_Gyro_Offset();
	
	printf("### init: pwm\r\n");
	start_pwm(TIM_CHANNEL_1);
	start_pwm(TIM_CHANNEL_2);
	
	set_pwm(TIM_CHANNEL_1, 1000);
	set_pwm(TIM_CHANNEL_2, 1000);
	
	HAL_Delay(2000);
	printf("### init: IBUS\r\n");
	
	//IBUS_Init();
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
		
		if(count % 10 == 0){
			IBUS_2pwm();
		}
		
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
