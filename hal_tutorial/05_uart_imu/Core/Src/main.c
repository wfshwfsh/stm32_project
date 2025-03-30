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
#include <stdio.h>
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
//#include "stdio.h"
//#include "string.h"

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

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

FILE __stdout;

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int ferror(FILE *f){
  /* Your implementation of ferror(). */
  return 0;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MPU6050_ADDR 0x68 << 1
#define MPU6250_ADDR_R (0x68 << 1 | 1)
#define MPU6250_ADDR_W (0x68 << 1)
 
void MPU6250_Init(void)
{
    uint8_t data = 0x00;  // ??????
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;
uint8_t mpu_data[14];

void MPU6250_ReadData(void)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, mpu_data, 14, HAL_MAX_DELAY);

    Accel_X = (int16_t)(mpu_data[0] << 8 | mpu_data[1]);
    Accel_Y = (int16_t)(mpu_data[2] << 8 | mpu_data[3]);
    Accel_Z = (int16_t)(mpu_data[4] << 8 | mpu_data[5]);

    Gyro_X  = (int16_t)(mpu_data[8] << 8 | mpu_data[9]);
    Gyro_Y  = (int16_t)(mpu_data[10] << 8 | mpu_data[11]);
    Gyro_Z  = (int16_t)(mpu_data[12] << 8 | mpu_data[13]);
}

float Accel_Xg, Accel_Yg, Accel_Zg;
float Gyro_Xdps, Gyro_Ydps, Gyro_Zdps;

void Convert_MPU_Data(void)
{
    Accel_Xg = Accel_X / 16384.0;
    Accel_Yg = Accel_Y / 16384.0;
    Accel_Zg = Accel_Z / 16384.0;

    Gyro_Xdps = Gyro_X / 131.0;
    Gyro_Ydps = Gyro_Y / 131.0;
    Gyro_Zdps = Gyro_Z / 131.0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
	
  /* USER CODE BEGIN 2 */
	char msg[] = "Hello STM32F411!\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg) - 1, 100);
	
	MPU6250_Init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		MPU6250_ReadData();
		Convert_MPU_Data();

		printf("Accel: X=%.2fg Y=%.2fg Z=%.2fg\n", Accel_Xg, Accel_Yg, Accel_Zg);
		printf("Gyro: X=%.2f°/s Y=%.2f°/s Z=%.2f°/s\n"   , Gyro_Xdps, Gyro_Ydps, Gyro_Zdps);
		
		HAL_Delay(500);
		
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
