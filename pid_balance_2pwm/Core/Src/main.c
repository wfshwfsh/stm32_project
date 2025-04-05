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
#include <stdio.h>
#include <string.h>

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

#define IBUS_FRAME_SIZE 32
uint8_t ibus_buffer[IBUS_FRAME_SIZE]={};
uint16_t ibus_channels[10]={};
uint16_t cur_channels[10]={};
int idx=0, ibus_idx =0;
int count=0;

enum {
	eCH1 = 0, 
	eCH2, 
	eCH3_THROTTLE, 
	eCH4,
};



//TIM_CHANNEL_ALL
void start_pwm(int ch) {
	HAL_TIM_PWM_Start(&htim3,ch);
}

void stop_pwm(int ch) {
	HAL_TIM_PWM_Stop(&htim3, ch);
}

void set_pwm(int ch, int pwmVal) {
	__HAL_TIM_SetCompare(&htim3, ch, pwmVal);
}
	
void IBUS_Init() {
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, ibus_buffer, IBUS_FRAME_SIZE);
}

void IBUS_update_ch() {
		for(int i=0;i<10;i++){
			  cur_channels[i] = ibus_channels[i];
		}
}

void IBUS_show_ch() {
 
		printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d  CH5:%d  CH6:%d\n",
            cur_channels[0], cur_channels[1], cur_channels[2],
            cur_channels[3], cur_channels[4], cur_channels[5]);
}

int pwm_filter(int val) {
		
		if(val <= 1000){
				return 1000;
		}else if(val > 2000){
				return 2000;
		}else{
				return val;
		}
}

void update_ch3(int val)
{
		set_pwm(TIM_CHANNEL_1, val);
		set_pwm(TIM_CHANNEL_2, val);
}

void IBUS_2pwm() {
		int pwmVal;
		
		// update CH3: throttle
		pwmVal = pwm_filter(cur_channels[2]);
		update_ch3(pwmVal);
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

        //printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d  CH5:%d  CH6:%d\n",
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
		
		//printf("666\n");
		memset(&ibus_buffer, 0, sizeof(ibus_buffer));
		IBUS_Init();
		count++;
	}
}

void pid_balance()
{
	

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
  /* USER CODE BEGIN 2 */
	start_pwm(TIM_CHANNEL_1);
	start_pwm(TIM_CHANNEL_2);
	set_pwm(TIM_CHANNEL_ALL, 1000);
	IBUS_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//int pwmVal=1100;
	//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	
  while (1)
  {
		count++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(count % 10 == 0){
			IBUS_2pwm();
		}
		
		if(count % 200 == 0){
			IBUS_show_ch();
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
