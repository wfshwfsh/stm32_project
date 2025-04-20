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
#include "math.h"
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
//if FS_I6A define to 1, means control by receive FS_I6A
#define FS_I6A 0

#define IBUS_FRAME_SIZE 32
#define MPU6050_ADDR 0x68 << 1
#define MPU6250_ADDR_R (0x68 << 1 | 1)
#define MPU6250_ADDR_W (0x68 << 1)
#define PI 3.14159265359f
#define PERIOD_MS 100.0f 
#define A2212_CH2_Base 100
#define A2212_CH2_Offset (+49)
#define A2212_CH2_RATE 1.0445
#define PID_CH1_ADJUST (+29)
#define PID_CH2_ADJUST (-13)

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


void MPU6250_Init(void)
{
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;

// PID parameter
float Kp = 0.176;//0.102;//0.0320;
float Ki = 0.000210;//0.05;
float Kd = 0;//0.35;//4.15;

//angle: 0.48 - 0.52
#define FUSION_RATE_ANGLE 0.48
//#define FUSION_RATE_PID 0.65
#define FUSION_RATE_PID 0.20


void MPU6250_ReadData(void)
{
	uint8_t mpu_data[14];
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

float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;

uint32_t currentTime=0, lastTime=0;
float error=0.0, lastError=0.0;
float	targetAngle=0.0, angleOffset=0.0;
float	prev_angle=0.0, curAngle=0.0;
float dt, integral=0.0;
float pidOutput=0.0, prev_pidOutput=0.0, prev_pidAdjust=0.0;

void Convert_MPU_Data(void)
{
	Accel_Xg = Accel_X / 16384.0;
	Accel_Yg = Accel_Y / 16384.0;
	Accel_Zg = Accel_Z / 16384.0;

	if( (gyro_x_offset == 0) && (gyro_y_offset == 0) && (gyro_z_offset == 0) ){
		Gyro_Xdps = Gyro_X / 131.0;
		Gyro_Ydps = Gyro_Y / 131.0;
		Gyro_Zdps = Gyro_Z / 131.0;
	}else{
		Gyro_Xdps = (Gyro_X / 131.0) - gyro_x_offset;
		Gyro_Ydps = (Gyro_Y / 131.0) - gyro_y_offset;
		Gyro_Zdps = (Gyro_Z / 131.0) - gyro_z_offset;
	}
}

//initial MPU6250
void Calibrate_Gyro_Offset()
{
    int samples = 1000;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < samples; i++) {
        MPU6250_ReadData(); // raw value or dps, ?????
				Convert_MPU_Data();

        sum_x += Gyro_Xdps;
        sum_y += Gyro_Ydps;
        sum_z += Gyro_Zdps;
        HAL_Delay(2); // ??????????
    }

    gyro_x_offset = sum_x / samples;
    gyro_y_offset = sum_y / samples;
    gyro_z_offset = sum_z / samples;
}

void show_MPU_Data()
{
	printf("Accel: X=%.2fg Y=%.2fg Z=%.2fg\r\n", Accel_Xg, Accel_Yg, Accel_Zg);
	printf("Gyro: X=%.2f Y=%.2f Z=%.2f\r\n"   , Gyro_Xdps, Gyro_Ydps, Gyro_Zdps);
}

//TIM_CHANNEL_ALL
void start_pwm(int ch) {
	HAL_TIM_PWM_Start(&htim3,ch);
}

void stop_pwm(int ch) {
	HAL_TIM_PWM_Stop(&htim3, ch);
}

void set_pwm(int ch, int pwmVal) {
	if(TIM_CHANNEL_2 == ch)
		pwmVal = pwmVal+A2212_CH2_Base;
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
 
	printf("CH1:%d  CH2:%d  CH3:%d  CH4:%d  CH5:%d  CH6:%d\r\n",
					cur_channels[0], cur_channels[1], cur_channels[2],
					cur_channels[3], cur_channels[4], cur_channels[5]);
}

int pwm_filter(int val)
{
	if(val <= 1000){
		return 1000;
	}else if(val > 2000){
		return 2000;
	}else{
		return val;
	}
}

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

void angle_calculate()
{
  currentTime = HAL_GetTick();
	if(0 == lastTime) goto UPDATE_TS;
  dt = (currentTime - lastTime)/PERIOD_MS;
	
  //------- acc --------------------------
  float _angle = -atan2(Accel_Xg, Accel_Zg) * 180 / PI - angleOffset;
	
  //-------  ---------------
	curAngle = FUSION_RATE_ANGLE * (curAngle + Gyro_Ydps * dt / 1000) + (1-FUSION_RATE_ANGLE) * _angle;
	
	//printf("Gyro_Xdps:%f \r\n", Gyro_Ydps);
	//printf("cur_ts:%u, last_ts:%u, dt:%f\r\n", currentTime, lastTime, dt);
	//printf("_angle:%f, curAngle:%f \r\n", _angle, curAngle);
  
	prev_angle = curAngle;
UPDATE_TS:
  lastTime = currentTime;
}

void pid_calculate()
{
	if(0 == dt) return;
  error = curAngle-targetAngle;
	
	// pid_i:
	//if(error > -3.0 && error < 3.0){
		integral += error*dt;
	//}
	
	// pid_d:
  float derivative = (error-lastError)/dt;
	float PID = Kp * error + Ki * integral + Kd * derivative;
  
	//pidOutput = FUSION_RATE_PID*prev_pidOutput + (1-FUSION_RATE_PID)*PID;
	pidOutput = PID;
	//printf("prev_pidOutput:%f pidOutput:%f PID:%f\r\n", prev_pidOutput, pidOutput, PID);
	//prev_pidOutput = pidOutput;
	lastError = error;
  
  //printf("%f\r\n", pidOutput);
	printf("curAngle:%f\t P:%f\t I:%f\t D:%f\r\n", curAngle, Kp * error, Ki * integral, Kd * derivative);
	//printf("curAngle:%f PID:%f\r\n", curAngle, PID);
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
	MX_TIM3_Init();
	
	printf("### init: i2c MPU\r\n");
  MX_I2C1_Init();
	
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
