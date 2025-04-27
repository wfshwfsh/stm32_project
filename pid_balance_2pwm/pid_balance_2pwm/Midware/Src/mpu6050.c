#include "mpu6050.h"
extern float dt_tim1_intr;

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;

float Accel_Xg, Accel_Yg, Accel_Zg;
float Gyro_Xdps, Gyro_Ydps, Gyro_Zdps;

float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;

void I2C_Delay()
{
	int z = 0xff;
	while (z--)
		;
}

uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_CFG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_SAMPLE_RATE_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	return MPU_Set_LPF(rate / 2);					  //????LPF???????
}



#define MPU6050_WAKE 	0x00
#define MPU6050_RESET 0x80

//initial MPU6250
void MPU6250_Init(void)
{
	uint8_t data;
	
	// reset
	data = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_PWR_MGMT1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(100);
	
	// wake
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_PWR_MGMT1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	// set gyro Fsr rate 2000dps
	data = (0x03 << 3);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_GYRO_CFG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	// set acc Fsr 2g
	data = (0x00 << 3);
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_ACCEL_CFG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	//set freq rate=50 
	MPU_Set_Rate(50);
	
	// disable all interrupt
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_INT_EN_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	// disable I2C master mode
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_USER_CTRL_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	// disable FIFO
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_FIFO_EN_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	// INT low enable
	data = 0X80;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_INTBP_CFG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	//MPU_Read_Byte(MPU_DEVICE_ID_REG);
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_DEVICE_ID_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	// set CLKSEL,PLL
	data = 0X01;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_PWR_MGMT1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	// both acc, gyro working
	data = 0X00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_PWR_MGMT2_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	
	MPU_Set_Rate(200);	
	
}

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

#define PERIOD_MS 100.0f

//angle: 0.48 - 0.52
#define FUSION_RATE_ANGLE 0.99

float	prev_angle=0.0, curAngle=0.0;
float angleOffset=0.0;


void angle_calculate()
{
  //------- acc --------------------------
  float accAngle = -atan2(Accel_Xg, Accel_Zg) * 180 / PI - angleOffset;
	
  //-------  ---------------
	curAngle = FUSION_RATE_ANGLE * (curAngle + Gyro_Ydps * dt_tim1_intr / 1000) + (1-FUSION_RATE_ANGLE) * accAngle;
	
	//printf("Gyro_Xdps:%f \r\n", Gyro_Ydps);
	//printf("_angle:%f, curAngle:%f \r\n", accAngle, curAngle);
  
	prev_angle = curAngle;
}
