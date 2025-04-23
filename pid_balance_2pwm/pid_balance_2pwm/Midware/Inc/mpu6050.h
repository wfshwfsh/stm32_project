#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "i2c.h"

#define PI 3.14159265359f

#define MPU6050_ADDR 0x68 << 1
#define MPU6250_ADDR_R (0x68 << 1 | 1)
#define MPU6250_ADDR_W (0x68 << 1)

#define MPU_ADDR 0X68

#define MPU_ADDR 0X68

////因?模?AD0默?接GND,所以????地址后,?0XD1和0XD0(如果接VCC,??0XD3和0XD2)
#define MPU_READ 0XD1
#define MPU_WRITE 0XD0

//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可?取版本?,寄存器手?未提到
//#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手?未提到
#define MPU_SELF_TESTX_REG 0X0D   //自檢寄存器X
#define MPU_SELF_TESTY_REG 0X0E   //自檢寄存器Y
#define MPU_SELF_TESTZ_REG 0X0F   //自檢寄存器Z
#define MPU_SELF_TESTA_REG 0X10   //自檢寄存器A
#define MPU_SAMPLE_RATE_REG 0X19  //采樣頻率分頻器
#define MPU_CFG_REG 0X1A          //配置寄存器
#define MPU_GYRO_CFG_REG 0X1B     //陀螺儀配置寄存器
#define MPU_ACCEL_CFG_REG 0X1C    //加速度計配置寄存器
#define MPU_MOTION_DET_REG 0X1F   //?????值?置寄存器
#define MPU_FIFO_EN_REG 0X23      // FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG 0X24  // IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG 0X25 // IIC從机0器件地址寄存器
#define MPU_I2CSLV0_REG 0X26      // IIC從机0數据地址寄存器
#define MPU_I2CSLV0_CTRL_REG 0X27 // IIC從机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG 0X28 // IIC從机1器件地址寄存器
#define MPU_I2CSLV1_REG 0X29      // IIC從机1數据地址寄存器
#define MPU_I2CSLV1_CTRL_REG 0X2A // IIC從机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG 0X2B // IIC從机2器件地址寄存器
#define MPU_I2CSLV2_REG 0X2C      // IIC從机2數据地址寄存器
#define MPU_I2CSLV2_CTRL_REG 0X2D // IIC從机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG 0X2E // IIC從机3器件地址寄存器
#define MPU_I2CSLV3_REG 0X2F      // IIC從机3數据地址寄存器
#define MPU_I2CSLV3_CTRL_REG 0X30 // IIC從机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG 0X31 // IIC從机4器件地址寄存器
#define MPU_I2CSLV4_REG 0X32      // IIC從机4數据地址寄存器
#define MPU_I2CSLV4_DO_REG 0X33   // IIC從机4寫數据寄存器
#define MPU_I2CSLV4_CTRL_REG 0X34 // IIC從机4控制寄存器
#define MPU_I2CSLV4_DI_REG 0X35   // IIC從机4讀數据寄存器

#define MPU_I2CMST_STA_REG 0X36 // IIC主机狀態寄存器
#define MPU_INTBP_CFG_REG 0X37  //中斷/旁路?置寄存器
#define MPU_INT_EN_REG 0X38     //中斷使能寄存器
#define MPU_INT_STA_REG 0X3A    //中斷狀態寄存器

#define MPU_ACCEL_XOUTH_REG 0X3B //加速度值,X軸高8位寄存器
#define MPU_ACCEL_XOUTL_REG 0X3C //加速度值,X軸低8位寄存器
#define MPU_ACCEL_YOUTH_REG 0X3D //加速度值,Y軸高8位寄存器
#define MPU_ACCEL_YOUTL_REG 0X3E //加速度值,Y軸低8位寄存器
#define MPU_ACCEL_ZOUTH_REG 0X3F //加速度值,Z軸高8位寄存器
#define MPU_ACCEL_ZOUTL_REG 0X40 //加速度值,Z軸低8位寄存器

#define MPU_TEMP_OUTH_REG 0X41 //?度值高八位寄存器
#define MPU_TEMP_OUTL_REG 0X42 //?度值低8位寄存器

#define MPU_GYRO_XOUTH_REG 0X43 //陀螺儀值,X軸高8位寄存器
#define MPU_GYRO_XOUTL_REG 0X44 //陀螺儀值,X軸低8位寄存器
#define MPU_GYRO_YOUTH_REG 0X45 //陀螺儀值,Y軸高8位寄存器
#define MPU_GYRO_YOUTL_REG 0X46 //陀螺儀值,Y軸低8位寄存器
#define MPU_GYRO_ZOUTH_REG 0X47 //陀螺儀值,Z軸高8位寄存器
#define MPU_GYRO_ZOUTL_REG 0X48 //陀螺儀值,Z軸低8位寄存器

#define MPU_I2CSLV0_DO_REG 0X63 // IIC從机0數据寄存器
#define MPU_I2CSLV1_DO_REG 0X64 // IIC從机1數据寄存器
#define MPU_I2CSLV2_DO_REG 0X65 // IIC從机2數据寄存器
#define MPU_I2CSLV3_DO_REG 0X66 // IIC從机3數据寄存器

#define MPU_I2CMST_DELAY_REG 0X67 // IIC主机延時管理寄存器
#define MPU_SIGPATH_RST_REG 0X68  //信號通道复位寄存器
#define MPU_MDETECT_CTRL_REG 0X69 //????控制寄存器
#define MPU_USER_CTRL_REG 0X6A    //用?控制寄存器
#define MPU_PWR_MGMT1_REG 0X6B    //電源管理寄存器1
#define MPU_PWR_MGMT2_REG 0X6C    //電源管理寄存器2
#define MPU_FIFO_CNTH_REG 0X72    // FIFO計數寄存器高八位
#define MPU_FIFO_CNTL_REG 0X73    // FIFO計數寄存器低八位
#define MPU_FIFO_RW_REG 0X74      // FIFO讀寫寄存器
#define MPU_DEVICE_ID_REG 0X75    //器件ID寄存器

extern int16_t Accel_X, Accel_Y, Accel_Z;
extern int16_t Gyro_X, Gyro_Y, Gyro_Z;

extern float Accel_Xg, Accel_Yg, Accel_Zg;
extern float Gyro_Xdps, Gyro_Ydps, Gyro_Zdps;

extern float gyro_x_offset;
extern float gyro_y_offset;
extern float gyro_z_offset;

extern uint32_t currentTime, lastTime;
extern float	prev_angle, curAngle;
extern float angleOffset;
extern float dt;


void MPU6250_Init(void);
void MPU6250_ReadData(void);
void Convert_MPU_Data(void);
void Calibrate_Gyro_Offset();
void show_MPU_Data();


void angle_calculate();
