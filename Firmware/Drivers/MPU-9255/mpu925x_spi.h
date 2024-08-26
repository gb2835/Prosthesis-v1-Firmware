/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_MPU925X_SPI_H_
#define INC_MPU925X_SPI_H_

#include "stm32l4xx_ll_spi.h"

#define MPU9250_DEVICE_ID	0x71
#define MPU9255_DEVICE_ID	0x73

#define MPU925X_REG_SMPLRT_DIV		0x19
#define MPU925X_REG_CONFIG			0x1A
#define MPU925X_REG_GYRO_CONFIG		0x1B
#define MPU925X_REG_ACCEL_CONFIG	0x1C
#define MPU925X_REG_ACCEL_CONFIG_2	0x1D
#define MPU925X_REG_ACCEL_XOUT_H	0x3B
#define MPU925X_REG_ACCEL_XOUT_L	0x3C
#define MPU925X_REG_ACCEL_YOUT_H	0x3D
#define MPU925X_REG_ACCEL_YOUT_L	0x3E
#define MPU925X_REG_ACCEL_ZOUT_H	0x3F
#define MPU925X_REG_ACCEL_ZOUT_L	0x40
#define MPU925X_REG_TEMP_OUT_H		0x41
#define MPU925X_REG_TEMP_OUT_L		0x42
#define MPU925X_REG_GYRO_XOUT_H		0x43
#define MPU925X_REG_GYRO_XOUT_L		0x44
#define MPU925X_REG_GYRO_YOUT_H		0x45
#define MPU925X_REG_GYRO_YOUT_L		0x46
#define MPU925X_REG_GYRO_ZOUT_H		0x47
#define MPU925X_REG_GYRO_ZOUT_L		0x48
#define MPU925X_REG_WHO_AM_I		0X75

#define MPU925X_ACCEL_SENSITIVITY_2G		16384
#define MPU925X_ACCEL_SENSITIVITY_4G		8192
#define MPU925X_ACCEL_SENSITIVITY_8G		4096
#define MPU925X_ACCEL_SENSITIVITY_16G		2048
#define MPU925X_GYRO_SENSITIVITY_250DPS		16384
#define MPU925X_GYRO_SENSITIVITY_500DPS		8192
#define MPU925X_GYRO_SENSITIVITY_1000DPS	4096
#define MPU925X_GYRO_SENSITIVITY_2000DPS	2048
#define MPU925X_TEMP_ROOMTEMP				21
#define MPU925X_TEMP_SENSITIVITY			333.87

enum MPU925x_AccelDlpfBandWidth_e
{
	mpu925x_accelDlpfBandWidth_5hz,
	mpu925x_accelDlpfBandWidth_10hz,
	mpu925x_accelDlpfBandWidth_20hz,
	mpu925x_accelDlpfBandWidth_41hz,
	mpu925x_accelDlpfBandWidth_92hz,
	mpu925x_accelDlpfBandWidth_184hz,
	mpu925x_accelDlpfBandWidth_460hz,
	mpu925x_accelDlpfBandWidth_1130hz
};

enum MPU925x_AccelSensitivity_e
{
	mpu925x_accelSensitivity_2g,
	mpu925x_accelSensitivity_4g,
	mpu925x_accelSensitivity_8g,
	mpu925x_accelSensitivity_16g
};

enum MPU925x_GyroDlpfBandWidth_e
{
	mpu925x_gyroDlpfBandWidth_5hz,
	mpu925x_gyroDlpfBandWidth_10hz,
	mpu925x_gyroDlpfBandWidth_20hz,
	mpu925x_gyroDlpfBandWidth_41hz,
	mpu925x_gyroDlpfBandWidth_92hz,
	mpu925x_gyroDlpfBandWidth_184hz,
	mpu925x_gyroDlpfBandWidth_250hz,
	mpu925x_gyroDlpfBandWidth_3600hz,
	mpu925x_gyroDlpfBandWidth_8800hz
};

enum MPU925x_GyroSensitivity_e
{
	mpu925x_gyroSensitivity_250dps,
	mpu925x_gyroSensitivity_500dps,
	mpu925x_gyroSensitivity_1000dps,
	mpu925x_gyroSensitivity_2000dps
};

struct MPU925x_IMUData_s
{
   double	ax;
   double	ay;
   double	az;
   double	gx;
   double	gy;
   double	gz;
};

uint8_t MPU925x_Init(SPI_TypeDef *spix, GPIO_TypeDef *cs_gpiox, uint16_t cs_pinx);
void MPU925x_SetAccelSensitivity(enum MPU925x_AccelSensitivity_e option);
void MPU925x_SetGyroSensitivity(enum MPU925x_GyroSensitivity_e option);
void MPU925x_SetAccelDlpfBandwidth(enum MPU925x_AccelDlpfBandWidth_e option);
void MPU925x_SetGyroDlpfBandwidth(enum MPU925x_GyroDlpfBandWidth_e option);
void MPU925x_SetSampleRateDiv(uint8_t divider);
struct MPU925x_IMUData_s MPU925x_ReadIMU(void);
void MPU925x_WriteReg(uint8_t adress, uint8_t data);
void MPU925x_ReadRegs(uint8_t address, uint8_t *data, uint8_t bytes);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_MPU925X_SPI_H_ */
