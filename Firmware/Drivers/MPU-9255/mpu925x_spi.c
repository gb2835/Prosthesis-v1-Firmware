/*******************************************************************************
*
* TITLE:	Driver for InvenSense MPU-9250 and MPU-9255 IMU using SPI
* AUTHOR:	Greg Berkeley
* RELEASE:	08/24/2024
*
* NOTES
* 1. Unless otherwise specified, units are
* 		- Accelerometer = g's
* 		- Gyroscope     = degrees/second
*
*******************************************************************************/

#include "mpu925x_spi.h"
#include "stm32l4xx_ll_gpio.h"


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

typedef struct
{
	SPI_TypeDef		*spiHandle;
	GPIO_TypeDef	*CS_GPIOx;
	uint16_t		CS_Pin;
} MPU925x_t;

float accelSensitivity = MPU925X_ACCEL_SENSITIVITY_2G;		// ±2g is default
float gyroSensitivity = MPU925X_GYRO_SENSITIVITY_250DPS;	// ±250°/s is default
MPU925x_t mpu925x;


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

uint8_t MPU925x_Init(SPI_TypeDef *spix, GPIO_TypeDef *cs_gpiox, uint16_t cs_pinx)
{
	uint8_t whoAmI;

	mpu925x.spiHandle = spix;
	mpu925x.CS_GPIOx = cs_gpiox;
	mpu925x.CS_Pin = cs_pinx;

	MPU925x_ReadRegs(MPU925X_REG_WHO_AM_I, &whoAmI, 1);

	if((whoAmI != MPU9250_DEVICE_ID) && (whoAmI != MPU9255_DEVICE_ID))
		return 1;

	return 0;
}

void MPU925x_SetAccelSensitivity(MPU925x_AccelSensitivity_t option)
{
	uint8_t data;

	switch (option)
	{
	case mpu925x_accelSensitivity_2g:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = data & ~0x18;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_2G;
		break;

	case mpu925x_accelSensitivity_4g:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x08;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_4G;
		break;

	case mpu925x_accelSensitivity_8g:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x10;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_8G;
		break;

	case mpu925x_accelSensitivity_16g:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x18;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_16G;
		break;
	}
}

void MPU925x_SetGyroSensitivity(MPU925x_GyroSensitivity_t option)
{
	uint8_t data;

	switch (option)
	{
	case mpu925x_gyroSensitivity_250dps:
		MPU925x_ReadRegs(MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = data & ~0x18;
		MPU925x_WriteReg(MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_250DPS;
		break;

	case mpu925x_gyroSensitivity_500dps:
		MPU925x_ReadRegs(MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x08;
		MPU925x_WriteReg(MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_500DPS;
		break;

	case mpu925x_gyroSensitivity_1000dps:
		MPU925x_ReadRegs(MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x10;
		MPU925x_WriteReg(MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_1000DPS;
		break;

	case mpu925x_gyroSensitivity_2000dps:
		MPU925x_ReadRegs(MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x18;
		MPU925x_WriteReg(MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_2000DPS;
		break;
	}
}

void MPU925x_SetAccelDlpfBandwidth(MPU925x_AccelDlpfBandWidth_t option)
{
	uint8_t data;

	switch (option)
	{
	case mpu925x_accelDlpfBandWidth_5hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x06;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case mpu925x_accelDlpfBandWidth_10hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x05;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case mpu925x_accelDlpfBandWidth_20hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x04;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case mpu925x_accelDlpfBandWidth_41hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x03;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case mpu925x_accelDlpfBandWidth_92hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x02;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case mpu925x_accelDlpfBandWidth_184hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x01;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case mpu925x_accelDlpfBandWidth_460hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = data & ~0x0F;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case mpu925x_accelDlpfBandWidth_1130hz:
		MPU925x_ReadRegs(MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x08;
		MPU925x_WriteReg(MPU925X_REG_ACCEL_CONFIG_2, data);
		break;
	}
}

void MPU925x_SetGyroDlpfBandwidth(MPU925x_GyroDlpfBandWidth_t option)
{
	uint8_t data;

	switch (option)
	{
	case mpu925x_gyroDlpfBandWidth_5hz:
		MPU925x_ReadRegs(MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x06;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_10hz:
		MPU925x_ReadRegs(MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x05;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_20hz:
		MPU925x_ReadRegs(MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x04;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_41hz:
		MPU925x_ReadRegs(MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x03;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_92hz:
		MPU925x_ReadRegs(MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x02;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_184hz:
		MPU925x_ReadRegs(MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x01;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_250hz:
		MPU925x_ReadRegs(MPU925X_REG_CONFIG, &data, 1);
		data = data & ~0x07;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_3600hz:
		MPU925x_ReadRegs(MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x03) | 0x02;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;

	case mpu925x_gyroDlpfBandWidth_8800hz:
		MPU925x_ReadRegs(MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x03) | 0x01;
		MPU925x_WriteReg(MPU925X_REG_CONFIG, data);
		break;
	}
}

// New sample rate = 1 kHz / (1 + divider)
void MPU925x_SetSampleRateDiv(uint8_t divider)
{
	MPU925x_WriteReg(MPU925X_REG_SMPLRT_DIV, divider);
}

MPU925x_IMUData_t MPU925x_ReadIMU(void)
{
	MPU925x_IMUData_t IMUData;
	uint8_t data[14];

	MPU925x_ReadRegs(MPU925X_REG_ACCEL_XOUT_H, data, 14);

	int16_t ax = ((int16_t) data[0] << 8) | data[1];
	int16_t ay = ((int16_t) data[2] << 8) | data[3];
	int16_t az = ((int16_t) data[4] << 8) | data[5];
	int16_t gx = ((int16_t) data[8] << 8) | data[9];
	int16_t gy = ((int16_t) data[10] << 8) | data[11];
	int16_t gz = ((int16_t) data[12] << 8) | data[13];

	IMUData.ax = ax / accelSensitivity;
	IMUData.ay = ay / accelSensitivity;
	IMUData.az = az / accelSensitivity;
	IMUData.gx = gx / gyroSensitivity;
	IMUData.gy = gy / gyroSensitivity;
	IMUData.gz = gz / gyroSensitivity;

	return IMUData;
}

void MPU925x_WriteReg(uint8_t adress, uint8_t data)
{
	LL_GPIO_ResetOutputPin(mpu925x.CS_GPIOx, mpu925x.CS_Pin);

	while (!(mpu925x.spiHandle->SR & SPI_SR_TXE));
	LL_SPI_TransmitData8(mpu925x.spiHandle, adress);
	while (!(mpu925x.spiHandle->SR & SPI_SR_RXNE));
	LL_SPI_ReceiveData8(mpu925x.spiHandle);						// Read out bogus data

	while (!(mpu925x.spiHandle->SR & SPI_SR_TXE));
	LL_SPI_TransmitData8(mpu925x.spiHandle, data);
	while (!(mpu925x.spiHandle->SR & SPI_SR_RXNE));
	LL_SPI_ReceiveData8(mpu925x.spiHandle);						// Read out bogus data

	LL_GPIO_SetOutputPin(mpu925x.CS_GPIOx, mpu925x.CS_Pin);
}

void MPU925x_ReadRegs(uint8_t address, uint8_t *data, uint8_t bytes)
{
	LL_GPIO_ResetOutputPin(mpu925x.CS_GPIOx, mpu925x.CS_Pin);

	while(!(LL_SPI_IsActiveFlag_TXE(mpu925x.spiHandle)));
	LL_SPI_TransmitData8(mpu925x.spiHandle, (address | 0x80));
	while(!(LL_SPI_IsActiveFlag_RXNE(mpu925x.spiHandle)));
	LL_SPI_ReceiveData8(mpu925x.spiHandle);							// Read out bogus data

	for(uint8_t i = 0; i < bytes; i++)
	{
		while(!(LL_SPI_IsActiveFlag_TXE(mpu925x.spiHandle)));
		LL_SPI_TransmitData8(mpu925x.spiHandle, 0x00);				// Send out 8 bits to read 8 more bits
		while(!(LL_SPI_IsActiveFlag_RXNE(mpu925x.spiHandle)));
		data[i] = LL_SPI_ReceiveData8(mpu925x.spiHandle);
	}

	LL_GPIO_SetOutputPin(mpu925x.CS_GPIOx, mpu925x.CS_Pin);
}


/*******************************************************************************
* END
*******************************************************************************/
