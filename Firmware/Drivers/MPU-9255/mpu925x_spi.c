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
* 2. #define NUMBER_OF_DEVICES must be updated to (at least) the number of
*    devices used.
*
*******************************************************************************/

#include "mpu925x_spi.h"
#include "stm32l4xx_ll_gpio.h"
#include <string.h>


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define NUMBER_OF_DEVICES	1

typedef struct
{
	SPI_TypeDef *SPI_Handle;
	GPIO_TypeDef *CS_GPIOx;
	uint16_t csPin;
	uint8_t isInit;
} Device_t;

static float accelSensitivity = MPU925X_ACCEL_SENSITIVITY_2G;	// ±2g is default
static float gyroSensitivity = MPU925X_GYRO_SENSITIVITY_250DPS;	// ±250°/s is default
static Device_t Device[NUMBER_OF_DEVICES];

static inline void ClearChipSelect(uint8_t deviceIndex);
static inline void SetChipSelect(uint8_t deviceIndex);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

uint8_t MPU925x_Init(uint8_t deviceIndex, MPU925x_t *Device_Init)
{
	if(deviceIndex++ > NUMBER_OF_DEVICES)
		__NOP(); // add assert??

	memcpy(&Device[deviceIndex], &Device_Init[deviceIndex], sizeof(MPU925x_t));

	ClearChipSelect(deviceIndex);

	uint8_t whoAmI;
	MPU925x_ReadRegs(deviceIndex, MPU925X_REG_WHO_AM_I, &whoAmI, sizeof(whoAmI));
	if((whoAmI != MPU9250_DEVICE_ID) && (whoAmI != MPU9255_DEVICE_ID))
		return MPU925x_WhoAmI_Error;

	Device[deviceIndex].isInit = 1;

	return MPU925x_NoError;
}

void MPU925x_SetAccelSensitivity(uint8_t deviceIndex, MPU925x_AccelSensitivity_t sensitivity)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t data;
	switch(sensitivity)
	{
	case MPU925x_AccelSensitivity_2g:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = data & ~0x18;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_2G;
		break;

	case MPU925x_AccelSensitivity_4g:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x08;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_4G;
		break;

	case MPU925x_AccelSensitivity_8g:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x10;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_8G;
		break;

	case MPU925x_AccelSensitivity_16g:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x18;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG, data);
		accelSensitivity = MPU925X_ACCEL_SENSITIVITY_16G;
		break;
	}
}

void MPU925x_SetGyroSensitivity(uint8_t deviceIndex, MPU925x_GyroSensitivity_t sensitivity)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t data;
	switch(sensitivity)
	{
	case MPU925x_GyroSensitivity_250dps:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = data & ~0x18;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_250DPS;
		break;

	case MPU925x_GyroSensitivity_500dps:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x08;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_500DPS;
		break;

	case MPU925x_GyroSensitivity_1000dps:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x10;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_1000DPS;
		break;

	case MPU925x_GyroSensitivity_2000dps:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x18) | 0x18;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_GYRO_CONFIG, data);
		gyroSensitivity = MPU925X_GYRO_SENSITIVITY_2000DPS;
		break;
	}
}

void MPU925x_SetAccelDlpfBandwidth(uint8_t deviceIndex, MPU925x_AccelDLPF_BandWidth_t option)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t data;
	switch (option)
	{
	case MPU925x_AccelDLPF_BandWidth_5hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x06;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case MPU925x_AccelDLPF_BandWidth_10hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x05;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case MPU925x_AccelDLPF_BandWidth_20hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x04;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case MPU925x_AccelDLPF_BandWidth_41hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x03;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case MPU925x_AccelDLPF_BandWidth_92hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x02;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case MPU925x_AccelDLPF_BandWidth_184hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x01;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case MPU925x_AccelDLPF_BandWidth_460hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = data & ~0x0F;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;

	case MPU925x_AccelDLPF_BandWidth_1130hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, &data, 1);
		data = (data & ~0x0F) | 0x08;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_ACCEL_CONFIG_2, data);
		break;
	}
}

void MPU925x_SetGyroDlpfBandwidth(uint8_t deviceIndex, MPU925x_GyroDLPF_BandWidth_t option)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t data;
	switch (option)
	{
	case MPU925x_GyroDLPF_BandWidth_5hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x06;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;

	case MPU925x_GyroDLPF_BandWidth_10hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x05;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;

	case MPU925x_GyroDLPF_BandWidth_20hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x04;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;

	case MPU925x_GyroDLPF_BandWidth_41hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x03;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;

	case MPU925x_GyroDLPF_BandWidth_92hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x02;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;

	case MPU925x_GyroDLPF_BandWidth_184hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = (data & ~0x07) | 0x01;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;

	case MPU925x_GyroDLPF_BandWidth_250hz:
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_CONFIG, &data, 1);
		data = data & ~0x07;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data); // reg config or reg gyro config??
		break;

	case MPU925x_GyroDLPF_BandWidth_3600hz: // 3600??
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x03) | 0x02;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;

	case MPU925x_GyroDLPF_BandWidth_8800hz: // 8800??
		MPU925x_ReadRegs(deviceIndex, MPU925X_REG_GYRO_CONFIG, &data, 1);
		data = (data & ~0x03) | 0x01;
		MPU925x_WriteReg(deviceIndex, MPU925X_REG_CONFIG, data);
		break;
	}
}

// New sample rate = 1 kHz / (1 + divider)
void MPU925x_SetSampleRateDiv(uint8_t deviceIndex, uint8_t divider)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	MPU925x_WriteReg(deviceIndex, MPU925X_REG_SMPLRT_DIV, divider);
}

MPU925x_IMU_Data_t MPU925x_ReadIMU(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	MPU925x_IMU_Data_t IMU_Data;
	uint8_t data[14];
	MPU925x_ReadRegs(deviceIndex, MPU925X_REG_ACCEL_XOUT_H, data, 14);

	int16_t ax = ((int16_t) data[0] << 8) | data[1];
	int16_t ay = ((int16_t) data[2] << 8) | data[3];
	int16_t az = ((int16_t) data[4] << 8) | data[5];
	int16_t gx = ((int16_t) data[8] << 8) | data[9];
	int16_t gy = ((int16_t) data[10] << 8) | data[11];
	int16_t gz = ((int16_t) data[12] << 8) | data[13];

	IMU_Data.ax = ax / accelSensitivity;
	IMU_Data.ay = ay / accelSensitivity;
	IMU_Data.az = az / accelSensitivity;
	IMU_Data.gx = gx / gyroSensitivity;
	IMU_Data.gy = gy / gyroSensitivity;
	IMU_Data.gz = gz / gyroSensitivity;

	return IMU_Data;
}

void MPU925x_WriteReg(uint8_t deviceIndex, uint8_t adress, uint8_t data)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	SetChipSelect(deviceIndex);

	while (!(Device[deviceIndex].SPI_Handle->SR & SPI_SR_TXE));
	LL_SPI_TransmitData8(Device[deviceIndex].SPI_Handle, adress);
	while (!(Device[deviceIndex].SPI_Handle->SR & SPI_SR_RXNE));
	LL_SPI_ReceiveData8(Device[deviceIndex].SPI_Handle);						// Read out bogus data

	while (!(Device[deviceIndex].SPI_Handle->SR & SPI_SR_TXE));
	LL_SPI_TransmitData8(Device[deviceIndex].SPI_Handle, data);
	while (!(Device[deviceIndex].SPI_Handle->SR & SPI_SR_RXNE));
	LL_SPI_ReceiveData8(Device[deviceIndex].SPI_Handle);						// Read out bogus data

	ClearChipSelect(deviceIndex);
}

void MPU925x_ReadRegs(uint8_t deviceIndex, uint8_t startAddress, uint8_t *data, uint8_t bytes)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	SetChipSelect(deviceIndex);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPI_Handle)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPI_Handle, (startAddress | 0x80));
	while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPI_Handle)));
	LL_SPI_ReceiveData8(Device[deviceIndex].SPI_Handle);							// Read out bogus data

	for(uint8_t i = 0; i < bytes; i++)
	{
		while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPI_Handle)));
		LL_SPI_TransmitData8(Device[deviceIndex].SPI_Handle, 0x00);				// Send out 8 bits to read 8 more bits
		while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPI_Handle)));
		data[i] = LL_SPI_ReceiveData8(Device[deviceIndex].SPI_Handle);
	}

	ClearChipSelect(deviceIndex);
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static inline void ClearChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_SetOutputPin(Device[deviceIndex].CS_GPIOx, Device[deviceIndex].csPin);
}

static inline void SetChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_ResetOutputPin(Device[deviceIndex].CS_GPIOx, Device[deviceIndex].csPin);
}


/*******************************************************************************
* END
*******************************************************************************/
