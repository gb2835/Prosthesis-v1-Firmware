/*
 * mpu-9255.h
 *
 *  Created on: Mar 25, 2024
 *      Author: greg
 */

#ifndef INC_MPU_9255_H_
#define INC_MPU_9255_H_

// Includes
//#include <stdint.h>

// Identifiers
#define MPU9255_DEVICE_ID 0x73

// Registers
#define MPU9255_REG_TEMP_OUT_H	0X41
#define MPU9255_REG_TEMP_OUT_L	0X42
#define MPU9255_REG_WHO_AM_I	0X75

//
typedef struct
{
	float temp_C;
} MPU9255;

// Initialization
uint8_t MPU9255_Init(MPU9255 *dev, SPI_HandleTypeDef *SPI_Handle);

// Data Acquisition
HAL_StatusTypeDef MPU9255_ReadTemp(SPI_HandleTypeDef *SPI_Handle);

// Low-Level Functions
HAL_StatusTypeDef MPU9255_ReadReg(MPU9255 *dev, uint8_t *reg, uint8_t *data);

#endif /* INC_MPU_9255_H_ */
