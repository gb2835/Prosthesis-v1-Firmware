/*******************************************************************************
*
* TITLE:	Driver for AMS AS5145B Magnetic Rotary Encoder
* AUTHOR:	Greg Berkeley
* RELEASE:	05/07/2024
*
* NOTES
* 1. This driver is based on
* 		- Product Document AS5145H/AS5145A/AS5145B
*			- Document Number: N/A
*			- Revision: v2-02
* 2. Unless otherwise specified, units are
* 		- Angles = degrees
* 2. #define AS5145B_NUMBER_OF_DEVICES must be updated to (at least) the number of devices used.
* 3. Only SSI functionality is used in this driver.
* 4. Minimum delays between clock edges are required for this device.
*    This driver is configured for the scenario where there is no clock pin (i.e. SPI) but a GPIO output pin instead.
*    Thus a delay function is used to generate the clock.
*
*******************************************************************************/

#include "as5145b.h"
#include <string.h>
#include "utilities.h"


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

typedef struct
{
	GPIO_TypeDef *DO_GPIOx;
	GPIO_TypeDef *CLK_GPIOx;
	GPIO_TypeDef *CSn_GPIOx;
	uint16_t DO_Pin;
	uint16_t CLK_Pin;
	uint16_t CSn_Pin;
	TIM_TypeDef *TIMx;
	uint8_t timerRateMHz;
	uint32_t isInit;
} Device_t;

static Device_t Device[AS5145B_NUMBER_OF_DEVICES];

static uint8_t ReadStatus(uint8_t deviceIndex);
static AS5145B_Data_t ReadData(uint8_t deviceIndex);
static inline void SetChipSelect(uint8_t deviceIndex);
static inline void ClearChipSelect(uint8_t deviceIndex);
static inline void RaiseClockEdge(uint8_t deviceIndex);
static inline void LowerClockEdge(uint8_t deviceIndex);
static inline uint8_t ReadDO_Pin(uint8_t deviceIndex);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

AS5145B_Error_e AS5145B_Init(uint8_t deviceIndex, AS5145B_Init_t *Device_Init)
{
	if(deviceIndex + 1 > AS5145B_NUMBER_OF_DEVICES)
		while(1);

	memcpy(&Device[deviceIndex], Device_Init, sizeof(AS5145B_Init_t));

	ClearChipSelect(deviceIndex);
	RaiseClockEdge(deviceIndex);

	uint8_t status = ReadStatus(deviceIndex);
	if((status & 0b111000) != 0b100000)
		return AS5145B_StatusError;

	Device[deviceIndex].isInit = 1;

	return AS5145B_NoError;
}

AS5145B_Data_t AS5145B_ReadData(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return ReadData(deviceIndex);
}

float AS5145B_ReadPosition(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	AS5145B_Data_t Data = ReadData(deviceIndex);
	return Data.position * AS5145B_RAW2DEG;
}

uint16_t AS5145B_ReadPosition_Raw(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	AS5145B_Data_t Data = ReadData(deviceIndex);
	return Data.position;
}

uint8_t AS5145B_ReadStatus(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return ReadStatus(deviceIndex);
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static uint8_t ReadStatus(uint8_t deviceIndex)
{
	AS5145B_Data_t Data = ReadData(deviceIndex);
	return Data.status;
}

static AS5145B_Data_t ReadData(uint8_t deviceIndex)
{
	SetChipSelect(deviceIndex);
	DelayUs(Device[deviceIndex].TIMx, Device[deviceIndex].timerRateMHz, 1);	// Delay of 500 ns minimum required for t_(CLK FE)

	AS5145B_Data_t Data;
	memset(&Data, 0, sizeof(Data));
	for(int i = 12-1; i >= 0; i--)
	{
		LowerClockEdge(deviceIndex);
		DelayUs(Device[deviceIndex].TIMx, Device[deviceIndex].timerRateMHz, 1);	// Delay of 500 ns minimum required for t_(CLK FE)
		RaiseClockEdge(deviceIndex);
		DelayUs(Device[deviceIndex].TIMx, Device[deviceIndex].timerRateMHz, 1);	// Delay of 500 ns minimum required for t_(CLK FE)

		Data.position |= ReadDO_Pin(deviceIndex) << i;
	}

	for(int i = 6-1; i >= 0; i--)
	{
		LowerClockEdge(deviceIndex);
		DelayUs(Device[deviceIndex].TIMx, Device[deviceIndex].timerRateMHz, 1);	// Delay of 500 ns minimum required for t_(CLK FE)
		RaiseClockEdge(deviceIndex);
		DelayUs(Device[deviceIndex].TIMx, Device[deviceIndex].timerRateMHz, 1);	// Delay of 500 ns minimum required for t_(CLK FE)

		Data.status |= ReadDO_Pin(deviceIndex) << i;
	}

	ClearChipSelect(deviceIndex);
	DelayUs(Device[deviceIndex].TIMx, Device[deviceIndex].timerRateMHz, 1);	// Delay of 500 ns minimum required for t_(CLK FE)

	return Data;
}

static inline void ClearChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_SetOutputPin(Device[deviceIndex].CSn_GPIOx, Device[deviceIndex].CSn_Pin);
}

static inline void SetChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_ResetOutputPin(Device[deviceIndex].CSn_GPIOx, Device[deviceIndex].CSn_Pin);
}

static inline void RaiseClockEdge(uint8_t deviceIndex)
{
	LL_GPIO_SetOutputPin(Device[deviceIndex].CLK_GPIOx, Device[deviceIndex].CLK_Pin);
}

static inline void LowerClockEdge(uint8_t deviceIndex)
{
	LL_GPIO_ResetOutputPin(Device[deviceIndex].CLK_GPIOx, Device[deviceIndex].CLK_Pin);
}

static inline uint8_t ReadDO_Pin(uint8_t deviceIndex)
{
	return LL_GPIO_IsInputPinSet(Device[deviceIndex].DO_GPIOx, Device[deviceIndex].DO_Pin) & 0x01;
}


/*******************************************************************************
* END
*******************************************************************************/
