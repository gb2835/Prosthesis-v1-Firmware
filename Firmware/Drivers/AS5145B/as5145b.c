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
* 2. #define AS5145B_NUMBER_OF_DEVICES must be updated to (at least) the number
*    of devices used.
* 3. Only SSI functionality is used in this driver.
* 4. Minimum delays between clock edges are required for this device. This
*    driver is configured for the scenario where there is no clock pin (i.e.
*    SPI) but a GPIO output pin instead. Thus a delay function is used to
*    generate the required delays.
*
*******************************************************************************/

#include "as5145b.h"
#include <string.h>
#include "utilities.h"

#include <assert.h>
/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define TIMERX				TIM6 // is this the right way to do this??
#define TIMERX_RATE_MHZ		10

typedef struct
{
	GPIO_TypeDef *DO_GPIOx;
	GPIO_TypeDef *CLK_GPIOx;
	GPIO_TypeDef *CSn_GPIOx;
	uint16_t DO_Pin;
	uint16_t CLK_Pin;
	uint16_t CSn_Pin;
	uint8_t isInit;
} Device_t;

static Device_t Device[AS5145B_NUMBER_OF_DEVICES];

static inline void SetChipSelect(uint8_t deviceIndex);
static inline void ClearChipSelect(uint8_t deviceIndex);
static inline void RaiseClockEdge(uint8_t deviceIndex);
static inline void LowerClockEdge(uint8_t deviceIndex);
static inline uint8_t ReadDO_Pin(uint8_t deviceIndex);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void AS5145B_Init(uint8_t deviceIndex, AS5145B_Init_t *Device_Init)
{
	if(deviceIndex++ > AS5145B_NUMBER_OF_DEVICES)
		__NOP(); // add assert??

	memcpy(&Device[deviceIndex], &Device_Init[deviceIndex], sizeof(Device_Init[deviceIndex]));

	ClearChipSelect(deviceIndex);
	RaiseClockEdge(deviceIndex);

	Device[deviceIndex].isInit = 1;
}

AS5145B_Data_t AS5145B_ReadData(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	SetChipSelect(deviceIndex);
	DelayUs(TIMERX, 1, TIMERX_RATE_MHZ);	// Delay of 500 ns minimum required for t_(CLK FE)

	// Read angular position from first 12 bits (MSB first)
	AS5145B_Data_t Data;
	memset(&Data, 0, sizeof(Data)); // debug check this??
	for(int i = 12-1; i >= 0; i--)
	{
		LowerClockEdge(deviceIndex);
		DelayUs(TIMERX, 1, TIMERX_RATE_MHZ);	// Delay of 500 ns minimum required for T_(CLK/2)
		RaiseClockEdge(deviceIndex);
		DelayUs(TIMERX, 1, TIMERX_RATE_MHZ);	// Delay of 500 ns minimum required for T_(CLK/2)
		uint8_t temp = ReadDO_Pin(deviceIndex);
		Data.position |= (temp) << i;
	}

	// Read remaining 6 status bits (MSB first)
	for(int i = 6-1; i >= 0; i--)
	{
		LowerClockEdge(deviceIndex);
		DelayUs(TIMERX, 1, TIMERX_RATE_MHZ);	// Delay of 500 ns minimum required for T_(CLK/2)
		RaiseClockEdge(deviceIndex);
		DelayUs(TIMERX, 1, TIMERX_RATE_MHZ);	// Delay of 500 ns minimum required for T_(CLK/2)
		uint8_t temp = ReadDO_Pin(deviceIndex);
		Data.status  |= (temp) << i;
	}

	ClearChipSelect(deviceIndex);
	DelayUs(TIMERX, 1, TIMERX_RATE_MHZ);	// Delay of 500 ns minimum required for t_(CSn)

	return Data;
}

float AS5145B_ReadPosition(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	AS5145B_Data_t Data = AS5145B_ReadData(deviceIndex);
	return (float) Data.position * AS5145B_RAW2DEG; // (float)??
}

uint16_t AS5145B_ReadPosition_Raw(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	AS5145B_Data_t Data = AS5145B_ReadData(deviceIndex);
	return Data.position;
}

uint8_t AS5145B_ReadStatus(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	AS5145B_Data_t Data = AS5145B_ReadData(deviceIndex);
	return Data.status;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

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
	LL_GPIO_SetOutputPin(Device[deviceIndex].CLK_GPIOx, Device[deviceIndex].CLK_Pin);
}

static inline uint8_t ReadDO_Pin(uint8_t deviceIndex)
{
	return LL_GPIO_IsInputPinSet(Device[deviceIndex].DO_GPIOx, Device[deviceIndex].DO_Pin) & 0x01; // 0x01??
}


/*******************************************************************************
* END
*******************************************************************************/
