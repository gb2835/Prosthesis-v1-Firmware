/*******************************************************************************
 *
 * TITLE   Driver for AMS AS5145B Magnetic Rotary Encoder
 * AUTHOR  Greg Berkeley
 * RELEASE 05/07/2024
 *
 * NOTES
 * 1. This driver is based on
 * 		- Product Document AS5145H/AS5145A/AS5145B
 *			- Document Number: N/A
 *			- Revision: v2-02
 * 2. Only SSI functionality is used in this driver.
 * 3. The DS requires minimum delays in the clock frequency for the device. This
 *    driver is configured for the scenario where there is no clock pin (i.e.
 *    SPI) but a GPIO output pin instead. The delay function Delay_500ns is thus
 *    used to generate the clock delays. The delays are configured according to
 *    Figure 10 and Figure 13 in DS for 80 MHz SYSCLK. For a slower SYSCLK the
 *    delays will be longer. Since the delays needed for the device are minimums,
 *    slower delays do not affect functionality but do however affect MCU
 *    performance due to longer delays.
 * 4. The Delay_500ns function was generated based on the following process:
 *    First, an oscope was used to measure how fast a pin goes HIGH then LOW.
 *    Then, the for loops were placed between the HIGH and LOW and tuned to get
 *    an extra 500 nanoseconds. Thus, it is a very approximate function for
 *    generating 500 nanosecond delays.
 *
 ******************************************************************************/

#include "as5145b.h"
#include <string.h>


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
} Device_t;

static Device_t Device;

static void AS5145B_Delay_500ns(void);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void AS5145B_Init(AS5145B_t *AS5145B_Init)
{
	memcpy( &Device, AS5145B_Init, sizeof(AS5145B_t) );

	LL_GPIO_SetOutputPin(Device.CSn_GPIOx, Device.CSn_Pin);
	LL_GPIO_SetOutputPin(Device.CLK_GPIOx, Device.CLK_Pin);
}

AS5145B_Data_t AS5145B_ReadData(void)
{
	LL_GPIO_ResetOutputPin(Device.CSn_GPIOx, Device.CSn_Pin);
	AS5145B_Delay_500ns();											// Delay of 500 ns minimum required for t_(CLK FE)

	// Read angular position from first 12 bits (MSB first)
	AS5145B_Data_t Data;
	memset(&Data, 0, sizeof(Data)); // debug check this??
	for(int i = 12-1; i >= 0; i--)
	{
		LL_GPIO_ResetOutputPin(Device.CLK_GPIOx, Device.CLK_Pin);
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2)
		LL_GPIO_SetOutputPin(Device.CLK_GPIOx, Device.CLK_Pin);
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2)
		uint8_t temp  = LL_GPIO_IsInputPinSet(Device.DO_GPIOx, Device.DO_Pin) & 0x01;
		Data.position |= (temp) << i;
	}

	// Read remaining 6 status bits (MSB first)
	for(int i = 6-1; i >= 0; i--)
	{
		LL_GPIO_ResetOutputPin(Device.CLK_GPIOx, Device.CLK_Pin);
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2)
		LL_GPIO_SetOutputPin(Device.CLK_GPIOx, Device.CLK_Pin);
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2)
		uint8_t temp  = LL_GPIO_IsInputPinSet(Device.DO_GPIOx, Device.DO_Pin) & 0x01;
		Data.status  |= (temp) << i;
	}

	LL_GPIO_SetOutputPin(Device.CSn_GPIOx, Device.CSn_Pin);
	AS5145B_Delay_500ns();											// Delay of 500 ns minimum required for t_(CSn)

	return Data;
}

float AS5145B_ReadPosition(void)
{
	AS5145B_Data_t Data = AS5145B_ReadData();
	float position = (float) Data.position * AS5145B_RAW2DEG;
	return position;
}

uint8_t AS5145B_ReadStatus(void)
{
	AS5145B_Data_t Data = AS5145B_ReadData();
	uint8_t status = Data.status;
	return status;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

// See NOTES at the top of this file for more information
static void AS5145B_Delay_500ns(void)
{
	for(uint8_t i = 0; i < 2; i++)
	{
		for(uint8_t j = 0; j < 3; j++)
			__NOP();
	}
}


/*******************************************************************************
* END
*******************************************************************************/
