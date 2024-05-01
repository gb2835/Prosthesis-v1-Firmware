/*******************************************************************************
 *
 * TITLE:   as5145b.c
 * AUTHOR:  Greg Berkeley
 * RELEASE: XX/XX/XXXX
 *
 * NOTES:
 * 1. See header file for more information.
 *
 ******************************************************************************/

// Include header files
#include "as5145b.h"

// Declare global variables
// None


/*******************************************************************************
 * INITIALIZATION FUNCTIONS
 ******************************************************************************/

// None


/*******************************************************************************
 * APPLICATION FUNCTIONS
 ******************************************************************************/

// None
int AS5145B_GetPosition (void)
{
	int DataPrecision = 12;
	int tempPosition  = 0;
	int i             = 0;
	uint8_t tempRead  = 0;
	uint8_t Flags[6];

	LL_GPIO_ResetOutputPin(MagEnc_CSn_GPIO_PORT, MagEnc_CSn_PIN);
	delay_us(1);
	// Sensor feeds out position MSB first
	for (i = DataPrecision - 1; i >= 0; i--)
	{
		LL_GPIO_ResetOutputPin(MagEnc_CLK_GPIO_PORT, MagEnc_CLK_PIN);
		delay_us(1);

		LL_GPIO_SetOutputPin(MagEnc_CLK_GPIO_PORT, MagEnc_CLK_PIN);
		delay_us(1);

		tempRead = LL_GPIO_IsInputPinSet(ENC2_DATA_GPIO_PORT, ENC2_DATA_PIN)
				& 0x01;
		tempPosition |= (tempRead) << i;
	}

	for (i = 0; i < 6; i++)
	{
		LL_GPIO_ResetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(1);

		LL_GPIO_SetOutputPin(ENC2_SCLK_GPIO_PORT, ENC2_SCLK_PIN);
		delay_us(1);

		tempRead = LL_GPIO_IsInputPinSet(ENC2_DATA_GPIO_PORT, ENC2_DATA_PIN)
				& 0x01;
		Flags[i] |= (tempRead) << i;
	}

	LL_GPIO_SetOutputPin(ENC2_CS_GPIO_PORT, ENC2_CS_PIN);

	return tempPosition;
}


/*******************************************************************************
 * LOW-LEVEL FUNCTIONS
 ******************************************************************************/

// None


/*******************************************************************************
 * END
 ******************************************************************************/
