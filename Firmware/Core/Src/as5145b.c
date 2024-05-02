/*******************************************************************************
 *
 * TITLE:   as5145b.c
 * AUTHOR:  Greg Berkeley
 * RELEASE: XX/XX/XXXX
 *
 * NOTES:
 * 1. None.
 *
 ******************************************************************************/

// Include header files
#include "as5145b.h"

// Declare global variables
// None


/*******************************************************************************
 * APPLICATION FUNCTIONS
 ******************************************************************************/

// Get angular position in degrees
uint16_t AS5145B_ReadPosition_Deg ( Enc_t *dev )
{
	// Declare variables
	uint8_t  DataPrecision = 12;	// 12 bit device
	uint16_t pos           = 0;		// Angular position
	uint8_t  Flags[6];				// ??

	// Enable CSn pin
	LL_GPIO_ResetOutputPin( dev->CSn_GPIOx, dev->CSn_Pin );
//	delay_us(1);

	// Get angular position in ADC (sensor feeds out position MSB first)
	for ( int i = DataPrecision - 1; i >= 0; i-- )
	{
		LL_GPIO_ResetOutputPin( dev->CLK_GPIOx, dev->CLK_Pin );						// Set clock low
//		delay_us(1);
		LL_GPIO_SetOutputPin( dev->CLK_GPIOx, dev->CLK_Pin );						// Set clock high
//		delay_us(1);
		uint8_t temp  = LL_GPIO_IsInputPinSet( dev->DO_GPIOx, dev->DO_Pin ) & 0x01;	// Read data bit
		pos          |= (temp) << i;												// Assign and shift bit
	}

	// Read/clear status bits
	for ( int i = 0; i < 6; i++ )
	{
		LL_GPIO_ResetOutputPin( dev->CLK_GPIOx, dev->CLK_Pin );						// Set clock low
//		delay_us(1);
		LL_GPIO_SetOutputPin( dev->CLK_GPIOx, dev->CLK_Pin );						// Set clock high
//		delay_us(1);
		uint8_t temp  = LL_GPIO_IsInputPinSet( dev->DO_GPIOx, dev->DO_Pin ) & 0x01;	// Read data bit
		Flags[i]     |= (temp) << i;												// Assign and shift bit
	}

	// Disable CSn pin
	LL_GPIO_SetOutputPin( dev->CSn_GPIOx, dev->CSn_Pin );

	// Convert from ADC to degrees
	pos *= ADC2DEG;

	// Return
	return pos;
}


/*******************************************************************************
 * END
 ******************************************************************************/
