/*******************************************************************************
 *
 * TITLE   as5145b.c
 * AUTHOR  Greg Berkeley
 * RELEASE 05/07/2024
 *
 * NOTES
 * 1. See header file for more information.
 *
 ******************************************************************************/

// Include header files
#include "as5145b.h"

// Declare global variables
static AS5145B_t AS5145B;	// Device handle (static to provide ownership to this driver)


/*******************************************************************************
 * INITIALIZATION FUNCTIONS
 ******************************************************************************/

// Initialize device
void AS5145B_Init ( AS5145B_Init_t *AS5145B_Init )
{
	// Copy memory of device initialization handle to device handle (provides ownership of the device handle to this driver)
	memcpy( &AS5145B, AS5145B_Init, sizeof(AS5145B_Init_t) );

	// Initialize pins
	LL_GPIO_SetOutputPin( AS5145B.CSn_GPIOx, AS5145B.CSn_Pin );		// Chip select pin initially high (Figure 13 in DS)
	LL_GPIO_SetOutputPin( AS5145B.CLK_GPIOx, AS5145B.CLK_Pin );		// Clock pin initially high (Figure 13 in DS)
}


/*******************************************************************************
 * APPLICATION FUNCTIONS
 ******************************************************************************/

// Read data (first 12 bits = position, remaining 6 bits = status, MSB first, Figure 13 in DS)
// @param posBias Amount of bias to be removed from angular position data in ADC
struct AS5145B_Data_s AS5145B_ReadData (void)
{
	// Declare variables
	struct AS5145B_Data_s data;		// Data structure

	// Declare initialized variables
	data.pos    = 0;	// Angular position
	data.status = 0;	// Status of device

	// Enable chip select pin
	LL_GPIO_ResetOutputPin( AS5145B.CSn_GPIOx, AS5145B.CSn_Pin );	// Chip select pin is active low
	AS5145B_Delay_500ns();											// Delay of 500 ns minimum required for t_(CLK FE) (Figure 10 and Figure 13 in DS)

	// Read angular position in ADC from first 12 bits (MSB first)
	for ( int i = 12-1; i >= 0; i-- )
	{
		LL_GPIO_ResetOutputPin( AS5145B.CLK_GPIOx, AS5145B.CLK_Pin );						// Set clock low
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2) (Figure 10 and Figure 13 in DS)
		LL_GPIO_SetOutputPin( AS5145B.CLK_GPIOx, AS5145B.CLK_Pin );							// Set clock high
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2) (Figure 10 and Figure 13 in DS)
		uint8_t temp  = LL_GPIO_IsInputPinSet( AS5145B.DO_GPIOx, AS5145B.DO_Pin ) & 0x01;	// Read data bit
		data.pos     |= (temp) << i;														// Assign and shift bit
	}

	// Read remaining 6 status bits (MSB first)
	for ( int i = 6-1; i >= 0; i-- )
	{
		LL_GPIO_ResetOutputPin( AS5145B.CLK_GPIOx, AS5145B.CLK_Pin );						// Set clock low
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2) (Figure 10 and Figure 13 in DS)
		LL_GPIO_SetOutputPin( AS5145B.CLK_GPIOx, AS5145B.CLK_Pin );							// Set clock high
		AS5145B_Delay_500ns();																// Delay of 500 ns minimum required for T_(CLK/2) (Figure 10 and Figure 13 in DS)
		uint8_t temp  = LL_GPIO_IsInputPinSet( AS5145B.DO_GPIOx, AS5145B.DO_Pin ) & 0x01;	// Read data bit
		data.status  |= (temp) << i;														// Assign and shift bit
	}

	// Disable Chip select pin
	LL_GPIO_SetOutputPin( AS5145B.CSn_GPIOx, AS5145B.CSn_Pin );		// Chip select pin is inactive high
	AS5145B_Delay_500ns();											// Delay of 500 ns minimum required for t_(CSn) (Figure 10 and Figure 13 in DS)

	// Return
	return data;
}

// Read angular position in degrees
// @param posBias	Amount of bias to be removed from angular position data in ADC
float AS5145B_ReadPosition_Deg (void)
{
	// Declare variables
	struct AS5145B_Data_s data;		// Data structure

	// Read data and separate angular position
	data          = AS5145B_ReadData();		// Read data
	int16_t temp = data.pos;						// Separate angular position

	// Convert angular position from ADC to degrees
	float pos = (float) temp*AS5145B_ADC2DEG;

	// Return
	return pos;
}

// Read status bits
uint8_t AS5145B_ReadStatus (void)
{
	// Declare variables
	struct AS5145B_Data_s data;		// Data structure

	// Read data and separate status
	data           = AS5145B_ReadData();	// Read data
	uint8_t status = data.status;			// Separate status

	// Return
	return status;
}


/*******************************************************************************
 * LOW-LEVEL FUNCTIONS
 ******************************************************************************/

// None


/*******************************************************************************
 * OTHER FUNCTIONS
 ******************************************************************************/

// Delay of approximately 500 nanoseconds for 80 MHz SYSCLK
void AS5145B_Delay_500ns (void)
{
	for ( uint8_t i = 0; i < 2; i++ )
	{
		for( uint8_t j = 0; j < 3; j++ )
		 __NOP();
	}
}


/*******************************************************************************
 * END
 ******************************************************************************/
