/*******************************************************************************
 *
 * TITLE   as5145b.h
 * AUTHOR  Greg Berkeley
 * RELEASE 05/07/2024
 *
 * NOTES
 * 1. Unless otherwise specified, the following references are used:
 * 		DS = Datasheet    (Title: Product Document AS5145H/AS5145A/AS5145B,
 *                         Document Number: N/A,
 *                         Revision: v2-02)
 * 2. Only SSI functionality is used in this driver to read data.
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

#ifndef INC_AS5145B_H_
#define INC_AS5145B_H_

// Include header files
#include "stm32l4xx_ll_gpio.h"		// Provides GPIO functionality
#include <string.h>					// Provides memcpy()


/*******************************************************************************
 * DEVICE ADDRESSES
 ******************************************************************************/

// #define None		(Section/Page X.X of DS/RM)


/*******************************************************************************
 * DEVICE IDENTIFIERS
 ******************************************************************************/

// #define None		(Section/Page X.X of DS/RM)


/*******************************************************************************
 * DEVICE REGISTERS (Section X.X of DS/RM)
 ******************************************************************************/

// #define None


/*******************************************************************************
 * DEVICE DEFINITIONS
 ******************************************************************************/

#define AS5145B_ADC2DEG	360/4096	// Convert angular position from ADC to degrees (12 bit device, 2^12 = 4096)


/*******************************************************************************
 * DRIVER DEFINITIONS
 ******************************************************************************/

// Device initialization handle (to be used in main)
typedef struct
{
	GPIO_TypeDef	*CSn_GPIOx;		// Chip select GPIO port
	GPIO_TypeDef	*CLK_GPIOx;		// Clock GPIO port
	GPIO_TypeDef	*DO_GPIOx;		// Digital output GPIO port
	uint16_t		CSn_Pin;		// Chip select pin number
	uint16_t		CLK_Pin;		// Clock pin number
	uint16_t		DO_Pin;			// Digital output pin number
} AS5145B_Init_t;

// Device handle (to be used only in this driver, static??)
typedef struct
{
	GPIO_TypeDef	*CSn_GPIOx;		// Chip select GPIO port
	GPIO_TypeDef	*CLK_GPIOx;		// Clock GPIO port
	GPIO_TypeDef	*DO_GPIOx;		// Digital output GPIO port
	uint16_t		CSn_Pin;		// Chip select pin number
	uint16_t		CLK_Pin;		// Clock pin number
	uint16_t		DO_Pin;			// Digital output pin number
} AS5145B_t;

// Data structure for digital output of device
struct AS5145B_Data_s
{
	int16_t pos;		// First 12 bits of data is angular position
	uint8_t  status;	// Last 6 bits of data is status
};


/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

// Initialization functions
void AS5145B_Init ( AS5145B_Init_t *dev );	// Initialize device

// Application functions
struct AS5145B_Data_s AS5145B_ReadData (void);	// Read data
float AS5145B_ReadPosition_Deg (void);			// Read angular position in degrees
uint8_t AS5145B_ReadStatus (void);				// Read status bits

// Low-Level functions
// None

// Other functions
void AS5145B_Delay_500ns (void);	// Delay for approximately 500 nanoseconds for 80 MHz SYSCLK


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_AS5145B_H_ */
