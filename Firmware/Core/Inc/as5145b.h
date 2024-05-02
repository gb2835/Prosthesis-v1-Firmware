/*******************************************************************************
 *
 * TITLE:   as5145b.h
 * AUTHOR:  Greg Berkeley
 * RELEASE: XX/XX/XXXX
 *
 * NOTES:
 * 1. Unless otherwise specified, the following references are used:
 * 		DS = Datasheet    (Product Document AS5145H/AS5145A/AS5145B,
 *                         Document Number: N/A,
 *                         Revision: v2-02)
 *
 ******************************************************************************/

#ifndef INC_AS5145B_H_
#define INC_AS5145B_H_

// Include header files
#include "stm32l4xx_ll_gpio.h"		// Needed for GPIO functionality


/*******************************************************************************
 * DEFINITIONS
 ******************************************************************************/

#define ADC2DEG	360/4096	// Convert angular position from ADC to degrees (12 bit device, 2^12 = 4096)


/*******************************************************************************
 * STRUCTURES
 ******************************************************************************/

// Device typedef structure
typedef struct
{
	GPIO_TypeDef	*CSn_GPIOx;
	GPIO_TypeDef	*CLK_GPIOx;
	GPIO_TypeDef	*DO_GPIOx;
	uint16_t		CSn_Pin;
	uint16_t		CLK_Pin;
	uint16_t		DO_Pin;
} Enc_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

// Application functions
uint16_t AS5145B_ReadPosition_Deg ( Enc_t *dev );		// Get angular position in degrees


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_AS5145B_H_ */
