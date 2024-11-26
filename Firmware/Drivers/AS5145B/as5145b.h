/*******************************************************************************
 *
 * See source file for more information.
 *
 ******************************************************************************/

#ifndef INC_AS5145B_H_
#define INC_AS5145B_H_

#include "stm32l4xx_ll_gpio.h"

#define AS5145B_RAW2DEG				360/4096.0f

typedef struct
{
	GPIO_TypeDef	*DO_GPIOx;
	GPIO_TypeDef	*CLK_GPIOx;
	GPIO_TypeDef	*CSn_GPIOx;
	uint16_t		DO_Pin;
	uint16_t		CLK_Pin;
	uint16_t		CSn_Pin;
} AS5145B_t;

typedef struct
{
	int16_t	position;
	uint8_t	status;
} AS5145B_Data_t;

void AS5145B_Init(uint8_t deviceIndex, AS5145B_t *Device_Init);
AS5145B_Data_t AS5145B_ReadData(uint8_t deviceIndex);
float AS5145B_ReadPosition(uint8_t deviceIndex);
uint16_t AS5145B_ReadPosition_Raw(uint8_t deviceIndex);
uint8_t AS5145B_ReadStatus(uint8_t deviceIndex);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_AS5145B_H_ */
