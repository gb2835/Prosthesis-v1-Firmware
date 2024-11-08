/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_MCP25625_H_
#define INC_MCP25625_H_

#include <stdint.h>
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"

// how can error handler tell diff between this and other enums??
typedef enum
{
	mcp25625_ok,
	mcp25625_initError,
} MCP25625_Errors_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t BRP	:6;
		enum
		{
			length1xT_Q,
			length2xT_Q,
			length3xT_Q,
			length4xT_Q
		} SJW :2;
	} Bits;
} CNF1_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t PRSEG	:3;
		uint8_t PHSEG1	:3;
		enum
		{
			busSampledOnceAtSamplePoint,
			busSampledThreeTimesAtSamePoint
		} SAM :1;
		enum
		{
			ps2LengthIsGreaterOfPs1AndIpt,
			ps2LengthDeterminedByCNF3
		} BLTMODE :1;
	} Bits;
} CNF2_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t PHSEG2			:3;
		uint8_t unimplemented	:3;
		enum
		{
			wakeUpFilterIsDisabled,
			wakeUpFilterIsEnabled
		} WAKFIL :1;
		enum
		{
			clockoutPinIsEnabledForclockOutFunction,
			clockoutPinIsEnabledForSofSignal,
		} SOF :1;
	} Bits;
} CNF3_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		enum
		{
			clockoutDiv1,
			clockoutDiv2,
			clockoutDiv4,
			clockoutDiv8
		} CLKPRE :2;
		enum
		{
			clockoutDisabled,
			clockoutEnabled
		} CLKEN :1;
		enum
		{
			oneShotModeDisabled,
			oneShotModeEnabled
		} OSM :1;
		enum
		{
			abortAllTransmissions,
			abortAllPendingTransmitBuffers
		} ABAT :1;
		enum
		{
			normalOperationMode,
			sleepMode,
			loopBackMode,
			listenOnlyMode,
			configurationMode
		} REQOP :3;
	} Bits;
} MCP25625_CANCTRL_Reg_t;

typedef struct
{
	SPI_TypeDef *SPIx;
	GPIO_TypeDef *CS_Port;
	uint16_t csPin;
	MCP25625_CANCTRL_Reg_t CANCTRL_Reg;
	CNF1_Reg_t CNF1_Reg;
	CNF2_Reg_t CNF2_Reg;
	CNF3_Reg_t CNF3_Reg;
} MCP25625_Inits_t;

uint8_t MCP25625_Init(MCP25625_Inits_t *Device_Init, uint16_t id, uint8_t nBytes);
uint8_t MCP25625_LoadTxBufferAtD0(uint8_t *data, uint8_t nBytes);
uint8_t MCP25625_ReadRxBufferAtD0(uint8_t *data, uint8_t nBytes);
uint8_t MCP25625_ReadRxBufferAtSIDH(uint8_t *data, uint8_t nDataBytes);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_MCP25625_H_ */
