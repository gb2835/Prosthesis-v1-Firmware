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
	mcp25625_noError,
	mcp25625_resetError,
	mcp25625_configError,
	mcp25625_canCtrlError
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
} MCP25625_CNF1_Reg_t;

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
} MCP25625_CNF2_Reg_t;

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
			clockoutPinIsEnabledForClockOutFunction,
			clockoutPinIsEnabledForSofSignal
		} SOF :1;
	} Bits;
} MCP25625_CNF3_Reg_t;

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

typedef uint8_t MCP25625_RXBxSIDH_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t EID 			:2;
		uint8_t unimplemented2	:1;
		enum
		{
			receiveStandardId,
			receiveExtendedId
		} IDE	:1;
		enum
		{
			standardFrameReceived,
			standardRemoteTransmitRequestReceived
		} SRR	:1;
		uint8_t SID	:3;
	} Bits;
} MCP25625_RXBxSIDL_Reg_t;

typedef uint8_t MCP25625_RXBxEID8_Reg_t;
typedef uint8_t MCP25625_RXBxEID0_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t DLC	:4;
		uint8_t RB0	:1;
		uint8_t RB1	:1;
		enum
		{
			extendedDataFrameReceived,
			extendedRemoteTransmitRequestReceived
		} RTR	:1;
		uint8_t unimplemented7	:1;
	} Bits;
} MCP25625_RXBxDLC_Reg_t;

typedef uint8_t MCP25625_RXBxDn_Reg_t;

typedef union
{
	uint8_t array[13];
	struct
	{
		MCP25625_RXBxSIDH_Reg_t RXBxSIDH_Reg;
		MCP25625_RXBxSIDL_Reg_t RXBxSIDL_Reg;
		MCP25625_RXBxEID8_Reg_t RXBxEIDH_Reg;
		MCP25625_RXBxEID0_Reg_t RXBxEIDL_Reg;
		MCP25625_RXBxDLC_Reg_t RXBxDLC_Reg;
		MCP25625_RXBxDn_Reg_t RXBxDn_Reg[8];
	} Struct;
} MCP25625_RXBx_t;

typedef uint8_t MCP25625_TXBxSIDH_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t EID 			:2;
		uint8_t unimplemented2	:1;
		enum
		{
			transmitStandardId,
			transmitExtendedId
		} EXIDE	:1;
		uint8_t unimplemented4	:1;
		uint8_t SID				:3;
	} Bits;
} MCP25625_TXBxSIDL_Reg_t;

typedef uint8_t MCP25625_TXBxEID8_Reg_t;
typedef uint8_t MCP25625_TXBxEID0_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t DLC 				:4;
		uint8_t unimplemented5_4	:2;
		enum
		{
			messageWillBeDataFrame,
			messageWillBeRemoteTransmitRequest
		} RTR	:1;
		uint8_t unused :1;
	} Bits;
} MCP25625_TXBxDLC_Reg_t;

typedef uint8_t MCP25625_TXBxDn_Reg_t;

typedef union
{
	uint8_t array[13];
	struct
	{
		MCP25625_TXBxSIDH_Reg_t TXBxSIDH_Reg;
		MCP25625_TXBxSIDL_Reg_t TXBxSIDL_Reg;
		MCP25625_TXBxEID8_Reg_t TXBxEIDH_Reg;
		MCP25625_TXBxEID0_Reg_t TXBxEIDL_Reg;
		MCP25625_TXBxDLC_Reg_t TXBxDLC_Reg;
		MCP25625_TXBxDn_Reg_t TXBxDn_Reg[8];
	} Struct;
} MCP25625_TXBx_t;

typedef struct
{
	SPI_TypeDef *SPIx;
	GPIO_TypeDef *CS_Port;
	uint16_t csPin;
	MCP25625_CANCTRL_Reg_t CANCTRL_Reg;
	MCP25625_CNF1_Reg_t CNF1_Reg;
	MCP25625_CNF2_Reg_t CNF2_Reg;
	MCP25625_CNF3_Reg_t CNF3_Reg;
} MCP25625_t;

uint8_t MCP25625_Init(MCP25625_t *Device_Inits);
uint8_t MCP25625_LoadTxBufferAtD0(uint8_t *data, uint8_t dataLength);
uint8_t MCP25625_LoadTxBufferAtSIDH(uint16_t id, uint8_t *data, uint8_t dataLength);
uint8_t MCP25625_ReadRxBufferAtD0(uint8_t *data, uint8_t dataLength);
uint8_t MCP25625_ReadRxBufferAtSIDH(MCP25625_RXBx_t *RXBx, uint8_t dataLength);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_MCP25625_H_ */
