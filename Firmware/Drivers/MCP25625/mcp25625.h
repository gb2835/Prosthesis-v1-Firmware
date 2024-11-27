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

#define MCP25625_NUMBER_OF_DEVICES	2

typedef enum
{
	MCP25625_NoError,
	MCP25625_ResetError,
	MCP25625_ConfigError,
	MCP25625_CANCTRL_Error
} MCP25625_Error_e;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t BRP	:6;
		enum
		{
			Length1xT_Q,
			Length2xT_Q,
			Length3xT_Q,
			Length4xT_Q
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
			BusSampledOnceAtSamplePoint,
			BusSampledThreeTimesAtSamePoint
		} SAM :1;
		enum
		{
			PS2LengthIsGreaterOfPS1AndIPT,
			PS2LengthDeterminedByCNF3
		} BLTMODE :1;
	} Bits;
} MCP25625_CNF2_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t PHSEG2			:3;
		uint8_t Unimplemented	:3;
		enum
		{
			WakeUpFilterIsDisabled,
			WakeUpFilterIsEnabled
		} WAKFIL :1;
		enum
		{
			ClockoutPinIsEnabledForClockOutFunction,
			ClockoutPinIsEnabledForSOF_Signal
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
			ClockoutDiv1,
			ClockoutDiv2,
			ClockoutDiv4,
			ClockoutDiv8
		} CLKPRE :2;
		enum
		{
			ClockoutDisabled,
			ClockoutEnabled
		} CLKEN :1;
		enum
		{
			OneShotModeDisabled,
			OneShotModeEnabled
		} OSM :1;
		enum
		{
			AbortAllTransmissions,
			AbortAllPendingTransmitBuffers
		} ABAT :1;
		enum
		{
			NormalOperationMode,
			SleepMode,
			LoopBackMode,
			ListenOnlyMode,
			ConfigurationMode
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
		uint8_t Unimplemented2	:1;
		enum
		{
			ReceiveStandardID,
			ReceiveExtendedID
		} IDE	:1;
		enum
		{
			StandardFrameReceived,
			StandardRemoteTransmitRequestReceived
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
			ExtendedDataFrameReceived,
			ExtendedRemoteTransmitRequestReceived
		} RTR	:1;
		uint8_t Unimplemented7	:1;
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
		uint8_t Unimplemented2	:1;
		enum
		{
			TransmitStandardID,
			TransmitExtendedID
		} EXIDE	:1;
		uint8_t Unimplemented4	:1;
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
		uint8_t Unimplemented4_5	:2;
		enum
		{
			MessageWillBeDataFrame,
			MessageWillBeRemoteTransmitRequest
		} RTR	:1;
		uint8_t Unused :1;
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
} MCP25625_Init_t;

uint8_t MCP25625_Init(uint8_t deviceIndex, MCP25625_Init_t *Device_Inits);
uint8_t MCP25625_LoadTxBufferAtD0(uint8_t deviceIndex, uint8_t *data, uint8_t dataLength);
uint8_t MCP25625_LoadTxBufferAtSIDH(uint8_t deviceIndex, uint16_t id, uint8_t *data, uint8_t dataLength);
uint8_t MCP25625_ReadRxBufferAtD0(uint8_t deviceIndex, uint8_t *data, uint8_t dataLength);
uint8_t MCP25625_ReadRxBufferAtSIDH(uint8_t deviceIndex, MCP25625_RXBx_t *RXBx, uint8_t dataLength);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_MCP25625_H_ */
