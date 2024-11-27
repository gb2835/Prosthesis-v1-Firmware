/*******************************************************************************
*
* TITLE:	Driver for Microchip MCP25625 CAN Controller
* AUTHOR:	Greg Berkeley
* RELEASE:	??
*
* NOTES
* 1. This driver is based on:
*     - MCP25625 CAN Controller with Integrated Transceiver
*        - Document Number: DS20005282C
* 2. #define MCP25625_NUMBER_OF_DEVICES must be updated to (at least) the number
*    of devices used.
* 3. Only standard data frames are used.
* 4. Interrupts are not used.
* 5. All TX buffers are set to lowest message priority (TXP = 00).
* 6. All RX buffers are set to receive any message.
*
*******************************************************************************/

#include "mcp25625.h"
#include "stm32l4xx_ll_utils.h"
#include "string.h"


/*******************************************************************************
* PRIVATE DEFINTIONS
*******************************************************************************/

#define CANCTRL_REG		0x0F
#define CANSTAT_REG		0x0E
#define CANINTF_REG		0x2C
#define CNF3_REG		0x28
#define CNF2_REG		0x29
#define CNF1_REG		0x2A
#define TXB0CTRL_REG	0x30
#define TXB0SIDH_REG	0x31
#define TXB1CTRL_REG	0x40
#define TXB1SIDH_REG	0x41
#define TXB2CTRL_REG	0x50
#define TXB2SIDH_REG	0x51
#define RXB0CTRL_REG	0x60
#define RXB1CTRL_REG	0x70

#define CANCTRL_RESET_VALUE	0b10000111

#define BIT_MODIFY					0b00000101
#define LOAD_TX_BUFFER_0_AT_SIDH	0b01000000
#define LOAD_TX_BUFFER_0_AT_D0		0b01000001
#define LOAD_TX_BUFFER_1_AT_SIDH	0b01000010
#define LOAD_TX_BUFFER_1_AT_D0		0b01000011
#define LOAD_TX_BUFFER_2_AT_SIDH	0b01000100
#define LOAD_TX_BUFFER_2_AT_D0		0b01000101
#define READ						0b00000011
#define READ_RX_BUFFER_0_AT_SIDH	0b10010000
#define READ_RX_BUFFER_0_AT_D0		0b10010010
#define READ_RX_BUFFER_1_AT_SIDH	0b10010100
#define READ_RX_BUFFER_1_AT_D0		0b10010110
#define READ_STATUS					0b10100000
#define RESET						0b11000000
#define RTS_T0						0b10000001
#define RTS_T1						0b10000010
#define RTS_T2						0b10000100
#define	WRITE						0b00000010

#define RECEIVE_ANY_MESSAGE_MASK	0b01100000
#define RX0IF_STATUS_MASK			0b00000001
#define RX1IF_STATUS_MASK			0b00000010
#define TX0REQ_STATUS_MASK			0b00000100
#define TX1REQ_STATUS_MASK			0b00010000
#define TX2REQ_STATUS_MASK			0b01000000

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t RX0IF	:1;
		uint8_t RX1IF	:1;
		uint8_t TX0IF	:1;
		uint8_t TX1IF	:1;
		uint8_t TX2IF	:1;
		uint8_t ERRIF	:1;
		uint8_t WAKIF	:1;
		uint8_t MERRF	:1;
	} Bits;
} CANINTF_Reg_t;

typedef union
{
	uint8_t value;
	struct
	{
		enum
		{
			lowestMessagePriority,
			lowIntermediateMessagePriority,
			highIntermediateMessagePriority,
			highestMessagePriority
		} TXP :2;
		uint8_t unimplemented1 :1;
		enum
		{
			notPendingTransmission,
			pendingTransmission
		} TXREQ :1;
		enum
		{
			noBusErrorDuringTransmission,
			busErrorDuringTransmission
		} TXERR :1;
		enum
		{
			noArbirtationLost,
			arbitrationLost
		} MLOA :1;
		enum
		{
			messageAborted,
			transmissionSuccessful
		} ABTF :1;
		uint8_t unimplemented7 :1;
	} Bits;
} TXBxCTRL_Reg_t;

typedef struct
{
	SPI_TypeDef *SPIx;
	GPIO_TypeDef *CS_Port;
	uint16_t csPin;
	MCP25625_CANCTRL_Reg_t CANCTRL_Reg;
	MCP25625_CNF1_Reg_t CNF1_Reg;
	MCP25625_CNF2_Reg_t CNF2_Reg;
	MCP25625_CNF3_Reg_t CNF3_Reg;
	uint8_t isInit;
} Device_t;

static Device_t Device[MCP25625_NUMBER_OF_DEVICES];

static void InitRXBx(uint8_t deviceIndex);
static void ResetDevice(uint8_t deviceIndex);
static void ReadRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nBytes);
static void WriteRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nBytes);
static void ModifyRegisterBits(uint8_t deviceIndex, uint8_t reg, uint8_t mask, uint8_t data);
static void RequestToSend(uint8_t deviceIndex, uint8_t RTS_Tx);
static uint8_t ReadStatus(uint8_t deviceIndex);
static inline void SetChipSelect(uint8_t deviceIndex);
static inline void ClearChipSelect(uint8_t deviceIndex);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

uint8_t MCP25625_Init(uint8_t deviceIndex, MCP25625_t *Device_Inits)
{
	if(deviceIndex++ > MCP25625_NUMBER_OF_DEVICES)
		__NOP(); // add assert??

	memcpy(&Device[deviceIndex], &Device_Inits[deviceIndex], sizeof(MCP25625_t));

	ClearChipSelect(deviceIndex);

	ResetDevice(deviceIndex);

	uint8_t canCtrlReg;
	ReadRegisterData(deviceIndex, CANCTRL_REG, &canCtrlReg, sizeof(canCtrlReg));
	if(canCtrlReg != CANCTRL_RESET_VALUE)
		return mcp25625_resetError;

	InitRXBx(deviceIndex);

	uint8_t configRegs[3];
	uint8_t configRegValues[3] = {Device[deviceIndex].CNF3_Reg.value, Device[deviceIndex].CNF2_Reg.value, Device[deviceIndex].CNF1_Reg.value};
	WriteRegisterData(deviceIndex, CNF3_REG, configRegValues, sizeof(configRegValues));
	ReadRegisterData(deviceIndex, CNF3_REG, configRegs, sizeof(configRegs));
	for(uint8_t i = 0; i < sizeof(configRegs); i++)
		if(configRegs[i] != configRegValues[i])
			return mcp25625_configError;

	WriteRegisterData(deviceIndex, CANCTRL_REG, &Device[deviceIndex].CANCTRL_Reg.value, sizeof(Device[deviceIndex].CANCTRL_Reg.value));
	ReadRegisterData(deviceIndex, CANCTRL_REG, &canCtrlReg, sizeof(canCtrlReg));
	if(canCtrlReg != Device[deviceIndex].CANCTRL_Reg.value)
		return mcp25625_canCtrlError;

	Device[deviceIndex].isInit = 1;

	return mcp25625_noError;
}

uint8_t MCP25625_LoadTxBufferAtD0(uint8_t deviceIndex, uint8_t *data, uint8_t dataLength)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t rtsTx = 0;
	uint8_t txbxDataAddress = 0;
	uint8_t status = ReadStatus(deviceIndex);
	if(!(status & TX2REQ_STATUS_MASK))
	{
		rtsTx = RTS_T2;
		txbxDataAddress = LOAD_TX_BUFFER_2_AT_D0;
	}
	else if(!(status & TX1REQ_STATUS_MASK))
	{
		rtsTx = RTS_T1;
		txbxDataAddress = LOAD_TX_BUFFER_1_AT_D0;
	}
	else if(!(status & TX0REQ_STATUS_MASK))
	{
		rtsTx = RTS_T0;
		txbxDataAddress = LOAD_TX_BUFFER_0_AT_D0;
	}

	if(txbxDataAddress)
	{
		SetChipSelect(deviceIndex);

		while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, txbxDataAddress);

		for(uint8_t i = 0; i < dataLength; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
			LL_SPI_TransmitData8(Device[deviceIndex].SPIx, data[i]);
		}

		while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
		ClearChipSelect(deviceIndex);

		while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
			LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		RequestToSend(deviceIndex, rtsTx);

		return 0;
	}

	return 1;
}

uint8_t MCP25625_LoadTxBufferAtSIDH(uint8_t deviceIndex, uint16_t id, uint8_t *data, uint8_t dataLength)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t rtsTx = 0;
	uint8_t txbxDataAddress = 0;
	uint8_t status = ReadStatus(deviceIndex);
	if(!(status & TX2REQ_STATUS_MASK))
	{
		rtsTx = RTS_T2;
		txbxDataAddress = LOAD_TX_BUFFER_2_AT_SIDH;
	}
	else if(!(status & TX1REQ_STATUS_MASK))
	{
		rtsTx = RTS_T1;
		txbxDataAddress = LOAD_TX_BUFFER_1_AT_SIDH;
	}
	else if(!(status & TX0REQ_STATUS_MASK))
	{
		rtsTx = RTS_T0;
		txbxDataAddress = LOAD_TX_BUFFER_0_AT_SIDH;
	}

	if(txbxDataAddress)
	{
		MCP25625_TXBx_t TXBx;
		memset(&TXBx, 0, sizeof(TXBx));
		TXBx.Struct.TXBxSIDH_Reg = id >> 3;
		TXBx.Struct.TXBxSIDL_Reg.Bits.EXIDE = transmitStandardId;
		TXBx.Struct.TXBxSIDL_Reg.Bits.SID = id & 0x07;
		TXBx.Struct.TXBxDLC_Reg.Bits.DLC = dataLength;
		for(uint8_t i = 0; i < dataLength; i++)
			TXBx.Struct.TXBxDn_Reg[i] = data[i];

		SetChipSelect(deviceIndex);

		while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, txbxDataAddress);

		uint8_t nBytes = dataLength + 5;						// data register + 5 registers in Rx buffer
		for(uint8_t i = 0; i < nBytes; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
			LL_SPI_TransmitData8(Device[deviceIndex].SPIx, TXBx.array[i]);
		}

		while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
		ClearChipSelect(deviceIndex);

		while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
			LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		RequestToSend(deviceIndex, rtsTx);

		return 0;
	}

	return 1;
}

uint8_t MCP25625_ReadRxBufferAtD0(uint8_t deviceIndex, uint8_t *data, uint8_t dataLength)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t rxbxDataAddress = 0;
	uint8_t status = ReadStatus(deviceIndex);
	if(status & RX0IF_STATUS_MASK)
		rxbxDataAddress = READ_RX_BUFFER_0_AT_D0;
	else if(status & RX1IF_STATUS_MASK)
		rxbxDataAddress = READ_RX_BUFFER_1_AT_D0;

	if(rxbxDataAddress)
	{
		SetChipSelect(deviceIndex);

		while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, rxbxDataAddress);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		for(uint8_t i = 0; i < dataLength; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
			LL_SPI_TransmitData8(Device[deviceIndex].SPIx, 0);
			while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
			data[i] = LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);
		}

		ClearChipSelect(deviceIndex);

		return 0;
	}

	return 1;
}

uint8_t MCP25625_ReadRxBufferAtSIDH(uint8_t deviceIndex, MCP25625_RXBx_t *RXBx, uint8_t dataLength)
{
	if(!Device[deviceIndex].isInit)
		__NOP(); // add assert??

	uint8_t rxbxSIDH_Address = 0;
	uint8_t status = ReadStatus(deviceIndex);
	if(status & RX0IF_STATUS_MASK)
		rxbxSIDH_Address = READ_RX_BUFFER_0_AT_SIDH;
	else if(status & RX1IF_STATUS_MASK)
		rxbxSIDH_Address = READ_RX_BUFFER_1_AT_SIDH;

	if(rxbxSIDH_Address)
	{
		SetChipSelect(deviceIndex);

		while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, rxbxSIDH_Address);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		uint8_t nBytes = dataLength + 5;						// data register + 5 registers in Rx buffer
		for(uint8_t i = 0; i < nBytes; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
			LL_SPI_TransmitData8(Device[deviceIndex].SPIx, 0);
			while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
			RXBx->array[i] = LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);
		}

		ClearChipSelect(deviceIndex);

		return 0;
	}

	return 1;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void InitRXBx(uint8_t deviceIndex)
{
	// Set RXBx to receive any message
	ModifyRegisterBits(deviceIndex, RXB0CTRL_REG, 0b01100000, 0b01100000);
	ModifyRegisterBits(deviceIndex, RXB1CTRL_REG, 0b01100000, 0b01100000);
}

static void ResetDevice(uint8_t deviceIndex)
{
	SetChipSelect(deviceIndex);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, RESET);

	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
	ClearChipSelect(deviceIndex);

	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	LL_mDelay(1);	// Minimum 2 us required (t_RL), usdelay()??
}

static void ReadRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nDataBytes)
{
	SetChipSelect(deviceIndex);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, READ);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
	LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, startReg);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
	LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	for(uint8_t i = 0; i < nDataBytes; i++)
	{
		while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, 0);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
		data[i] = LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);
	}

	ClearChipSelect(deviceIndex);
}

static void WriteRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nDataBytes)
{
	SetChipSelect(deviceIndex);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, WRITE);
	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, startReg);

	for(uint8_t i = 0; i < nDataBytes; i++)
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, data[i]);

	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
	ClearChipSelect(deviceIndex);

	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);
}

static void ModifyRegisterBits(uint8_t deviceIndex, uint8_t reg, uint8_t mask, uint8_t data)
{
	SetChipSelect(deviceIndex);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, BIT_MODIFY);
	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, reg);
	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, mask);
	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, data);

	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
	ClearChipSelect(deviceIndex);

	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);
}

static void RequestToSend(uint8_t deviceIndex, uint8_t RTS_Tx)
{
	SetChipSelect(deviceIndex);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, RTS_Tx);

	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
	ClearChipSelect(deviceIndex);

	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);
}

static uint8_t ReadStatus(uint8_t deviceIndex)
{
	SetChipSelect(deviceIndex);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, READ_STATUS);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
	LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, 0);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
	uint8_t status = LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	ClearChipSelect(deviceIndex);

	return status;
}

static inline void SetChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_ResetOutputPin(Device[deviceIndex].CS_Port, Device[deviceIndex].csPin);
}

static inline void ClearChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_SetOutputPin(Device[deviceIndex].CS_Port, Device[deviceIndex].csPin);
}


/*******************************************************************************
* END
*******************************************************************************/
