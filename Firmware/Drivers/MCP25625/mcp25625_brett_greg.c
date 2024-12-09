/*******************************************************************************
*
* TITLE		Driver for Microchip MCP25625 CAN Controller
* AUTHOR	Greg Berkeley
* RELEASE	??
*
* NOTES
* 1. This driver is based on:
*     - MCP25625 CAN Controller with Integrated Transceiver
*        - Document Number: DS20005282C
* 2. #define MCP25625_NUMBER_OF_DEVICES must be updated to (at least) the number of devices used.
* 2. Only standard data frames are used.
* 3. Polling is used (no interrupts).
* 4. All TX buffers are set to lowest message priority (TXP = 00).??
* 5. All RX buffers are set to receive any message.
*
*******************************************************************************/

#include "mcp25625.h"
#include "stm32l4xx_ll_utils.h"
#include <string.h>


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
			LowestMessagePriority,
			LowIntermediateMessagePriority,
			HighIntermediateMessagePriority,
			HighestMessagePriority
		} TXP :2;
		uint8_t Unimplemented1 :1;
		enum
		{
			NotPendingTransmission,
			PendingTransmission
		} TXREQ :1;
		enum
		{
			NoBusErrorDuringTransmission,
			BusErrorDuringTransmission
		} TXERR :1;
		enum
		{
			NoArbirtationLost,
			ArbitrationLost
		} MLOA :1;
		enum
		{
			MessageAborted,
			TransmissionSuccessful
		} ABTF :1;
		uint8_t Unimplemented7 :1;
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
static void ReadRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nBytes); // static inline??
static void WriteRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nBytes); // static inline??
static void ModifyRegisterBits(uint8_t deviceIndex, uint8_t reg, uint8_t mask, uint8_t data);
static void RequestToSend(uint8_t deviceIndex, uint8_t RTS_Tx); // static inline??
static uint8_t ReadStatus(uint8_t deviceIndex);
static void SetChipSelect(uint8_t deviceIndex); // static inline??
static void ClearChipSelect(uint8_t deviceIndex); // static inline??


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

MCP25625_Error_e MCP25625_Init(uint8_t deviceIndex, MCP25625_Init_t *Device_Inits)
{
	if(deviceIndex + 1 > MCP25625_NUMBER_OF_DEVICES)
		__NOP(); // add assert??

	memcpy(&Device[deviceIndex], Device_Inits, sizeof(MCP25625_Init_t));

	ClearChipSelect(deviceIndex);
	ResetDevice(deviceIndex);

	uint8_t canCtrlReg;
	ReadRegisterData(deviceIndex, CANCTRL_REG, &canCtrlReg, sizeof(canCtrlReg));
	if(canCtrlReg != CANCTRL_RESET_VALUE)
		return MCP25625_ResetError;

	InitRXBx(deviceIndex);

	uint8_t configRegs[3];
	uint8_t configRegValues[3] = {Device[deviceIndex].CNF3_Reg.value, Device[deviceIndex].CNF2_Reg.value, Device[deviceIndex].CNF1_Reg.value};
	WriteRegisterData(deviceIndex, CNF3_REG, configRegValues, sizeof(configRegValues));
	ReadRegisterData(deviceIndex, CNF3_REG, configRegs, sizeof(configRegs));
	for(uint8_t i = 0; i < sizeof(configRegs); i++)
		if(configRegs[i] != configRegValues[i])
			return MCP25625_ConfigError;

	WriteRegisterData(deviceIndex, CANCTRL_REG, &Device[deviceIndex].CANCTRL_Reg.value, sizeof(Device[deviceIndex].CANCTRL_Reg.value));
	ReadRegisterData(deviceIndex, CANCTRL_REG, &canCtrlReg, sizeof(canCtrlReg));
	if(canCtrlReg != Device[deviceIndex].CANCTRL_Reg.value)
		return MCP25625_CANCTRL_Error;

	Device[deviceIndex].isInit = 1;

	return MCP25625_NoError;
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

		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, txbxDataAddress);

		for(uint8_t i = 0; i < dataLength; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
			LL_SPI_TransmitData8(Device[deviceIndex].SPIx, data[i]);
		}

		while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));


		while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
			LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		RequestToSend(deviceIndex, rtsTx);
		ClearChipSelect(deviceIndex);
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
		TXBx.Struct.TXBxSIDL_Reg.Bits.EXIDE = TransmitStandardID;
		TXBx.Struct.TXBxSIDL_Reg.Bits.SID = id & 0x07;
		TXBx.Struct.TXBxDLC_Reg.Bits.DLC = dataLength;
		for(uint8_t i = 0; i < dataLength; i++)
			TXBx.Struct.TXBxDn_Reg[i] = data[i];

		SetChipSelect(deviceIndex);

		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, txbxDataAddress);

		uint8_t nBytes = dataLength + 5;						// data register + 5 registers in Rx buffer
		for(uint8_t i = 0; i < nBytes; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));		// block on txe
			LL_SPI_TransmitData8(Device[deviceIndex].SPIx, TXBx.array[i]);
		}

		while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));				// wait for fifo to clear


		while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))			// clear out rx fifo
			LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		ClearChipSelect(deviceIndex);

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

		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, rxbxDataAddress);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		for(uint8_t i = 0; i < dataLength; i++)
		{
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

		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, rxbxSIDH_Address);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

		uint8_t nBytes = dataLength + 5;						// data register + 5 registers in Rx buffer
		for(uint8_t i = 0; i < nBytes; i++)
		{
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

//	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, RESET);
	// wait for fifo to clear all bytes
	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
	// while fifo has data, read to clear the fifo
	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	ClearChipSelect(deviceIndex);
	LL_mDelay(1);					// Minimum 2 us required (t_RL)
}

static void ReadRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nDataBytes)
{
	SetChipSelect(deviceIndex);

	// send read command. 3 byte fifo
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, READ);
	// send register address
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, startReg);
	// block until both bytes are sent
	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));
	// clear rx fifo of responses to command/reg
	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	for(uint8_t i = 0; i < nDataBytes; i++)
	{
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, 0);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))); // waiting for rxne blocks sending the next
		data[i] = LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);
	}

	ClearChipSelect(deviceIndex);
}

static void WriteRegisterData(uint8_t deviceIndex, uint8_t startReg, uint8_t *data, uint8_t nDataBytes)
{
	SetChipSelect(deviceIndex);

	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, WRITE);
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, startReg);

	for(uint8_t i = 0; i < nDataBytes; i++) {
		while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx))); // don't send unless their is room
		LL_SPI_TransmitData8(Device[deviceIndex].SPIx, data[i]);
	}
	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));			// wait for tx to clear the fifo

	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))		// clear out the rx fifo
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	ClearChipSelect(deviceIndex);
}

static void ModifyRegisterBits(uint8_t deviceIndex, uint8_t reg, uint8_t mask, uint8_t data)
{
	SetChipSelect(deviceIndex);

	// first 3 bytes will def fit in fifo, so don't wait
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, BIT_MODIFY);
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, reg);
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, mask);

	while(!(LL_SPI_IsActiveFlag_TXE(Device[deviceIndex].SPIx)));	// check on the 4th for space
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, data);

	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));		// block until fifo is cleared

	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))	// clear rx fifo
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	ClearChipSelect(deviceIndex);
}

static void RequestToSend(uint8_t deviceIndex, uint8_t RTS_Tx)
{
	SetChipSelect(deviceIndex);

	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, RTS_Tx);

	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));		// block until fifo is cleared
	while(!LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx));	// ISSUE: This line was added to prevent missing receive bits
	while(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx))	// clear rx fifo
		LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);

	ClearChipSelect(deviceIndex);
}

static uint8_t ReadStatus(uint8_t deviceIndex)
{
	SetChipSelect(deviceIndex);

	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, READ_STATUS);
	LL_SPI_TransmitData8(Device[deviceIndex].SPIx, 0);
	while(LL_SPI_GetTxFIFOLevel(Device[deviceIndex].SPIx));		// block until fifo is cleared

	while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));// wait for rx data
	LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);				// clear first byte
	while(!(LL_SPI_IsActiveFlag_RXNE(Device[deviceIndex].SPIx)));// wait for rx data
	uint8_t status = LL_SPI_ReceiveData8(Device[deviceIndex].SPIx);	// this is the response byte

	ClearChipSelect(deviceIndex);

	return status;
}

static void SetChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_ResetOutputPin(Device[deviceIndex].CS_Port, Device[deviceIndex].csPin);
}

static void ClearChipSelect(uint8_t deviceIndex)
{
	LL_GPIO_SetOutputPin(Device[deviceIndex].CS_Port, Device[deviceIndex].csPin);
}


/*******************************************************************************
* END
*******************************************************************************/
