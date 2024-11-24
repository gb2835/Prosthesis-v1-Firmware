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
* 2. Only standard data frames are used.
* 3. Polling is used (no interrupts).
* 4. All TX buffers are set to lowest message priority (TXP = 00).??
* 5. All RX buffers are set to receive any message.
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

static Device_t Device;

static void InitRXBx(void);
static void ResetDevice(void);
static void ReadRegisterData(uint8_t startReg, uint8_t *data, uint8_t nBytes); // static inline??
static void WriteRegisterData(uint8_t startReg, uint8_t *data, uint8_t nBytes); // static inline??
static void ModifyRegisterBits(uint8_t reg, uint8_t mask, uint8_t data);
static void RequestToSend(uint8_t RTS_Tx); // static inline??
static uint8_t ReadStatus(void);
static void SetChipSelect(void); // static inline??
static void ClearChipSelect(void); // static inline??


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

uint8_t MCP25625_Init(MCP25625_t *Device_Inits)
{
	memcpy(&Device, Device_Inits, sizeof(MCP25625_t));

	ClearChipSelect();

	ResetDevice();

	uint8_t canCtrlReg;
	ReadRegisterData(CANCTRL_REG, &canCtrlReg, sizeof(canCtrlReg));
	if(canCtrlReg != CANCTRL_RESET_VALUE)
		return mcp25625_resetError;

	InitRXBx();

	uint8_t configRegs[3];
	uint8_t configRegValues[3] = {Device.CNF3_Reg.value, Device.CNF2_Reg.value, Device.CNF1_Reg.value};
	WriteRegisterData(CNF3_REG, configRegValues, sizeof(configRegValues));
	ReadRegisterData(CNF3_REG, configRegs, sizeof(configRegs));
	for(uint8_t i = 0; i < sizeof(configRegs); i++)
		if(configRegs[i] != configRegValues[i])
			return mcp25625_configError;

	WriteRegisterData(CANCTRL_REG, &Device.CANCTRL_Reg.value, sizeof(Device.CANCTRL_Reg.value));
	ReadRegisterData(CANCTRL_REG, &canCtrlReg, sizeof(canCtrlReg));
	if(canCtrlReg != Device.CANCTRL_Reg.value)
		return mcp25625_canCtrlError;

	Device.isInit = 1;

	return mcp25625_noError;
}

uint8_t MCP25625_LoadTxBufferAtD0(uint8_t *data, uint8_t dataLength)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint8_t rtsTx = 0;
	uint8_t txbxDataAddress = 0;
	uint8_t status = ReadStatus();
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
		SetChipSelect();

		while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
		LL_SPI_TransmitData8(Device.SPIx, txbxDataAddress);

		for(uint8_t i = 0; i < dataLength; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
			LL_SPI_TransmitData8(Device.SPIx, data[i]);
		}

		while(LL_SPI_GetTxFIFOLevel(Device.SPIx));
		ClearChipSelect();

		while(LL_SPI_IsActiveFlag_RXNE(Device.SPIx))
			LL_SPI_ReceiveData8(Device.SPIx);

		RequestToSend(rtsTx);

		return 0;
	}

	return 1;
}

uint8_t MCP25625_LoadTxBufferAtSIDH(uint16_t id, uint8_t *data, uint8_t dataLength)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint8_t rtsTx = 0;
	uint8_t txbxDataAddress = 0;
	uint8_t status = ReadStatus();
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
		TXBx.Struct.TXBxSIDH_Reg = id >> 3;
		TXBx.Struct.TXBxSIDL_Reg.Bits.EXIDE = transmitStandardId;
		TXBx.Struct.TXBxSIDL_Reg.Bits.SID = id & 0x07;
		TXBx.Struct.TXBxDLC_Reg.Bits.DLC = dataLength;
		for(uint8_t i = 0; i < dataLength; i++)
			TXBx.Struct.TXBxDn_Reg[i] = data[i];

		SetChipSelect();

		while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
		LL_SPI_TransmitData8(Device.SPIx, txbxDataAddress);

		uint8_t nBytes = dataLength + 5;						// data register + 5 registers in Rx buffer
		for(uint8_t i = 0; i < nBytes; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
			LL_SPI_TransmitData8(Device.SPIx, TXBx.array[i]);
		}

		while(LL_SPI_GetTxFIFOLevel(Device.SPIx));
		ClearChipSelect();

		while(LL_SPI_IsActiveFlag_RXNE(Device.SPIx))
			LL_SPI_ReceiveData8(Device.SPIx);

		RequestToSend(rtsTx);

		return 0;
	}

	return 1;
}

uint8_t MCP25625_ReadRxBufferAtD0(uint8_t *data, uint8_t dataLength)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint8_t rxbxDataAddress = 0;
	uint8_t status = ReadStatus();
	if(status & RX0IF_STATUS_MASK)
		rxbxDataAddress = READ_RX_BUFFER_0_AT_D0;
	else if(status & RX1IF_STATUS_MASK)
		rxbxDataAddress = READ_RX_BUFFER_1_AT_D0;

	if(rxbxDataAddress)
	{
		SetChipSelect();

		while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
		LL_SPI_TransmitData8(Device.SPIx, rxbxDataAddress);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
		LL_SPI_ReceiveData8(Device.SPIx);

		for(uint8_t i = 0; i < dataLength; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
			LL_SPI_TransmitData8(Device.SPIx, 0);
			while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
			data[i] = LL_SPI_ReceiveData8(Device.SPIx);
		}

		ClearChipSelect();

		return 0;
	}

	return 1;
}

uint8_t MCP25625_ReadRxBufferAtSIDH(MCP25625_RXBx_t *RXBx, uint8_t dataLength)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint8_t rxbxSIDH_Address = 0;
	uint8_t status = ReadStatus();
	if(status & RX0IF_STATUS_MASK)
		rxbxSIDH_Address = READ_RX_BUFFER_0_AT_SIDH;
	else if(status & RX1IF_STATUS_MASK)
		rxbxSIDH_Address = READ_RX_BUFFER_1_AT_SIDH;

	if(rxbxSIDH_Address)
	{
		SetChipSelect();

		while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
		LL_SPI_TransmitData8(Device.SPIx, rxbxSIDH_Address);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
		LL_SPI_ReceiveData8(Device.SPIx);

		uint8_t nBytes = dataLength + 5;						// data register + 5 registers in Rx buffer
		for(uint8_t i = 0; i < nBytes; i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
			LL_SPI_TransmitData8(Device.SPIx, 0);
			while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
			RXBx->array[i] = LL_SPI_ReceiveData8(Device.SPIx);
		}

		ClearChipSelect();

		return 0;
	}

	return 1;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void InitRXBx(void)
{
	// Set RXBx to receive any message
	ModifyRegisterBits(RXB0CTRL_REG, 0b01100000, 0b01100000);
	ModifyRegisterBits(RXB1CTRL_REG, 0b01100000, 0b01100000);
}

static void ResetDevice(void)
{
	SetChipSelect();

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, RESET);

	while(LL_SPI_GetTxFIFOLevel(Device.SPIx));
	ClearChipSelect();

	while(LL_SPI_IsActiveFlag_RXNE(Device.SPIx))
		LL_SPI_ReceiveData8(Device.SPIx);

	LL_mDelay(1);	// Minimum 2 us required (t_RL), usdelay()??
}

static void ReadRegisterData(uint8_t startReg, uint8_t *data, uint8_t nDataBytes)
{
	SetChipSelect();

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, READ);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
	LL_SPI_ReceiveData8(Device.SPIx);

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, startReg);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
	LL_SPI_ReceiveData8(Device.SPIx);

	for(uint8_t i = 0; i < nDataBytes; i++)
	{
		while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
		LL_SPI_TransmitData8(Device.SPIx, 0);
		while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
		data[i] = LL_SPI_ReceiveData8(Device.SPIx);
	}

	ClearChipSelect();
}

static void WriteRegisterData(uint8_t startReg, uint8_t *data, uint8_t nDataBytes)
{
	SetChipSelect();

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, WRITE);
	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, startReg);

	for(uint8_t i = 0; i < nDataBytes; i++)
		LL_SPI_TransmitData8(Device.SPIx, data[i]);

	while(LL_SPI_GetTxFIFOLevel(Device.SPIx));
	ClearChipSelect();

	while(LL_SPI_IsActiveFlag_RXNE(Device.SPIx))
		LL_SPI_ReceiveData8(Device.SPIx);
}

static void ModifyRegisterBits(uint8_t reg, uint8_t mask, uint8_t data)
{
	SetChipSelect();

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, BIT_MODIFY);
	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, reg);
	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, mask);
	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, data);

	while(LL_SPI_GetTxFIFOLevel(Device.SPIx));
	ClearChipSelect();

	while(LL_SPI_IsActiveFlag_RXNE(Device.SPIx))
		LL_SPI_ReceiveData8(Device.SPIx);
}

static void RequestToSend(uint8_t RTS_Tx)
{
	SetChipSelect();

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, RTS_Tx);

	while(LL_SPI_GetTxFIFOLevel(Device.SPIx));
	ClearChipSelect();

	while(LL_SPI_IsActiveFlag_RXNE(Device.SPIx))
		LL_SPI_ReceiveData8(Device.SPIx);
}

static uint8_t ReadStatus(void)
{
	SetChipSelect();

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, READ_STATUS);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
	LL_SPI_ReceiveData8(Device.SPIx);

	while(!(LL_SPI_IsActiveFlag_TXE(Device.SPIx)));
	LL_SPI_TransmitData8(Device.SPIx, 0);
	while(!(LL_SPI_IsActiveFlag_RXNE(Device.SPIx)));
	uint8_t status = LL_SPI_ReceiveData8(Device.SPIx);

	ClearChipSelect();

	return status;
}

static void SetChipSelect(void)
{
	LL_GPIO_ResetOutputPin(Device.CS_Port, Device.csPin);
}

static void ClearChipSelect(void)
{
	LL_GPIO_SetOutputPin(Device.CS_Port, Device.csPin);
}


/*******************************************************************************
* END
*******************************************************************************/
