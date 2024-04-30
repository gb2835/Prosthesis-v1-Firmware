/*
 * mcp25625.h
 *
 *  Created on: Dec 17, 2023
 *      Author: Brett Anderson
 */

#ifndef INC_MCP25625_H_
#define INC_MCP25625_H_

#include "string.h"		// Can we lose this??
#include "spi.h"		// Can we lose this??

#pragma pack(push, 1)

typedef union {
	uint8_t value;
	struct {
		uint8_t unused :1;
		uint8_t ABTF :1; 	// message abort flag
		uint8_t MLOA :1; // message lost arbitration
		uint8_t TXERR :1; // transmission error detected
		uint8_t TXREQ :1; // message transmit request
		uint8_t unused1 :1;
		uint8_t TXP :2;			// transmit buffer priority
	} bits;
} txb_ctrl_t;

typedef uint8_t txb_sidh_t; // full byte is addr 10:3. Just for clarity

typedef union {
	uint8_t value;
	struct {
		uint8_t EID :2;	// extended id 17:16
		uint8_t unused1 :1;
		uint8_t EXIDE :1; // extended id enable
		uint8_t unused :1;
		uint8_t SID :3;	// low 3 bits of standard id bits
	} bits;
} txb_sidl_t;

typedef uint8_t txb_eid8_t; // extended id byte, 15:8. Just for consistency.

typedef uint8_t txb_eid0_t; // extended id byte, 7:0. just for consistency.

typedef union {
	uint8_t value;
	struct {
		uint8_t unused :1;
		uint8_t RTR :1;		// remote transmission request
		uint8_t unused1 :2;
		uint8_t DLC :4;		// data length code.
	} bits;
} txb_dlc_t;


typedef union {
	uint8_t bytes[13];
	struct {
		txb_sidh_t SIDH;
		txb_sidl_t SIDL;
		txb_eid8_t EIDH;
		txb_eid0_t EIDL;
		txb_dlc_t DLC;
		uint8_t data[8];
	} txb;
}txbuff_t;


typedef union {
	uint8_t value;
	struct {
		uint8_t unused :1;
		uint8_t RXM :2;		// Receive buffer oprating mode
		uint8_t unused1 :1;
		uint8_t RXRTR :1;		// received remote transfer request
		uint8_t BUKT :1;		// rollover enable
		uint8_t BUKT1 :1;		// read only copy of BUKT
		uint8_t FILHIT0 :1;		// filter hit bit
	} bits;
} rxb_ctrl_t;

typedef uint8_t rxb_sidh_t;		// sid 10:3. just for concistency

typedef union {
	uint8_t value;
	struct {
		uint8_t sid :3;		// sid 2:0
		uint8_t SRR :1; 	// standard frame remote transmit request
		uint8_t IDE :1;		// extended identifier flag
		uint8_t unused :1;
		uint8_t EID :2;		// extended id 17:16
	} bits;
} rxb_sidl_t;

typedef uint8_t rxb_eid8_t;	// eid 15:8
typedef uint8_t rxb_eid0_t;	// eid 7:0

typedef union {
	uint8_t value;
	struct {
		uint8_t unused :1;
		uint8_t RTR :1;		// extended frame remote transmission request
		uint8_t RB1 :1;		// reserved bit
		uint8_t RB0 :1;		// reserved bit
		uint8_t DLC :3;		// data length code
	} bits;
} rxb_dlc_t;

typedef union {
	uint8_t bytes[13];
	struct {
		rxb_sidh_t SIDH;
		rxb_sidl_t SIDL;
		rxb_eid8_t EIDH;
		rxb_eid0_t EIDL;
		rxb_dlc_t DLC;
		uint8_t data[8];
	} txb;
}rxbuff_t;

typedef union {
	uint8_t value;
	struct {
		uint8_t SJW :2;		// sync jump width
		uint8_t BRP :6;		// baud rate prescaler
	} bits;
} cnf1_t;

typedef union {
	uint8_t value;
	struct {
		uint8_t BLTMODE	:1;	// PS2 bit time length
		uint8_t SAM		:1;	// sample point config
		uint8_t PHSEG1	:3;	// ps1 length
		uint8_t PRSEG	:3;	// propagation seg length
	} bits;
} cnf2_t;

typedef union {
	uint8_t value;
	struct {
		uint8_t SOF		:1;	// start of frame signal
		uint8_t WAKFIL	:1;	// wake up filter
		uint8_t unused	:3;
		uint8_t PHSEG2	:3;	// PS2 length
	} bits;
} cnf3_t;

typedef union {
	uint8_t value;
	struct {
		uint8_t MERRE	:1; // message error int en
		uint8_t WAKIE	:1; // wake up int en
		uint8_t ERRIE	:1; // error int en
		uint8_t TX2IE	:1; // transmit buffer 2 empty int en
		uint8_t TX1IE	:1; // transmit buffer 1 empty int en
		uint8_t TX0IE	:1; // transmit buffer 0 empty int en
		uint8_t RX1IE	:1; // receive buffer 1 full int en
		uint8_t RX0IE	:1;	// receive buffer 0 full int en
	} bits;
} caninte_t;

typedef union {
	uint8_t value;
	struct {
		uint8_t MERRF	:1; // message error int flag
		uint8_t WAKIF	:1; // wake up int flag
		uint8_t ERRIF	:1;	// error int flag
		uint8_t TX2IF	:1; // tx buff 2 empty int flag
		uint8_t TX1IF	:1; // tx buff 1 empty int flag
		uint8_t TX0IF	:1; // tx buff 0 empty int flag
		uint8_t RX1IF	:1; // rx buff 1 full int flag
		uint8_t RX0IF	:1;	// rx buff 0 full int flag
	} bits;
} canintf_t;

typedef union {
	uint8_t value;
	struct {
		uint8_t REQOP	:3;	// request operation mode
		uint8_t ABAT	:1; // abort all pending transmissions
		uint8_t OSM		:1; // one shot mode
		uint8_t CLKEN	:1; // clk out pin en
		uint8_t CLKPRE	:2;	// clk out pi prescaler
	} bits;
} canctrl_t;

typedef union {
	uint8_t value;
	struct {
		uint8_t OPMOD	:3;	// operation mode
		uint8_t unused	:1;
		uint8_t ICOD	:3;	// interrupt flag
		uint8_t unused1	:1;
	} bits;
} canstat_t;

typedef enum {
	SPI_TRANSFER_TYPE_READ_REG,
	SPI_TRANSFER_TYPE_WRITE_REG,
	SPI_TRANSFER_TYPE_SEND,
	SPI_TRANSFER_TYPE_READ,
	SPI_TRANSFER_TYPE_ERROR
} spi_transfer_type_t;

typedef struct {
	uint8_t tx[16];			// tx data
	uint8_t txLength;		// length of loaded tx data
	uint8_t rx[16];			// rx data
	uint8_t rxLength;		// length of received rx data
} spi_transfer_t;

typedef struct {
	spi_transfer_type_t	type;
	union {
		spi_transfer_t * transfer;	// pointer to transfer data
		uint32_t error;				// error
	} data;
} transfer_event_t;

typedef struct {
	uint8_t	ready;			// is this buffer ready to send
	spi_transfer_t transfer;
	void (*callback)(transfer_event_t * evt);
} spi_buffer_t;

#pragma pack(pop)

#define MAX_TRANSFER_BUFFERS 16

typedef struct {
	canstat_t 		stat;
	canctrl_t 		ctrl;
	canintf_t 		intf;
	caninte_t 		inte;
	spi_buffer_t	transferBuffer[MAX_TRANSFER_BUFFERS];
	uint8_t 		currentTransferIndex;
	uint8_t 		sending;		// is spi actively sending
} mcp25625_t;

#define CMD_RESET					0xC0	/* reset internal registers to default */
#define CMD_READ					0x03	/* read from register beginning at address */
#define CMD_READ_RX_BUFFER_BASE		0x90	/* read from rx buff at one of 4 addresses. SEE read buffer locations */
#define CMD_WRITE					0x02	/* write data to register beginning at address */
#define CMD_LOAD_TX_BUFFER_BASE		0x40	/* load buffer starting at one of 6 addresses. See write buffer locations */
#define CMD_RTS_BASE				0x80	/* request to send any of tx buffers. OR with requested buffer. See buffer bit defs */
#define CMD_READ_STATUS				0xA0	/* quick polling of status bits for tx and rx */
#define CMD_RX_STATUS				0xB0	/* quick polling command for filter match, message type */
#define CMD_BIT_MODIFY				0x05	/* set or clear individual bits of register */

#define RX0_SIDH					0x00	/* rxb 0, start at sidh */
#define RX0_D0						0x01	/* rxb 0, start at d0 */
#define RX1_SIDH					0x02	/* rxb 1, start at sidh */
#define RX1_D0						0x03	/* rxb 1, start at d0 */

#define	TXB2_SIDH					0x04	/* txb 0, start at sidh */
#define TXB2_D0						0x05	/* txb 0, start at d0 */
#define	TXB1_SIDH					0x02	/* txb 0, start at sidh */
#define TXB1_D0						0x03	/* txb 0, start at d0 */
#define	TXB0_SIDH					0x00	/* txb 0, start at sidh */
#define TXB0_D0						0x01	/* txb 0, start at d0 */

#define TXB0CTRL					0x30
#define TXB1CTRL					0x40
#define TXB2CTRL					0x50
#define TXBCTRL_TXREQ				0x0B
#define RXB0CTRL					0x60
#define RXB1CTRL					0x70
#define CNF3						0x28	// Configuration 3
#define CNF2						0x29	// Configuration 2
#define CNF1						0x2A	// Configuration 1
#define CANINTE						0x2B
#define CANINTF						0x2C
#define CANSTAT						0x0E
#define CANCTRL						0x0F	// CAN control


void CAN_transmit(uint16_t CAN_ID, uint8_t length, uint8_t * message);
void CAN_configure();
#endif /* INC_MCP25625_H_ */
