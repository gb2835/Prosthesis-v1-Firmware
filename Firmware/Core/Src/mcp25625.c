/*
 * mcp25625.c
 *
 *  Created on: Dec 17, 2023
 *      Author: Brett Anderson
 */

#include "mcp25625.h"

static mcp25625_t can_tranceiver;

// Disable CS pin (active low)
__STATIC_INLINE void clearChipSelect() {
	LL_GPIO_SetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin); }

// Enable CS pin (active low)
__STATIC_INLINE void setChipSelect() {
	LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin);}

//
void mcp25625_reset()
{
	// Transmit reset command
	setChipSelect();							// Enable CS pin
	LL_SPI_TransmitData8( SPI2, CMD_RESET );	// Send reset command
	while ( (SPI2->SR & SPI_SR_BSY) );			// Wait for status register to not be busy
	clearChipSelect();							// Disable CS pin

	// Clear Rx FIFO
	while ( (SPI2->SR & SPI_SR_FRLVL) )		// Check status register for data in Rx
	{
		uint8_t dummy = SPI2->DR;			// Clear Rx FIFO from the receives
		(void)dummy;						// Suppress unused variable warning
	}
}

/*
 * Writing a register does not need to respect the CANINTF register
 */
void mcp25625_writeRegister ( uint8_t reg, uint8_t value )
{
	// Transmit value to register
	setChipSelect();						// Enable CS
	LL_SPI_TransmitData8(SPI2, CMD_WRITE);	// Send write command
	LL_SPI_TransmitData8(SPI2, reg);		// Send register to write to
	LL_SPI_TransmitData8(SPI2, value);		// Transmit value to register
	while ( (SPI2->SR & SPI_SR_FTLVL) );	// Wait for Tx FIFO to be empty
	while ( (SPI2->SR & SPI_SR_BSY) ); 		// Wait for Tx FIFO to not be busy
	clearChipSelect();						// Disable CS

	// Clear Rx FIFO
	while ( (SPI2->SR & SPI_SR_FRLVL) )		// Check status register for data in Rx
	{
		uint8_t dummy = SPI2->DR;			// Clear Rx FIFO from the receives
		(void)dummy;						// Suppress unused variable warning
	}
}

/*
 * Reading a register does not need to respect the CANINTF register
 */
uint8_t mcp25625_readRegister (uint8_t reg)
{
	// Read register
	setChipSelect();						// Enable CS
	LL_SPI_TransmitData8(SPI2, CMD_READ);	// Send read command
	LL_SPI_TransmitData8(SPI2, reg);		// Send register to read from
	LL_SPI_TransmitData8(SPI2, 0x00);		// ??
	while ( (SPI2->SR & SPI_SR_FTLVL) );	// Wait for Tx FIFO to be empty
	while ( (SPI2->SR & SPI_SR_BSY) ); 		// Wait for Tx FIFO to not be busy
	uint8_t dummy  = SPI2->DR; 				// First byte is from CMD transfer
	dummy          = SPI2->DR;				// Second byte is from address
	uint8_t result = SPI2->DR;				// Actual result value
	clearChipSelect();						// Disable CS
	(void)dummy;							// Suppress unused variable warning

	// Return
	return result;
}

void mcp25625_loadTXB ( uint8_t reg, uint8_t length, uint8_t * data )
{
	setChipSelect();

	LL_SPI_TransmitData8(SPI2, reg);
	for ( uint8_t i = 0; i < length; i++ )
	{
		while ( !(SPI2->SR & SPI_SR_TXE) );
		LL_SPI_TransmitData8(SPI2, data[i]);
	}

	// wait for completion
	while ( (SPI2->SR & SPI_SR_FTLVL) ); 	// transmit fifo empty?
	while ( (SPI2->SR & SPI_SR_BSY) ); 		// no longer busy
	clearChipSelect();

	// clear receive fifo
	while ( (SPI2->SR & SPI_SR_FRLVL) )
	{
		uint8_t dummy = SPI2->DR; 			// clear rx fifo from the receives.
		(void)dummy;						// suppress unused variable warning
	}
}

/*
 * Abstraction for reading the CANINTF register
 */
__STATIC_INLINE canintf_t getCANINTF() {
	return (canintf_t)mcp25625_readRegister(CANINTF); }

/*
 * Abstraction for reading the TXB0CTRL register
 */
__STATIC_INLINE txb_ctrl_t getTXB0CTRL() {
	return (txb_ctrl_t)mcp25625_readRegister(TXB0CTRL); }

/*
 * Abstraction for reading the TXB1CTRL register
 */
__STATIC_INLINE txb_ctrl_t getTXB1CTRL() {
	return (txb_ctrl_t)mcp25625_readRegister(TXB1CTRL); }

/*
 * Abstraction for reading the TXB2CTRL register
 */
__STATIC_INLINE txb_ctrl_t getTXB2CTRL() {
	return (txb_ctrl_t)mcp25625_readRegister(TXB2CTRL); }

/*
 * Abstraction for reading the RXB1CTRL register
 */
__STATIC_INLINE rxb_ctrl_t getRXB1CTRL() {
	return (rxb_ctrl_t)mcp25625_readRegister(RXB1CTRL); }

/*
 * Abstraction for reading the RXB0CTRL register
 */
__STATIC_INLINE rxb_ctrl_t getRXB0CTRL() {
	return (rxb_ctrl_t)mcp25625_readRegister(RXB0CTRL); }

/* Abstraction for loading TXB2
 *
 */
__STATIC_INLINE void loadTXB2 (uint8_t * data) {
	mcp25625_loadTXB((CMD_LOAD_TX_BUFFER_BASE | TXB2_SIDH), 13, data); }

/* Abstraction for loading TXB1
 *
 */
__STATIC_INLINE void loadTXB1 (uint8_t * data) {
	mcp25625_loadTXB((CMD_LOAD_TX_BUFFER_BASE | TXB1_SIDH), 13, data); }

/* Abstraction for loading TXB2
 *
 */
__STATIC_INLINE void loadTXB0 (uint8_t * data) {
	mcp25625_loadTXB((CMD_LOAD_TX_BUFFER_BASE | TXB0_SIDH), 13, data); }

/*
 * Abstraction for setting TXREQ
 */
__STATIC_INLINE void setTXREQ2() {
	mcp25625_writeRegister(TXB2CTRL, TXBCTRL_TXREQ); }

/*
 * Abstraction for setting TXREQ
 */
__STATIC_INLINE void setTXREQ1() {
	mcp25625_writeRegister(TXB1CTRL, TXBCTRL_TXREQ); }

/*
 * Abstraction for setting TXREQ
 */
__STATIC_INLINE void setTXREQ0() {
	mcp25625_writeRegister(TXB0CTRL, TXBCTRL_TXREQ); }

/*
 * This must respect the TXB Control registers. Read it first!
 * By default, higher number buffer registers have higher priority.
 */
void mcp25625_sendCAN (txbuff_t * buffer)
{
	txb_ctrl_t txb;

	txb = getTXB2CTRL();
	// check if already sending
	if (!txb.bits.TXREQ)
	{
		// not sending, can load txb block
		loadTXB2(buffer->bytes);
		setTXREQ2();
		return; // once txbuffer is loaded, return
	}

	txb = getTXB1CTRL();
	// check if txb1 is already sending
	if (!txb.bits.TXREQ)
	{
		// not sending, can load txb buffer
		loadTXB1(buffer->bytes);
		setTXREQ1();
		return; //once tx buffer is loaded, return
	}

	txb = getTXB0CTRL();
	// check if txb0 is already sending
	if (!txb.bits.TXREQ)
	{
		// not sending, can load txb buffer
		loadTXB0(buffer->bytes);
		setTXREQ0();
		return; // once tx buffer is loaded, return
	}
}

/*
 * callable entry function
 * IN: ID, length of message, pointer to message
 */
void CAN_transmit ( uint16_t CAN_ID, uint8_t length, uint8_t * message )
{
	txbuff_t t;
	memset(&t, 0, sizeof(t));

	t.txb.SIDH = CAN_ID >> 3;
	t.txb.SIDL.value = (CAN_ID & 0x07) << 5;
	t.txb.DLC.value = length;
	for (uint8_t i = 0; i < length; i++)
	{
		t.txb.data[i] = message[i];
	}

	mcp25625_sendCAN(&t);
}

void mcp25625_init (void)
{
	memset(&can_tranceiver, 0, sizeof(can_tranceiver));
	clearChipSelect();
	mcp25625_reset();
}

void CAN_configure()
{
	cnf1_t c1;
	cnf2_t c2;
	cnf3_t c3;
	canctrl_t canctrl;

	c1.value      = 0xC0;
	c2.value      = 0x89;
	c3.value      = 0x04;
	canctrl.value = 0x08;

	mcp25625_init();

	mcp25625_writeRegister(CNF1, c1.value);				// Configuration 1 register
	mcp25625_writeRegister(CNF2, c2.value);				// Configuration 2 register
	mcp25625_writeRegister(CNF3, c3.value);				// Configuration 3 register
	mcp25625_writeRegister(CANCTRL, canctrl.value);		// CAN control register
}
