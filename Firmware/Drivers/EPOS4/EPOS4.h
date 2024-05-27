/*******************************************************************************
 *
 * NOTES
 * 1. Unless otherwise specified, the following references are used:
 * 		AN = Application Notes (Title: EPOS4 Application Notes,
 *                              Document Number: rel8760,
 *                              Revision: 2019-11)
 *
 ******************************************************************************/

// Include header files
#include "mcp25625.h"	// Provides SPI to CAN functionality


/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

void EPOS4_SetCSTMode( uint16_t CAN_ID );														// Set Cyclic Synchronous Torque (CST) Mode (Table 7-71 of AN)
void EPOS4_ClearFault( uint16_t CAN_ID );														// Clear fault (Table 7-73 of AN)
void EPOS4_SetTorque( uint16_t CAN_ID, int32_t torque );										// Set torque per thousand of motor rated torque (Table 7-71 of AN)
void EPOS4_StopMotion( uint16_t CAN_ID );														// Stop motion of motor (Table 7-72 of AN)
void EPOS4_DataFramer( uint8_t * data, uint16_t object, uint8_t subindex, uint32_t value );		// Fills out the can data array with the standard Object, Subindex, Value system used by EPOS4
void EPOS4_usDelay( uint32_t us );																// Can we lose this??
