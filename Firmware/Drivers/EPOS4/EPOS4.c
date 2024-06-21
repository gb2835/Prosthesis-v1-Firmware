/*******************************************************************************
*
* TITLE   Driver for EPOS4 Motor Controller
* AUTHOR  Greg Berkeley
* RELEASE 04/30/2024
*
* NOTES
* 1. Unless otherwise specified, the following references are used:
* 		AN = Application Notes (Title: EPOS4 Application Notes,
*                              Document Number: rel8760,
*                              Revision: 2019-11)
* 2. Double question marks (??) are commented at locations where possible
*    improvements may be made.
*
*******************************************************************************/

// Include header files
#include "EPOS4.h"


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

void EPOS4_usDelay(uint32_t us);	// Can we lose this??


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

// Set Cyclic Synchronous Torque (CST) Mode (Table 7-71 of AN)
// NOTE: According to AN the below process should start at Set CST operation mode.
//       However, the device doesn't operate without first doing Shutdown and
//       then Switch on and enable device. Thus, those functions are added to the
//       beginning.
void EPOS4_SetCSTMode( uint16_t CAN_ID )
{
    uint8_t data[8];

    // Shutdown device
    EPOS4_DataFramer( data, 0x6040, 0x00, 0x06 );
    CAN_transmit( CAN_ID, 8, data );
    LL_mDelay(10);									// Can we do better??

    // Switch on and enable device
    EPOS4_DataFramer( data, 0x6040, 0x00, 0x0F );
    CAN_transmit( CAN_ID, 8, data );
    LL_mDelay(10);									// Can we do better??

    // Set CST operation mode
    EPOS4_DataFramer( data, 0x6060 , 0, 0x0A );
    CAN_transmit( CAN_ID, 8, data );
    LL_mDelay(10);									// Can we do better??

    // Shutdown device
    EPOS4_DataFramer( data, 0x6040, 0x00, 0x06 );
    CAN_transmit( CAN_ID, 8, data );
    LL_mDelay(10);									// Can we do better??

    // Switch on and enable device
    EPOS4_DataFramer( data, 0x6040, 0x00, 0x0F );
    CAN_transmit( CAN_ID, 8, data );
    LL_mDelay(10);									// Can we do better??
}

// Table 7-73 of AN
void EPOS4_ClearFault( uint16_t CAN_ID )
{
    uint8_t data[8];
    EPOS4_DataFramer(data, 0x6040, 0x00, 0x80);

    CAN_transmit(CAN_ID, 8, data);
}

// Set torque per thousand of motor rated torque (Table 7-71 of AN)
// Motor rated torque = nominal current * torque constant
// For example, 100 = 10% of motor rated torque
void EPOS4_SetTorque( uint16_t CAN_ID, int32_t torque )
{
    uint8_t data[8];

    EPOS4_DataFramer(data, 0x6071, 0x00, torque);

    CAN_transmit(CAN_ID, 8, data);
    EPOS4_usDelay(50);				// Can we do better??
}

// Table 7-72 of AN
void EPOS4_StopMotion(uint16_t CAN_ID)
{
    uint8_t data[8];
    EPOS4_DataFramer(data, 0x6071, 0x00, 0x00);

    CAN_transmit(CAN_ID, 8, data);
    EPOS4_usDelay(1500);			// Can we do better??
}

void EPOS4_DataFramer(uint8_t *data, uint16_t object, uint8_t subindex, uint32_t value)
{
    data[0] = 0x22; 					// [Byte 0] legend Table 5-43 page 5-55 of AN
    data[1] = (0x00 | object); 			// Index LowByte
    data[2] = (0x00 | (object >> 8)); 	// Index HighByte
    data[3] = subindex; 				// subindex
    data[4] = (0x00 | value); 			// SDO Byte 0
    data[5] = (0x00 | (value >> 8)); 	// SDO Byte 1
    data[6] = (0x00 | (value >> 16)); 	// SDO Byte 2
    data[7] = (0x00 | (value >> 24));	// SDO Byte 3
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

// Can we lose this??
void EPOS4_usDelay(uint32_t us)
{
    uint32_t i,k;
    for(k = 0 ; k < us; k++)
    {
    	for(i = 0; i < 11; i++)
         __NOP();
    }
}


/*******************************************************************************
* END
*******************************************************************************/
