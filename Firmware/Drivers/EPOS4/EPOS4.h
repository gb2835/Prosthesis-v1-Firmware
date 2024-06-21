/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#include "mcp25625.h"


/*******************************************************************************
 * PUBLIC DEFINTIONS
 ******************************************************************************/

void EPOS4_SetCSTMode(uint16_t CAN_ID);
void EPOS4_ClearFault(uint16_t CAN_ID);
void EPOS4_SetTorque(uint16_t CAN_ID, int32_t torque);
void EPOS4_StopMotion(uint16_t CAN_ID);
void EPOS4_DataFramer(uint8_t *data, uint16_t object, uint8_t subindex, uint32_t value);
