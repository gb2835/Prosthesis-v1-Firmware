//#include "stm32l4xx_hal.h"
#include "mcp25625.h"	// Can we lose this??
//#include "sensor.h"
//#include "CAN.h"

// Fills out the can data array with the standard Object, Subindex, Value system used by EPOS4
void EPOS4_data_framer(uint8_t * data, uint16_t object, uint8_t subindex, uint32_t value);

// These are Universal
void EPOS4_set_operation_mode(uint16_t CAN_ID, uint32_t mode);
void EPOS4_enable(uint16_t CAN_ID);
void EPOS4_enable2(uint16_t CAN_ID);

// Related only to PVM
void EPOS4_PVM_set_velocity(uint16_t CAN_ID, uint32_t rpm);
void EPOS4_PVM_start(uint16_t CAN_ID);
void EPOS4_PVM_stop(uint16_t CAN_ID);
void EPOS4_clear_errors(uint16_t CAN_ID);
void EPOS4_CST_stop(uint16_t CAN_ID);
void EPOS4_CST_apply_torque(uint16_t CAN_ID, uint32_t torque);
