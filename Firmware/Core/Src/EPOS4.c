#include "EPOS4.h"

// Can we lose this??
static void delay_us(uint32_t us)
{
    uint32_t i,k;
    for(k=0;k<us;k++)
    {
    	for(i=0;i<11;i++)
         __NOP();  // Timed at 48 MHz clock
    }
}

// This is useful for later adding in functionality. Should work for any Client to Server SDO
void EPOS4_data_framer(uint8_t * data, uint16_t object, uint8_t subindex, uint32_t value)
{
    data[0] = 0x22; 					// [Byte 0] legend Table 5-43 page 5-55 Application Notes
    data[1] = (0x00 | object); 			// Index LowByte
    data[2] = (0x00 | (object >> 8)); 	// Index HighByte
    data[3] = subindex; 				// subindex
    data[4] = (0x00 | value); 			// SDO Byte 0
    data[5] = (0x00 | (value >> 8)); 	// SDO Byte 1
    data[6] = (0x00 | (value >> 16)); 	// SDO Byte 2
    data[7] = (0x00 | (value >> 24));	// SDO Byte 3
}

void EPOS4_set_operation_mode(uint16_t CAN_ID, uint32_t mode)
{
    uint8_t data[8];

    EPOS4_data_framer(data, 0x6060 , 0, mode);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
//    HAL_Delay(10);
}

void EPOS4_enable(uint16_t CAN_ID)
{
    uint8_t data[8];
    EPOS4_data_framer(data, 0x6040, 0x00, 0x06);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
//    HAL_Delay(10);

    EPOS4_enable2(CAN_ID);
}


void EPOS4_enable2(uint16_t CAN_ID)
{
    uint8_t data[8];

    EPOS4_data_framer(data, 0x6040, 0x00, 0x0F);
    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
//    HAL_Delay(10);
}

void EPOS4_PVM_set_velocity(uint16_t CAN_ID, uint32_t rpm)
{
    uint8_t data[8];

    EPOS4_data_framer(data, 0x60FF, 0x00, rpm);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
//    HAL_Delay(10);
}

void EPOS4_PVM_start(uint16_t CAN_ID)
{
    uint8_t data[8];

    EPOS4_data_framer(data, 0x6040, 0x00, 0x0F);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
//    HAL_Delay(10);
}


void EPOS4_PVM_stop(uint16_t CAN_ID)
{
    uint8_t data[8];
    EPOS4_data_framer(data, 0x6040, 0x00, 0x010F);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(10000);
//    HAL_Delay(10);
}


void EPOS4_CST_apply_torque(uint16_t CAN_ID, uint32_t torque)
{
    uint8_t data[8];

    EPOS4_data_framer(data, 0x6071, 0x00, torque);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(50); //1500
}

void EPOS4_CST_stop(uint16_t CAN_ID)
{
    uint8_t data[8];
    EPOS4_data_framer(data, 0x6071, 0x00, 0x00);

    CAN_transmit(CAN_ID, 8, data);
    delay_us(1500);
}

void EPOS4_clear_errors(uint16_t CAN_ID)
{
    uint8_t data[8];
    EPOS4_data_framer(data, 0x6040, 0x00, 0x80);

    CAN_transmit(CAN_ID, 8, data);
}
