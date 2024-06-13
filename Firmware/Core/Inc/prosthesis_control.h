/*******************************************************************************
 *
 * TITLE   Prosthesis Control
 * AUTHOR  Greg Berkeley
 * RELEASE XX/XX/XXXX
 *
 * NOTES
 * 1. See source file for more information.
 *
 ******************************************************************************/

#ifndef INC_PROSTHESIS_CONTROL_H_
#define INC_PROSTHESIS_CONTROL_H_


/*******************************************************************************
 * PUBLIC DEFINITIONS
 ******************************************************************************/

enum TestPrograms_e
{
	None,				// No test program runs
	ReadOnly,
	ConstantTorque,
	MagneticEncoderBias,
	ImpedanceControl
};

extern uint16_t CAN_ID;
extern uint8_t isProsthesisControlRequired;

void RunProsthesisControl(void);
void RequireTestProgram(enum TestPrograms_e option);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_CONTROL_H_ */
