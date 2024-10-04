/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_PROSTHESIS_CONTROL_H_
#define INC_PROSTHESIS_CONTROL_H_

enum TestPrograms_e
{
	None,
	ReadOnly,
	ConstantMotorTorque100nm,
	MagneticEncoderBias,
	ImpedanceControl
};

struct Configuration_s
{
	enum Side_e
	{
		Left,
		Right
	} Side;
	enum Device_e
	{
		Ankle,
		Combined,
		Knee
	} Device;
};

extern uint16_t kneeCANID;
extern uint8_t isProsthesisControlRequired;

void InitProsthesisControl(struct Configuration_s option);
void RunProsthesisControl(void);
void RequireTestProgram(enum TestPrograms_e option);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_CONTROL_H_ */
