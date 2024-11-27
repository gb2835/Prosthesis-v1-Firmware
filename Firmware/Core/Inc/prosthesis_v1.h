/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_PROSTHESIS_CONTROL_H_
#define INC_PROSTHESIS_CONTROL_H_

typedef enum
{
	AnkleCAN_ControllerIndex,
	KneeCAN_ControllerIndex
} CAN_ControllerIndex_e;

typedef enum
{
	AnkleEncoderIndex,
	KneeEncoderIndex
} EncoderIndex_e;

typedef enum
{
	AnkleMotorControllerIndex,
	KneeMotorControllerIndex
} MotorControllerIndex_e;

typedef enum
{
	none,
	readOnly,
	constantMotorTorque100Nmm,
	magneticEncoderBias,
	impedanceControl
} TestPrograms_t;

typedef struct
{
	enum
	{
		left,
		right
	} Side;
	enum
	{
		ankle,
		combined,
		knee
	} Joint;
	uint8_t ankleMotorId;
	uint8_t kneeMotorId;
} Prosthesis_t;

extern uint8_t isProsthesisControlRequired;

void InitProsthesisControl(Prosthesis_t *Options);
void RunProsthesisControl(void);
void RequireTestProgram(TestPrograms_t option);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_CONTROL_H_ */
