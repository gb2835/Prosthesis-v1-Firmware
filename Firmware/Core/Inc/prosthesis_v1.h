/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_PROSTHESIS_V1_H_
#define INC_PROSTHESIS_V1_H_

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
	Ankle,
	Combined,
	Knee
} Joint_e;

typedef enum
{
	AnkleMotorControllerIndex,
	KneeMotorControllerIndex
} MotorControllerIndex_e;

typedef enum
{
	Left,
	Right
} Side_e;

typedef enum
{
	None,
	ReadOnly,
	EncoderBias,
	ImpedanceControl
} TestProgram_e;

typedef struct
{
	Side_e Side;
	Joint_e Joint;
} Prosthesis_Init_t;

extern uint8_t isProsthesisControlRequired;

void InitProsthesisControl(Prosthesis_Init_t *Device_Init);
void RunProsthesisControl(void);
void RequireTestProgram(TestProgram_e testProgram);


/*******************************************************************************
 * END
 ******************************************************************************/

#endif /* INC_PROSTHESIS_V1_H_ */
