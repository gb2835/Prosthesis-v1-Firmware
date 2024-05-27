/*******************************************************************************
 *
 * TITLE   Prosthesis Control
 * AUTHOR  Greg Berkeley
 * RELEASE XX/XX/XXXX
 *
 * NOTES
 * 1. None.
 *
 * ABSTRACT
 * The below code provides the functionality for prosthesis control.
 *
 ******************************************************************************/

#include "as5145b.h"
#include "EPOS4.h"
#include "prosthesis_control.h"
#include <math.h>
#include "mcp25625.h"
#include "mpu9255.h"
#include <stdint.h>
#include "stm32l4xx_ll_adc.h"


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

uint8_t isProsthesisControlRequired = 0;
uint16_t CAN_ID = 0x601;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

enum TestPrograms_e testProgram;
float Kp;
float Kd;
float equilibriumPoint_deg;

uint8_t isFirst = 1;
uint8_t isSecond = 0;
uint8_t isTestProgramRequired = 0;

// For CubeMonitor
float CM_angle_deg[2];					// [0] = k-0, [1] = k-1
float CM_jointTorque_nm;
float CM_speed_dps = 0.0f;
uint16_t CM_average_MagEnc;
uint16_t CM_loadCell_bot[3];			// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_loadCell_top[3];			// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_loadCell_top_filtered[3];	// [0] = k-0, [1] = k-1, [2] = k-2 (float or uint??)
uint16_t CM_loadCell_bot_filtered[3];	// [0] = k-0, [1] = k-1, [2] = k-2 (float or uint??)

static void GetInputs (void);
static void RunStateMachine (void);
static void SetOutputs (void);
static void RunImpedanceControl (void);
static void RunTestProgram (void);
static uint16_t ReadLoadCell ( ADC_TypeDef *ADCx );


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void RunProsthesisControl (void)
{
	GetInputs();

	// If test program is required, run test program
	// Otherwise continue prosthesis control
	if (isTestProgramRequired)
	{
		RunTestProgram();
	}
	else
	{
		RunStateMachine();
		SetOutputs();
	}
}

void RequireTestProgram ( enum TestPrograms_e option )
{
	testProgram = option;

	if ( testProgram != None)
		isTestProgramRequired = 1;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void GetInputs (void)
{
	float tau = 1 / ( 2 * 3.1416f * 10 );			// Time constant
	float dt = 1 / 512.0f;							// Sample time
	float encBias_deg = 1325.0f * 360.0f/4096.0f;	// Bias found using RunTestProgram below

	CM_angle_deg[0]	= AS5145B_ReadPosition_Deg() - encBias_deg;
	CM_loadCell_bot[0] = ReadLoadCell(ADC2);
	CM_loadCell_top[0] = ReadLoadCell(ADC1);

	// No derivative of angle (angular speed) on first execution
	// No filtering for load cells on first or second execution
	if (isFirst)
	{
		CM_speed_dps = 0;

		CM_angle_deg[1] = CM_angle_deg[0];
		CM_loadCell_bot[2] = CM_loadCell_bot[0];
		CM_loadCell_top[2] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[0] = CM_loadCell_bot[0];
		CM_loadCell_top_filtered[0] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[2] = CM_loadCell_bot_filtered[0];
		CM_loadCell_top_filtered[2] = CM_loadCell_top_filtered[0];

		isFirst = 0;
		isSecond = 1;
	}
	else if (isSecond)
	{
		// Practical differentiator (fc = 10 Hz, bilinear transformation used)
		CM_speed_dps = ( 2*( CM_angle_deg[0] - CM_angle_deg[1] ) + ( 2*tau - dt )*CM_speed_dps ) / ( dt + 2*tau );

		CM_angle_deg[1] = CM_angle_deg[0];
		CM_loadCell_bot[1] = CM_loadCell_bot[0];
		CM_loadCell_top[1] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[0] = CM_loadCell_bot[0];
		CM_loadCell_top_filtered[0] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[1] = CM_loadCell_bot_filtered[0];
		CM_loadCell_top_filtered[1] = CM_loadCell_top_filtered[0];

		isSecond = 0;
	}
	else
	{
		// Practical differentiator (fc = 10 Hz, bilinear transformation used)
		CM_speed_dps = ( 2*( CM_angle_deg[0] - CM_angle_deg[1] ) + ( 2*tau - dt )*CM_speed_dps ) / ( dt + 2*tau );

		// 2nd order low-pass Butterworth (fc = 20 Hz)
		CM_loadCell_bot_filtered[0] =   1.6556f * CM_loadCell_bot_filtered[1] - 0.7068f * CM_loadCell_bot_filtered[2]
									  + 0.0128f * CM_loadCell_bot[0] + 0.0256f * CM_loadCell_bot[1] + 0.0128f * CM_loadCell_bot[2];
		CM_loadCell_top_filtered[0] =   1.6556f * CM_loadCell_top_filtered[1] - 0.7068f * CM_loadCell_top_filtered[2]
									  + 0.0128f * CM_loadCell_top[0] + 0.0256f * CM_loadCell_top[1] + 0.0128f * CM_loadCell_top[2];

		CM_angle_deg[1] = CM_angle_deg[0];
		CM_loadCell_bot[2] = CM_loadCell_bot[1];
		CM_loadCell_top[2] = CM_loadCell_top[1];
		CM_loadCell_bot[1] = CM_loadCell_bot[0];
		CM_loadCell_top[1] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[2] = CM_loadCell_bot[1];
		CM_loadCell_top_filtered[2] = CM_loadCell_top[1];
		CM_loadCell_bot_filtered[2] = CM_loadCell_bot_filtered[1];
		CM_loadCell_top_filtered[2] = CM_loadCell_top_filtered[1];
		CM_loadCell_bot_filtered[1] = CM_loadCell_bot[0];
		CM_loadCell_top_filtered[1] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[1] = CM_loadCell_bot_filtered[0];
		CM_loadCell_top_filtered[1] = CM_loadCell_top_filtered[0];
	}

	mpu9255_process();
}

static void RunStateMachine (void)
{
	switch (0)
	{
	case 0:
		Kp = 2.5;
		Kd = 0;
		equilibriumPoint_deg = 0;
		break;
	}
}

static void SetOutputs (void)
{

}

static void RunImpedanceControl (void)
{
	float gearRatio = 40;
	float torqueConst_nmpa = 0.095f;	// is this number accurate??
	float nomCurrent_amp = 8;			// is this number accurate??

	float errorPos_deg = equilibriumPoint_deg - CM_angle_deg[0];
	CM_jointTorque_nm = Kp*errorPos_deg - Kd*CM_speed_dps;
	int32_t motTorque = CM_jointTorque_nm / ( torqueConst_nmpa * gearRatio * nomCurrent_amp ) * 1000.0f;
	EPOS4_SetTorque( CAN_ID, motTorque );
}

static void RunTestProgram (void)
{
	switch (testProgram)
	{
	case None:
		break;
	case ReadOnly:
		break;
	case ConstantTorque:
	{
		int32_t torque = 150;
		EPOS4_SetTorque( CAN_ID, torque );
		break;
	}
	case AverageMagEnc:
	{
		uint16_t i;
		uint32_t sum = 0;

		for ( i = 0; i < 1000; i++ )
		{
			struct AS5145B_Data_s data  = AS5145B_ReadData();
			sum                        += data.pos;
		}

		CM_average_MagEnc = sum / i;
		(void) CM_average_MagEnc;

		while (1);	// Halt program
	}
	case ImpedanceControl:
	{
		// First compute average of current position and use as equilibrium point
		// Then run impedance control
		if (isFirst)
		{
			uint16_t i;

			Kp = 2.5;
			Kd = 0;
			uint32_t sum = 0;

			for ( i = 0; i < 1000; i++ )
			{
				struct AS5145B_Data_s data = AS5145B_ReadData();
				sum += data.pos;
			}

			equilibriumPoint_deg = (float) sum/i * 360/4096;

			isFirst = 0;
		}
		else
		{
			RunImpedanceControl();
		}

		break;
	}
	}
}

// move to driver??
static uint16_t ReadLoadCell ( ADC_TypeDef *ADCx )
{
	LL_ADC_REG_StartConversion(ADCx);
	while ( !LL_ADC_IsActiveFlag_EOC(ADCx) );				// change to EOC??
	LL_ADC_ClearFlag_EOC(ADCx);								// change to EOC?? remove this??
	uint16_t val = LL_ADC_REG_ReadConversionData12(ADCx);	// change resolution??
	return val;
}


/*******************************************************************************
* END
*******************************************************************************/
