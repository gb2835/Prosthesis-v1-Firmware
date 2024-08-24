/*******************************************************************************
*
* TITLE   Prosthesis Control
* AUTHOR  Greg Berkeley
* RELEASE XX/XX/XXXX
*
* NOTES
* 1. None.
*
*******************************************************************************/

#include "as5145b.h"
#include "EPOS4.h"
#include "prosthesis_control.h"
#include <math.h>
#include "mcp25625.h"
#include "mpu925x_spi.h"
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

// Can we do better??
#define KNEE
#define LEFT

enum StateMachine_e
{
	Stance,
	SwingFlexion,
	SwingExtension
};

struct ControlParams_s
{
	float eqPoint;	// Units in degrees
	float kd;		// Units in N*m/(deg/s)
	float kp;		// Units in N*m/deg
};

struct LoadCell_Data_s
{
	double bot[3];
	double top[3];
};

static enum TestPrograms_e testProgram;
struct ControlParams_s ProsCtrl;
struct MPU925x_IMUData_s IMUData;

float dt = 1 / 512.0f;
uint8_t isFirst = 1;
uint8_t isSecond = 0;
uint8_t isTestProgramRequired = 0;

#ifdef KNEE
float encBias = 2244 * AS5145B_RAW2DEG;
#endif
#ifdef ANKLE
float encBias = 1325 * AS5145B_RAW2DEG;
#endif

// For CubeMonitor
double CM_limbAngle;
double CM_jointAngle[2];											// [0] = k-0, [1] = k-1
double CM_jointSpeed = 0.0f;
float CM_jointTorque;
float CM_lcBot_staticUpperLimit, CM_lcTop_staticUpperLimit;
struct ControlParams_s CM_ImpCtrl, CM_StanceCtrl, CM_SwingFlexCtrl, CM_SwingExtCtrl;
struct MPU925x_IMUData_s CM_IMUData;
struct LoadCell_Data_s CM_LoadCell[3], CM_LoadCell_Filtered[3];		// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_magEncBias;
uint16_t CM_state;

void GetInputs(void);
uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
void ProcessInputs(void);
void CalibrateIMU(void);
void ComputeLimbAngle(void);
void RunStateMachine(void);
void RunImpedanceControl(void);
void RunTestProgram(void);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

// This is to localize variables that are subject to change during testing
void InitProsthesisControl(void)
{
	CM_ImpCtrl.kd = 0.0f;
	CM_ImpCtrl.kp = 0.0f;
	CM_StanceCtrl.eqPoint = 0.0f;		// Vanderbilt = -4.99 deg
	CM_StanceCtrl.kd = 0.0f;			// Vanderbilt = 0 N*m/(deg/s)
	CM_StanceCtrl.kp = 0.0f;			// 2.50 used to keep heat down in EPOS, Vanderbilt = 4.97 N*m/deg
	CM_SwingFlexCtrl.eqPoint = 0.0f;	// Vanderbilt = -35.0 deg
	CM_SwingFlexCtrl.kd = 0.0f;			// 0.05 used to get zero overshoot and 0.5 sec settling time, Vanderbilt = 0 N*m/(deg/s)
	CM_SwingFlexCtrl.kp = 0.0f;			// 0.45 on the bench "feels" right, Vanderbilt = 0.65 N*m/deg
	CM_SwingExtCtrl.eqPoint = 0.f;
	CM_SwingExtCtrl.kd = 0.0f;
	CM_SwingExtCtrl.kp = 0.0f;

	CM_lcBot_staticUpperLimit = 2300.0f;
	CM_lcTop_staticUpperLimit = 2300.0f;
}

void RequireTestProgram(enum TestPrograms_e option)
{
	testProgram = option;

	if(testProgram != None)
		isTestProgramRequired = 1;
}

void RunProsthesisControl(void)
{
	GetInputs();
	ProcessInputs();

	if(isTestProgramRequired)
	{
		RunTestProgram();
	}
	else
	{
		RunStateMachine();
		RunImpedanceControl();
	}

	// Check for first and second executions, needed for derivatives, filters, etc.
	if(isFirst)
	{
		isFirst = 0;
		isSecond = 1;
	}
	else if(isSecond)
	{
		isSecond = 0;
	}
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

void GetInputs(void)
{
	CM_jointAngle[0] = AS5145B_ReadPosition_Deg() - encBias;
	CM_LoadCell->bot[0] = ReadLoadCell(ADC2);
	CM_LoadCell->top[0] = ReadLoadCell(ADC1);
	IMUData = MPU925x_ReadIMU();
}

// Should be moved to ADC driver??
uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while ( !LL_ADC_IsActiveFlag_EOC(ADCx) );
	LL_ADC_ClearFlag_EOC(ADCx);								// remove this??
	uint16_t val = LL_ADC_REG_ReadConversionData12(ADCx);	// Change resolution??
	return val;
}

void ProcessInputs(void)
{
	double tau = 1.0 / (2 * M_PI * 10);		// Time constant for practical differentiator (fc = 10 Hz)

	// Derivative of angle and filtering of load cells
	// No derivative of angle (angular speed) on first execution
	// No filtering of load cells on first or second execution
	if(isFirst)
	{
		CM_jointSpeed = 0.0;

		CM_jointAngle[1] = CM_jointAngle[0];
		CM_LoadCell->bot[2] = CM_LoadCell->bot[0];
		CM_LoadCell->top[2] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[0] = CM_LoadCell->bot[0];
		CM_LoadCell_Filtered->top[0] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[2] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[2] = CM_LoadCell_Filtered->top[0];
	}
	else if(isSecond)
	{
		// Practical differentiator (bilinear transformation used)
		CM_jointSpeed = (2*(CM_jointAngle[0] - CM_jointAngle[1]) + (2*tau - dt)*CM_jointSpeed) / (dt + 2*tau);

		CM_jointAngle[1] = CM_jointAngle[0];
		CM_LoadCell->bot[1] = CM_LoadCell->bot[0];
		CM_LoadCell->top[1] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[0] = CM_LoadCell->bot[0];
		CM_LoadCell_Filtered->top[0] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[1] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[1] = CM_LoadCell_Filtered->top[0];
	}
	else
	{
		// Practical differentiator (bilinear transformation used)
		CM_jointSpeed = (2*(CM_jointAngle[0] - CM_jointAngle[1]) + (2*tau - dt)*CM_jointSpeed) / (dt + 2*tau);

		// 2nd order low-pass Butterworth (fc = 20 Hz)
		CM_LoadCell_Filtered->bot[0] =   1.6556 * CM_LoadCell_Filtered->bot[1] - 0.7068 * CM_LoadCell_Filtered->bot[2]
									   + 0.0128 * CM_LoadCell->bot[0] + 0.0256 * CM_LoadCell->bot[1] + 0.0128 * CM_LoadCell->bot[2];
		CM_LoadCell_Filtered->top[0] =   1.6556 * CM_LoadCell_Filtered->top[1] - 0.7068 * CM_LoadCell_Filtered->top[2]
									   + 0.0128 * CM_LoadCell->top[0] + 0.0256 * CM_LoadCell->top[1] + 0.0128 * CM_LoadCell->top[2];

		CM_jointAngle[1] = CM_jointAngle[0];
		CM_LoadCell->bot[2] = CM_LoadCell->bot[1];
		CM_LoadCell->bot[1] = CM_LoadCell->bot[0];
		CM_LoadCell->top[2] = CM_LoadCell->top[1];
		CM_LoadCell->top[1] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[2] = CM_LoadCell_Filtered->bot[1];
		CM_LoadCell_Filtered->bot[1] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[2] = CM_LoadCell_Filtered->top[1];
		CM_LoadCell_Filtered->top[1] = CM_LoadCell_Filtered->top[0];
	}

	CalibrateIMU();
	ComputeLimbAngle();
}

void CalibrateIMU(void)
{
	double axBias = 0.0;
	double ayBias = 0.0;
	double azBias = 0.0;
	double gxBias = 0.0;
	double gyBias = 0.0;
	double gzBias = 0.0;
	double n = 1.0;

	// Sine and cosine of Euler angles (1 = z angle, 2 = x' angle, 3 = z'' angle)
	#ifdef RIGHT
	double c1 = cos(0.0);
	double c2 = cos(0.0);
	double c3 = cos(0.0);
	double s1 = sin(0.0);
	double s2 = sin(0.0);
	double s3 = sin(0.0);
	#endif
	#ifdef LEFT
	double c1 = cos(M_PI);
	double c2 = cos(M_PI);
	double c3 = cos(0.0);
	double s1 = sin(M_PI);
	double s2 = sin(M_PI);
	double s3 = sin(0.0);
	#endif

	// Rotate IMU data and remove biases
	CM_IMUData.ax = n * (IMUData.ax*(c1*c3 - c2*s1*s3) + IMUData.ay*(  -c3*s1 - c1*c2*s3) + IMUData.az*( s2*s3)) - axBias;
	CM_IMUData.ay = n * (IMUData.ax*(c1*s3 + c2*c3*s1) + IMUData.ay*(c1*c2*c3 - s1*s3   ) + IMUData.az*(-c3*s2)) - ayBias;
	CM_IMUData.az = n * (IMUData.ax*(        s1*s2   ) + IMUData.ay*(           c1*s2   ) + IMUData.az*( c2   )) - azBias;
	CM_IMUData.gx = n * (IMUData.gx*(c1*c3 - c2*s1*s3) + IMUData.gy*(  -c3*s1 - c1*c2*s3) + IMUData.gz*( s2*s3)) - gxBias;
	CM_IMUData.gy = n * (IMUData.gx*(c1*s3 + c2*c3*s1) + IMUData.gy*(c1*c2*c3 - s1*s3   ) + IMUData.gz*(-c3*s2)) - gyBias;
	CM_IMUData.gz = n * (IMUData.gx*(        s1*s2   ) + IMUData.gy*(           c1*s2   ) + IMUData.gz*( c2   )) - gzBias;
}

void ComputeLimbAngle(void)
{
	double accelAngle = (atan(CM_IMUData.ax / sqrt(pow(CM_IMUData.ay, 2) + pow(CM_IMUData.az, 2)))) * 180.0/M_PI;
	static double compFiltAngle = 0.0;
	static double dGyroAngle = 0.0;

	// Change in angle from gyro (trapezoidal used)
	dGyroAngle = dt/2 * (CM_IMUData.gz + dGyroAngle);

	// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
	double alpha = 0.002;
	compFiltAngle = accelAngle*alpha + (1 - alpha) * (dGyroAngle + compFiltAngle);

	#ifdef KNEE
	CM_limbAngle = compFiltAngle - CM_jointAngle[0];
	#endif
	#ifdef ANKLE
	CM_limbAngle = compFiltAngle + CM_jointAngle[0];
	#endif
}

void RunStateMachine(void)
{
	static enum StateMachine_e state;

	if(isFirst)
	{
		state = Stance;
	}

	switch(state)
	{
	case Stance:
		CM_state = 1800;
		ProsCtrl.eqPoint = CM_StanceCtrl.eqPoint;
		ProsCtrl.kd = CM_StanceCtrl.kd;
		ProsCtrl.kp = CM_StanceCtrl.kp;

		if((CM_LoadCell_Filtered->top[0] < CM_lcTop_staticUpperLimit) && (CM_LoadCell_Filtered->bot[0] < CM_lcBot_staticUpperLimit))
		{
			state = SwingFlexion;
		}

		break;

	case SwingFlexion:
		CM_state = 2400;
		ProsCtrl.eqPoint = CM_SwingFlexCtrl.eqPoint;
		ProsCtrl.kd = CM_SwingFlexCtrl.kd;
		ProsCtrl.kp = CM_SwingFlexCtrl.kp;

		if(CM_jointSpeed > 0)
		{
			state = SwingExtension;
		}

		break;

	case SwingExtension:
		CM_state = 2900;
		ProsCtrl.eqPoint = CM_SwingExtCtrl.eqPoint;
		ProsCtrl.kd = CM_SwingExtCtrl.kd;
		ProsCtrl.kp = CM_SwingExtCtrl.kp;

		if(CM_LoadCell_Filtered->bot[0] > CM_lcBot_staticUpperLimit)
		{
			state = Stance;
		}

		break;
	}
}

void RunImpedanceControl(void)
{
	float gearRatio = 40.0f;
	float nomCurrent = 8.0f;						// is this number accurate??
	float torqueConst = 60.0f / (2*M_PI * 100);		// Units in N*m/A, for Kv = 100 rpm/V

	float errorPos = ProsCtrl.eqPoint - CM_jointAngle[0];

	#ifdef KNEE
	CM_jointTorque = -(ProsCtrl.kp*errorPos - ProsCtrl.kd*CM_jointSpeed);
	#endif
	#ifdef ANKLE
	CM_jointTorque_nm = ProsCtrl.kp*errorPos - ProsCtrl.kd*CM_jointSpeed;
	#endif

	int32_t motorTorque = CM_jointTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
	EPOS4_SetTorque(CAN_ID, motorTorque);
}

void RunTestProgram(void)
{
	switch (testProgram)
	{
	case None:
		break;
	case ReadOnly:
		break;
	case ConstantTorque:
	{
		int32_t torque = 100;
		EPOS4_SetTorque(CAN_ID, torque);
		break;
	}
	case MagneticEncoderBias:
	{
		uint16_t i;

		uint32_t sum = 0;

		for(i = 0; i < 1000; i++)
		{
			uint16_t bias = AS5145B_ReadPosition_Raw();
			sum += bias;
		}

		CM_magEncBias = sum / i;

		break;
	}
	case ImpedanceControl:
	{
		// First compute average of current position and use as equilibrium point
		// Then run impedance control
		if (isFirst)
		{
			uint16_t i;
			float sum = 0.0f;

			for(i = 0; i < 1000; i++)
			{
				float pos = AS5145B_ReadPosition_Deg();
				sum += pos;
			}

			CM_ImpCtrl.eqPoint = sum / i - encBias;
		}
		else
		{
			ProsCtrl.kd = CM_ImpCtrl.kd;
			ProsCtrl.kp = CM_ImpCtrl.kp;
			ProsCtrl.eqPoint = CM_ImpCtrl.eqPoint;

			RunImpedanceControl();
		}

		break;
	}
	}
}


/*******************************************************************************
* END
*******************************************************************************/
