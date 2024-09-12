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
#include <stdint.h>
#include "stm32l4xx_ll_adc.h"
#include "mpu925x_spi.h"


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

uint8_t isProsthesisControlRequired = 0;
uint16_t ankleCANID = 0x00;
uint16_t kneeCANID = 0x601;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

enum StateMachine_e
{
	Waiting,
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

struct DeviceParams_s
{
	double jointAngle[2];	// [0] = k-0, [1] = k-1
	double jointSpeed;
	double limbAngle;
	float jointTorque;
	struct ControlParams_s ImpCtrl;
	struct ControlParams_s StanceCtrl;
	struct ControlParams_s SwingFlexCtrl;
	struct ControlParams_s SwingExtCtrl;
	struct MPU925x_IMUData_s IMUData;
};

struct LoadCell_Data_s
{
	double bot[3];
	double top[3];
};

enum TestPrograms_e testProgram;
float ankleEncBias, kneeEncBias;
struct Configuration_s config;
struct ControlParams_s AnkleProsCtrl, KneeProsCtrl, ProsCtrl;
struct MPU925x_IMUData_s AnkleIMUData, KneeIMUData;

double dt = 1 / 512.0;
uint8_t isFirst = 1;
uint8_t isInit = 0;
uint8_t isSecond = 0;
uint8_t isSimulatedWalkingRequired = 0;
uint8_t isTestProgramRequired = 0;

float CM_lcBot_staticUpperLimit, CM_lcTop_staticUpperLimit;
struct DeviceParams_s CM_Ankle, CM_Knee;
struct LoadCell_Data_s CM_LoadCell[3], CM_LoadCell_Filtered[3];		// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_ankleEncBias, CM_kneeEncBias;
uint16_t CM_state;

float CM_gain = 0.0f;
uint8_t CM_start = 0;

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

// Includes variables that are subject to change during testing for convenience
void InitProsthesisControl(struct Configuration_s option)
{
	config.Device = option.Device;
	config.Side = option.Side;

	ankleEncBias = 1325 * AS5145B_RAW2DEG;
	kneeEncBias = 2244 * AS5145B_RAW2DEG;

	CM_Ankle.ImpCtrl.eqPoint = 0.0f;
	CM_Ankle.ImpCtrl.kd = 0.0f;
	CM_Ankle.ImpCtrl.kp = 0.0f;
	CM_Ankle.StanceCtrl.eqPoint = 0.0f;
	CM_Ankle.StanceCtrl.kd = 0.0f;
	CM_Ankle.StanceCtrl.kp = 0.0f;
	CM_Ankle.SwingFlexCtrl.eqPoint = 0.0f;
	CM_Ankle.SwingFlexCtrl.kd = 0.0f;
	CM_Ankle.SwingFlexCtrl.kp = 0.0f;
	CM_Ankle.SwingExtCtrl.eqPoint = 0.0f;
	CM_Ankle.SwingExtCtrl.kd = 0.0f;
	CM_Ankle.SwingExtCtrl.kp = 0.0f;

	CM_Knee.ImpCtrl.eqPoint = 0.0f;
	CM_Knee.ImpCtrl.kd = 0.0f;
	CM_Knee.ImpCtrl.kp = 0.0f;
	CM_Knee.StanceCtrl.eqPoint = -8.0f;
	CM_Knee.StanceCtrl.kd = 0.05f;
	CM_Knee.StanceCtrl.kp = 2.5f;
	CM_Knee.SwingFlexCtrl.eqPoint = -65.0f;
	CM_Knee.SwingFlexCtrl.kd = 0.02f;
	CM_Knee.SwingFlexCtrl.kp = 0.15f;
	CM_Knee.SwingExtCtrl.eqPoint = -40.0f;
	CM_Knee.SwingExtCtrl.kd = 0.03f;
	CM_Knee.SwingExtCtrl.kp = 0.1f;

	CM_lcBot_staticUpperLimit = 2300.0f;
	CM_lcTop_staticUpperLimit = 2300.0f;

	isInit = 1;
}

void RequireTestProgram(enum TestPrograms_e option)
{
	testProgram = option;

	if(testProgram != None)
	{
		isTestProgramRequired = 1;
		if(testProgram == SimulatedWalking)
			isSimulatedWalkingRequired = 1;
	}
}

void RunProsthesisControl(void)
{
	if(!isInit)
		while(1);	// InitProsthesisControl() must be performed

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
	// Differentiate between two IMUs??
	if((config.Device == Ankle) || (config.Device == Combined))
	{
		CM_Ankle.jointAngle[0] = AS5145B_ReadPosition_Deg() - ankleEncBias;
		AnkleIMUData = MPU925x_ReadIMU();
	}
	else if((config.Device == Knee) || (config.Device == Combined))
	{
		CM_Knee.jointAngle[0] = AS5145B_ReadPosition_Deg() - kneeEncBias;
		KneeIMUData = MPU925x_ReadIMU();
	}

	CM_LoadCell->bot[0] = ReadLoadCell(ADC2);
	CM_LoadCell->top[0] = ReadLoadCell(ADC1);
}

// Should be moved to ADC driver??
uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while ( !LL_ADC_IsActiveFlag_EOC(ADCx) );
	LL_ADC_ClearFlag_EOC(ADCx);								// remove this??
	uint16_t data = LL_ADC_REG_ReadConversionData12(ADCx);	// Change resolution??
	return data;
}

void ProcessInputs(void)
{
	double tau = 1.0 / (2 * M_PI * 10);		// Time constant for practical differentiator (fc = 10 Hz)

	// Derivative of angle and filtering of load cells
	// No derivative of angle (angular speed) on first execution
	// No filtering of load cells on first or second execution
	if(isFirst)
	{
		if((config.Device == Ankle) || (config.Device == Combined))
		{
			CM_Ankle.jointSpeed = 0.0;
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}
		else if((config.Device == Knee) || (config.Device == Combined))
		{
			CM_Knee.jointSpeed = 0.0;
			CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];
		}

		CM_LoadCell->bot[2] = CM_LoadCell->bot[0];
		CM_LoadCell->top[2] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[0] = CM_LoadCell->bot[0];
		CM_LoadCell_Filtered->top[0] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[2] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[2] = CM_LoadCell_Filtered->top[0];
	}
	else if(isSecond)
	{
		if((config.Device == Ankle) || (config.Device == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}
		else if((config.Device == Knee) || (config.Device == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Knee.jointSpeed = (2*(CM_Knee.jointAngle[0] - CM_Knee.jointAngle[1]) + (2*tau - dt)*CM_Knee.jointSpeed) / (dt + 2*tau);
			CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];
		}

		CM_LoadCell->bot[1] = CM_LoadCell->bot[0];
		CM_LoadCell->top[1] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[0] = CM_LoadCell->bot[0];
		CM_LoadCell_Filtered->top[0] = CM_LoadCell->top[0];
		CM_LoadCell_Filtered->bot[1] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[1] = CM_LoadCell_Filtered->top[0];
	}
	else
	{
		if((config.Device == Ankle) || (config.Device == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}
		else if((config.Device == Knee) || (config.Device == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Knee.jointSpeed = (2*(CM_Knee.jointAngle[0] - CM_Knee.jointAngle[1]) + (2*tau - dt)*CM_Knee.jointSpeed) / (dt + 2*tau);
			CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];
		}

		// 2nd order low-pass Butterworth (fc = 20 Hz)
		CM_LoadCell_Filtered->bot[0] =   1.6556 * CM_LoadCell_Filtered->bot[1] - 0.7068 * CM_LoadCell_Filtered->bot[2]
									   + 0.0128 * CM_LoadCell->bot[0] + 0.0256 * CM_LoadCell->bot[1] + 0.0128 * CM_LoadCell->bot[2];
		CM_LoadCell_Filtered->top[0] =   1.6556 * CM_LoadCell_Filtered->top[1] - 0.7068 * CM_LoadCell_Filtered->top[2]
									   + 0.0128 * CM_LoadCell->top[0] + 0.0256 * CM_LoadCell->top[1] + 0.0128 * CM_LoadCell->top[2];

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
	double ankleAxBias = 0.0;
	double ankleAyBias = 0.0;
	double ankleAzBias = 0.0;
	double ankleGxBias = 0.0;
	double ankleGyBias = 0.0;
	double ankleGzBias = 0.0;
	double ankleN = 1.0;		// For normalization

	double kneeAxBias = 0.0;
	double kneeAyBias = 0.0;
	double kneeAzBias = 0.0;
	double kneeGxBias = 0.0;
	double kneeGyBias = 0.0;
	double kneeGzBias = 0.0;
	double kneeN = 1.0;			// For normalization

	// Sine and cosine of Euler angles (1 = z angle, 2 = x' angle, 3 = z'' angle)
	double ac1, ac2, ac3, as1, as2, as3;
	double kc1, kc2, kc3, ks1, ks2, ks3;
	if(config.Side == Left)
	{
		ac1 = cos(M_PI);
		ac2 = cos(M_PI);
		ac3 = cos(0.0);
		as1 = sin(M_PI);
		as2 = sin(M_PI);
		as3 = sin(0.0);

		kc1 = cos(M_PI);
		kc2 = cos(M_PI);
		kc3 = cos(0.0);
		ks1 = sin(M_PI);
		ks2 = sin(M_PI);
		ks3 = sin(0.0);
	}
	else
	{
		ac1 = cos(0.0);
		ac2 = cos(0.0);
		ac3 = cos(0.0);
		as1 = sin(0.0);
		as2 = sin(0.0);
		as3 = sin(0.0);

		kc1 = cos(0.0);
		kc2 = cos(0.0);
		kc3 = cos(0.0);
		ks1 = sin(0.0);
		ks2 = sin(0.0);
		ks3 = sin(0.0);
	}

	CM_Ankle.IMUData.ax = ankleN * (AnkleIMUData.ax*(ac1*ac3 - ac2*as1*as3) + AnkleIMUData.ay*(-ac3*as1     - ac1*ac2*as3 ) + AnkleIMUData.az*( as2*as3)) - ankleAxBias;
	CM_Ankle.IMUData.ay = ankleN * (AnkleIMUData.ax*(ac1*as3 + ac2*ac3*as1) + AnkleIMUData.ay*( ac1*ac2*ac3 - as1*as3     ) + AnkleIMUData.az*(-ac3*as2)) - ankleAyBias;
	CM_Ankle.IMUData.az = ankleN * (AnkleIMUData.ax*(as1*as2              ) + AnkleIMUData.ay*( ac1*as2                   ) + AnkleIMUData.az*( ac2    )) - ankleAzBias;
	CM_Ankle.IMUData.gx = ankleN * (AnkleIMUData.gx*(ac1*ac3 - ac2*as1*as3) + AnkleIMUData.gy*(-ac3*as1      - ac1*ac2*as3) + AnkleIMUData.gz*( as2*as3)) - ankleGxBias;
	CM_Ankle.IMUData.gy = ankleN * (AnkleIMUData.gx*(ac1*as3 + ac2*ac3*as1) + AnkleIMUData.gy*( ac1*ac2*ac3  - as1*as3    ) + AnkleIMUData.gz*(-ac3*as2)) - ankleGyBias;
	CM_Ankle.IMUData.gz = ankleN * (AnkleIMUData.gx*(as1*as2              ) + AnkleIMUData.gy*( ac1*as2                   ) + AnkleIMUData.gz*( ac2    )) - ankleGzBias;

	CM_Knee.IMUData.ax = kneeN * (KneeIMUData.ax*(kc1*kc3 - kc2*ks1*ks3) + KneeIMUData.ay*(-kc3*ks1 - kc1*kc2*ks3) + KneeIMUData.az*( ks2*ks3)) - kneeAxBias;
	CM_Knee.IMUData.ay = kneeN * (KneeIMUData.ax*(kc1*ks3 + kc2*kc3*ks1) + KneeIMUData.ay*( kc1*kc2*kc3 - ks1*ks3) + KneeIMUData.az*(-kc3*ks2)) - kneeAyBias;
	CM_Knee.IMUData.az = kneeN * (KneeIMUData.ax*(ks1*ks2              ) + KneeIMUData.ay*( kc1*ks2              ) + KneeIMUData.az*( kc2    )) - kneeAzBias;
	CM_Knee.IMUData.gx = kneeN * (KneeIMUData.gx*(kc1*kc3 - kc2*ks1*ks3) + KneeIMUData.gy*(-kc3*ks1 - kc1*kc2*ks3) + KneeIMUData.gz*( ks2*ks3)) - kneeGxBias;
	CM_Knee.IMUData.gy = kneeN * (KneeIMUData.gx*(kc1*ks3 + kc2*kc3*ks1) + KneeIMUData.gy*( kc1*kc2*kc3 - ks1*ks3) + KneeIMUData.gz*(-kc3*ks2)) - kneeGyBias;
	CM_Knee.IMUData.gz = kneeN * (KneeIMUData.gx*(ks1*ks2              ) + KneeIMUData.gy*( kc1*ks2              ) + KneeIMUData.gz*( kc2    )) - kneeGzBias;
}

void ComputeLimbAngle(void)
{
	double accelAngle = (atan(CM_Knee.IMUData.ax / sqrt(pow(CM_Knee.IMUData.ay, 2) + pow(CM_Knee.IMUData.az, 2)))) * 180.0/M_PI;
	static double compFiltAngle = 0.0;
	static double dGyroAngle = 0.0;

	// Change in angle from gyro (trapezoidal used)
	dGyroAngle = dt/2 * (CM_Knee.IMUData.gz + dGyroAngle);

	// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
	double alpha = 0.002;
	compFiltAngle = accelAngle*alpha + (1 - alpha) * (dGyroAngle + compFiltAngle);

//	CM_Ankle.limbAngle = compFiltAngle + CM_Ankle.jointAngle[0]; ??
	CM_Knee.limbAngle = compFiltAngle - CM_Knee.jointAngle[0];
}

void RunStateMachine(void)
{
	static enum StateMachine_e state;
	static uint32_t count = 0;

	if(isFirst)
	{
		if(isSimulatedWalkingRequired)
			state = Waiting;
		else
			state = Stance;
	}

	switch(state)
	{
	case Waiting:
		CM_state = 1700;
		if(CM_start)
		{
			state = Stance;
		}
		break;
	case Stance:
		CM_state = 1800;
		ProsCtrl.eqPoint = CM_Knee.StanceCtrl.eqPoint;
		ProsCtrl.kd = CM_Knee.StanceCtrl.kd;
		ProsCtrl.kp = CM_Knee.StanceCtrl.kp;

		if(isSimulatedWalkingRequired)
		{
			if(count > 1000)
			{
				state = SwingFlexion;
				count = 0;
			}
			else count++;
		}
		else if((CM_LoadCell_Filtered->top[0] < CM_lcTop_staticUpperLimit) && (CM_LoadCell_Filtered->bot[0] < CM_lcBot_staticUpperLimit))
			state = SwingFlexion;

		break;

	case SwingFlexion:
		CM_state = 2400;
		ProsCtrl.eqPoint = CM_Knee.SwingFlexCtrl.eqPoint;
		ProsCtrl.kd = CM_Knee.SwingFlexCtrl.kd;
		ProsCtrl.kp = CM_Knee.SwingFlexCtrl.kp;

		if(isSimulatedWalkingRequired)
		{
			if(count > 1000)
			{
				state = SwingExtension;
				count = 0;
			}
			else count++;
		}
		else if(CM_Knee.jointSpeed > 0)
			state = SwingExtension;

		break;

	case SwingExtension:
		CM_state = 2900;
		ProsCtrl.eqPoint = CM_Knee.SwingExtCtrl.eqPoint;
		ProsCtrl.kd = CM_Knee.SwingExtCtrl.kd;
		ProsCtrl.kp = CM_Knee.SwingExtCtrl.kp;

		if(isSimulatedWalkingRequired)
		{
			if(count > 1000)
			{
				state = Stance;
				count = 0;
			}
			else count++;
		}
		else if(CM_LoadCell_Filtered->bot[0] > CM_lcBot_staticUpperLimit)
			state = Stance;

		break;
	}
}

void RunImpedanceControl(void)
{
	float gearRatio = 40.0f;
	float nomCurrent = 8.0f;						// is this number accurate??
	float torqueConst = 60.0f / (2*M_PI * 100);		// Units in N*m/A, for Kv = 100 rpm/V

	float errorPos = ProsCtrl.eqPoint - CM_Knee.jointAngle[0];

	//	CM_Ankle.jointTorque_nm = ProsCtrl.kp*errorPos - ProsCtrl.kd*CM_Ankle.jointSpeed * CM_gain; ??
	CM_Knee.jointTorque = -(ProsCtrl.kp*errorPos - ProsCtrl.kd*CM_Knee.jointSpeed) * CM_gain;

	int32_t motorTorque = CM_Knee.jointTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
	EPOS4_SetTorque(kneeCANID, motorTorque);
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
		EPOS4_SetTorque(kneeCANID, torque);
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

		CM_kneeEncBias = sum / i;

		break;
	}
	case ImpedanceControl:
	{
		// First compute average of current position and use as equilibrium point
		// Then run impedance control
		if (isFirst)
		{
			CM_gain = 1.0f;
			uint16_t i;
			float sum = 0.0f;

			for(i = 0; i < 1000; i++)
			{
				float pos = AS5145B_ReadPosition_Deg();
				sum += pos;
			}

			CM_Knee.ImpCtrl.eqPoint = sum / i - kneeEncBias;
		}
		else
		{
			ProsCtrl.kd = CM_Knee.ImpCtrl.kd;
			ProsCtrl.kp = CM_Knee.ImpCtrl.kp;
			ProsCtrl.eqPoint = CM_Knee.ImpCtrl.eqPoint;

			RunImpedanceControl();
		}

		break;
	}
	case SimulatedWalking:
		RunStateMachine();
		RunImpedanceControl();
		break;
	}
}


/*******************************************************************************
* END
*******************************************************************************/
