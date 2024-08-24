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

// Can we do better
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
	float eqPoint_deg;
	float kd;			// Units in N*m/(deg/s)
	float kp;			// Units in N*m/deg
};

struct LoadCell_Data_s
{
	float bot[3];
	float top[3];
};

static enum TestPrograms_e testProgram;
struct ControlParams_s ProsCtrl;
struct MPU925x_IMUData_s IMUData;

float dt = 1 / 512.0f;
uint8_t isFirst = 1;
uint8_t isSecond = 0;
uint8_t isTestProgramRequired = 0;

#ifdef KNEE
float encBias_deg = 2244 * AS5145B_RAW2DEG;
#else
float encBias_deg = 1325 * AS5145B_RAW2DEG;
#endif

// For CubeMonitor
double CM_limbAngle_deg;
float CM_jointAngle_deg[2];											// [0] = k-0, [1] = k-1
float CM_jointSpeed_dps = 0.0f;
float CM_jointTorque_nm;
float CM_lcBot_staticUpperLimit, CM_lcTop_staticUpperLimit;
struct ControlParams_s CM_ImpCtrl, CM_StanceCtrl, CM_SwingFlexCtrl, CM_SwingExtCtrl;
struct MPU925x_IMUData_s CM_IMUData;
struct LoadCell_Data_s CM_LoadCell[3], CM_LoadCell_Filtered[3];		// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_magEncBias_raw;
uint16_t CM_state;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void CalibrateIMU(void);
static void ComputeLimbAngle(void);
static void RunStateMachine(void);
static void RunImpedanceControl(void);
static void RunTestProgram(void);
struct IMU_Data_s IMU_read(void);
unsigned int WriteReg(uint8_t adress, uint8_t data);
void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

// This is to localize variables that are subject to change during testing
void InitProsthesisControl(void)
{
	CM_ImpCtrl.kd = 0.0f;
	CM_ImpCtrl.kp = 0.0f;
	CM_StanceCtrl.eqPoint_deg = 0.0f;		// Vanderbilt = -4.99 deg
	CM_StanceCtrl.kd = 0.0f;				// Vanderbilt = 0 N*m/(deg/s)
	CM_StanceCtrl.kp = 0.0f;				// 2.50 used to keep heat down in EPOS, Vanderbilt = 4.97 N*m/deg
	CM_SwingFlexCtrl.eqPoint_deg = 0.0f;	// Vanderbilt = -35.0 deg
	CM_SwingFlexCtrl.kd = 0.0f;				// 0.05 used to get zero overshoot and 0.5 sec settling time, Vanderbilt = 0 N*m/(deg/s)
	CM_SwingFlexCtrl.kp = 0.0f;				// 0.45 on the bench "feels" right, Vanderbilt = 0.65 N*m/deg
	CM_SwingExtCtrl.eqPoint_deg = 0.f;
	CM_SwingExtCtrl.kd = 0.0f;
	CM_SwingExtCtrl.kp = 0.0f;

	CM_lcBot_staticUpperLimit = 2200;
	CM_lcTop_staticUpperLimit = 2370;
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

static void GetInputs(void)
{
	CM_jointAngle_deg[0] = AS5145B_ReadPosition_Deg() - encBias_deg;
	CM_LoadCell->bot[0] = ReadLoadCell(ADC2);
	CM_LoadCell->top[0] = ReadLoadCell(ADC1);
	IMUData = MPU925x_ReadIMU();
	IMU_Orientation();
}

// Should be moved to ADC driver??
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while ( !LL_ADC_IsActiveFlag_EOC(ADCx) );
	LL_ADC_ClearFlag_EOC(ADCx);								// remove this??
	uint16_t val = LL_ADC_REG_ReadConversionData12(ADCx);	// Change resolution??
	return val;
}

static void ProcessInputs(void)
{
	float tau = 1 / (2 * 3.1416f * 10);		// Time constant for practical differentiator (fc = 10 Hz)

	// Derivative of angle and filtering of load cells
	// No derivative of angle (angular speed) on first execution
	// No filtering of load cells on first or second execution
	if(isFirst)
	{
		CM_jointSpeed_dps = 0.0f;

		CM_jointAngle_deg[1] = CM_jointAngle_deg[0];
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
		CM_jointSpeed_dps = ( 2*( CM_jointAngle_deg[0] - CM_jointAngle_deg[1] ) + ( 2*tau - dt )*CM_jointSpeed_dps ) / ( dt + 2*tau );

		CM_jointAngle_deg[1] = CM_jointAngle_deg[0];
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
		CM_jointSpeed_dps = ( 2*( CM_jointAngle_deg[0] - CM_jointAngle_deg[1] ) + ( 2*tau - dt )*CM_jointSpeed_dps ) / ( dt + 2*tau );

		// 2nd order low-pass Butterworth (fc = 20 Hz)
		CM_LoadCell_Filtered->bot[0] =   1.6556f * CM_LoadCell_Filtered->bot[1] - 0.7068f * CM_LoadCell_Filtered->bot[2]
									   + 0.0128f * CM_LoadCell->bot[0] + 0.0256f * CM_LoadCell->bot[1] + 0.0128f * CM_LoadCell->bot[2];
		CM_LoadCell_Filtered->top[0] =   1.6556f * CM_LoadCell_Filtered->top[1] - 0.7068f * CM_LoadCell_Filtered->top[2]
									   + 0.0128f * CM_LoadCell->top[0] + 0.0256f * CM_LoadCell->top[1] + 0.0128f * CM_LoadCell->top[2];

		CM_jointAngle_deg[1] = CM_jointAngle_deg[0];
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

static void CalibrateIMU(void)
{
	double axBias = 0.0;
	double ayBias = 0.0;
	double azBias = 0.0;
	double gxBias = 0.0;
	double gyBias = 0.0;
	double gzBias = 0.0;
	double n = 1.0;				// Scaling factor (helps with normalization)

	// Sine and cosine of Euler angles (1 = z angle, 2 = x' angle, 3 = z'' angle)
	double c1 = cos(0);
	double c2 = cos(0);
	double c3 = cos(0);
	double s1 = sin(0);
	double s2 = sin(0);
	double s3 = sin(0);

	// Rotate IMU data and remove biases
	CM_IMUData.ax = n * (IMUData.ax*(c1*c3 - c2*s1*s3) + IMUData.ay*(  -c3*s1 - c1*c2*s3) + IMUData.az*( s2*s3)) - axBias;
	CM_IMUData.ay = n * (IMUData.ax*(c1*s3 + c2*c3*s1) + IMUData.ay*(c1*c2*c3 - s1*s3   ) + IMUData.az*(-c3*s2)) - ayBias;
	CM_IMUData.az = n * (IMUData.ax*(        s1*s2   ) + IMUData.ay*(           c1*s2   ) + IMUData.az*( c2   )) - azBias;
	CM_IMUData.gx = n * (IMUData.gx*(c1*c3 - c2*s1*s3) + IMUData.gy*(  -c3*s1 - c1*c2*s3) + IMUData.gz*( s2*s3)) - gxBias;
	CM_IMUData.gy = n * (IMUData.gx*(c1*s3 + c2*c3*s1) + IMUData.gy*(c1*c2*c3 - s1*s3   ) + IMUData.gz*(-c3*s2)) - gyBias;
	CM_IMUData.gz = n * (IMUData.gx*(        s1*s2   ) + IMUData.gy*(           c1*s2   ) + IMUData.gz*( c2   )) - gzBias;
}

static void ComputeLimbAngle(void)
{
	double accelAngle_deg = (atan( CM_IMUData.ax / sqrt(pow(CM_IMUData.ay, 2) + pow(CM_IMUData.az, 2)))) * 180/3.1416;
	static double compFiltAngle_deg = 0.0;
	static double dGyroAngle_deg = 0.0;

	// Change in angle from gyro (trapezoidal used)
	dGyroAngle_deg = dt/2 * (CM_IMUData.gz + dGyroAngle_deg);

	// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
	double alpha = 0.002;
	compFiltAngle_deg = accelAngle_deg*alpha + (1 - alpha) * (dGyroAngle_deg + compFiltAngle_deg);

	#ifdef KNEE
	CM_limbAngle_deg = compFiltAngle_deg - CM_jointAngle_deg[0];
	#else
	CM_limbAngle_deg = compFiltAngle_deg + CM_jointAngle_deg[0];
	#endif
}

static void RunStateMachine(void)
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
		ProsCtrl.eqPoint_deg = CM_StanceCtrl.eqPoint_deg;
		ProsCtrl.kd = CM_StanceCtrl.kd;
		ProsCtrl.kp = CM_StanceCtrl.kp;

		if((CM_LoadCell_Filtered->top[0] < CM_lcTop_staticUpperLimit) && (CM_LoadCell_Filtered->bot[0] < CM_lcBot_staticUpperLimit))
		{
			state = SwingFlexion;
		}

		break;

	case SwingFlexion:
		CM_state = 2400;
		ProsCtrl.eqPoint_deg = CM_SwingFlexCtrl.eqPoint_deg;
		ProsCtrl.kd = CM_SwingFlexCtrl.kd;
		ProsCtrl.kp = CM_SwingFlexCtrl.kp;

		if(CM_jointSpeed_dps > 0)
		{
			state = SwingExtension;
		}

		break;

	case SwingExtension:
		CM_state = 2900;
		ProsCtrl.eqPoint_deg = CM_SwingExtCtrl.eqPoint_deg;
		ProsCtrl.kd = CM_SwingExtCtrl.kd;
		ProsCtrl.kp = CM_SwingExtCtrl.kp;

		if(CM_LoadCell_Filtered->bot[0] > CM_lcBot_staticUpperLimit)
		{
			state = Stance;
		}

		break;
	}
}

static void RunImpedanceControl(void)
{
	float gearRatio = 40.0f;
	float nomCurrent_amp = 8.0f;					// is this number accurate??
	float torqueConst = 60 / (2*3.1416f * 100);		// Units in N*m/A, for Kv = 100 rpm/V

	float errorPos_deg = ProsCtrl.eqPoint_deg - CM_jointAngle_deg[0];

	#ifdef KNEE
	CM_jointTorque_nm = -(ProsCtrl.kp*errorPos_deg - ProsCtrl.kd*CM_jointSpeed_dps);
	#else
	CM_jointTorque_nm = ProsCtrl.kp*errorPos_deg - ProsCtrl.kd*CM_jointSpeed_dps;
	#endif

	int32_t motorTorque = CM_jointTorque_nm / (torqueConst * gearRatio * nomCurrent_amp) * 1000;
	EPOS4_SetTorque(CAN_ID, motorTorque);
}

static void RunTestProgram(void)
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

		uint32_t sum = 0.0f;

		for(i = 0; i < 1000; i++)
		{
			uint16_t bias_raw = AS5145B_ReadPosition_Raw();
			sum += bias_raw;
		}

		CM_magEncBias_raw = sum / i;

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
				float pos_deg = AS5145B_ReadPosition_Deg();
				sum += pos_deg;
			}

			CM_ImpCtrl.eqPoint_deg = sum / i - encBias_deg;
		}
		else
		{
			ProsCtrl.kd = CM_ImpCtrl.kd;
			ProsCtrl.kp = CM_ImpCtrl.kp;
			ProsCtrl.eqPoint_deg = CM_ImpCtrl.eqPoint_deg;

			RunImpedanceControl();
		}

		break;
	}
	}
}


/*******************************************************************************
* Should move to IMU driver??
*******************************************************************************/

void IMU_Orientation(void)
{
	double accel[3];
	double gyro[3];

	#ifdef RIGHT
	int8_t orientation[3] = {1, 2, 3};
	#else
	int8_t orientation[3] = {-1, 2, -3};
	#endif

	if(orientation[0] > 0)
	{
		accel[orientation[0]-1] = IMUData.ax;
		gyro[orientation[0]-1] = IMUData.gx;
	}
	else
	{
		double tmp = IMUData.ax;
		accel[-orientation[0]-1] = -tmp;

		tmp = IMUData.gx;
		gyro[-orientation[0]-1] = -tmp;
	}

	if(orientation[1] > 0)
	{
		accel[orientation[1]-1] = IMUData.ay;
		gyro[orientation[1]-1] = IMUData.gy;
	}
	else
	{
		double tmp = IMUData.ay;
		accel[-orientation[1]-1] = -tmp;

		tmp = IMUData.gy;
		gyro[-orientation[1]-1] = -tmp;
	}

	if(orientation[2] > 0)
	{
		accel[orientation[2]-1] = IMUData.az;
		gyro[orientation[2]-1] = IMUData.gz;
	}
	else
	{
		double tmp = IMUData.az;
		accel[-orientation[2]-1] = -tmp;

		tmp = IMUData.gz;
		gyro[-orientation[2]-1] = -tmp;
	}

	IMUData.ax = accel[0];
	IMUData.ay = accel[1];
	IMUData.az = accel[2];
	IMUData.gx = gyro[0];
	IMUData.gy = gyro[1];
	IMUData.gz = gyro[2];
}

unsigned int WriteReg(uint8_t adress, uint8_t data)
{
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);

	while (!(SPI1->SR & SPI_SR_TXE));
	LL_SPI_TransmitData8(SPI1, adress);
	while (!(SPI1->SR & SPI_SR_RXNE));
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE));
	LL_SPI_TransmitData8(SPI1, data);
	while (!(SPI1->SR & SPI_SR_RXNE));
	temp_val = LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);
	return temp_val;
}

void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes)
{
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);

	while (!(SPI1->SR & SPI_SR_TXE));
	LL_SPI_TransmitData8(SPI1, (ReadAddr | 0x80));
	while (!(SPI1->SR & SPI_SR_RXNE));
	LL_SPI_ReceiveData8(SPI1);

	for (i = 0; i < Bytes; i++)
	{
		while(!(SPI1->SR & SPI_SR_TXE));
		LL_SPI_TransmitData8(SPI1, 0x00);
		while(!(SPI1->SR & SPI_SR_RXNE));
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI1);
	}

	LL_GPIO_SetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);
}


/*******************************************************************************
* END
*******************************************************************************/
