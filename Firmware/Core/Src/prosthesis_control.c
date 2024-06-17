/*******************************************************************************
 *
 * TITLE   Prosthesis Control
 * AUTHOR  Greg Berkeley
 * RELEASE XX/XX/XXXX
 *
 * NOTES
 * 1. None.
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

#define KNEE

#define MPUREG_I2C_SLV0_ADDR	0x25
#define AK8963_I2C_ADDR			0x0c
#define READ_FLAG				0x80
#define MPUREG_I2C_SLV0_REG		0x26
#define AK8963_HXL				0x03
#define MPUREG_I2C_SLV0_CTRL	0x27
#define MPUREG_ACCEL_XOUT_H		0x3B

enum StateMachine_e
{
	Stance,
	Swing
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

// Should be moved to IMU driver??
struct IMU_Data_s
{
   double	ax_g;
   double	ay_g;
   double	az_g;
   double	gx_dps;
   double	gy_dps;
   double	gz_dps;
};

static enum TestPrograms_e testProgram;
struct ControlParams_s ProsCtrl;
struct IMU_Data_s IMU_Data;					// Uncalibrated data

double compFiltAngle_deg = 0.0;
double dGyroAngle_deg = 0.0;
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
struct ControlParams_s CM_ImpCtrl, CM_StanceCtrl, CM_SwingCtrl;
struct IMU_Data_s CM_IMU_Data;										// Calibrated data
struct LoadCell_Data_s CM_LoadCell[3], CM_LoadCell_Filtered[3];		// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_magEncBias_raw;

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

void InitProsthesisControl(void)
{
	CM_ImpCtrl.kd = 0.0f;
	CM_ImpCtrl.kp = 2.5f;
	CM_StanceCtrl.eqPoint_deg = -4.99f;		// Vanderbilt = -4.99 deg
	CM_StanceCtrl.kd = 0.0;					// Vanderbilt = 0 N*m/(deg/s)
	CM_StanceCtrl.kp = 2.5f;				// 2.50 used to keep heat down in EPOS, Vanderbilt = 4.97 N*m/deg
	CM_SwingCtrl.eqPoint_deg = -35.0f;		// Vanderbilt = -35.0 deg
	CM_SwingCtrl.kd = 0.05f;				// 0.05 used to get zero overshoot and 0.5 sec settling time, Vanderbilt = 0 N*m/(deg/s)
	CM_SwingCtrl.kp = 0.45f;				// 0.45 on the bench "feels" right, Vanderbilt = 0.65 N*m/deg
}

void RunProsthesisControl(void)
{
	GetInputs();
	ProcessInputs();

	// If test program is required, run test program
	// Otherwise continue prosthesis control
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

void RequireTestProgram(enum TestPrograms_e option)
{
	testProgram = option;

	if(testProgram != None)
		isTestProgramRequired = 1;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void GetInputs(void)
{
	CM_jointAngle_deg[0] = AS5145B_ReadPosition_Deg() - encBias_deg;
	CM_LoadCell->bot[0] = ReadLoadCell(ADC2);
	CM_LoadCell->top[0] = ReadLoadCell(ADC1);
	IMU_Data = IMU_read();
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
	double axBias_g = 0.0;
	double ayBias_g = 0.0;
	double azBias_g = 0.0;
	double gxBias_dps = 0.0;
	double gyBias_dps = 0.0;
	double gzBias_dps = 0.0;
	double n = 1.0;				// Scaling factor (helps with normalization)

	// Sine and cosine of Euler angles (1 = z angle, 2 = x' angle, 3 = z'' angle)
	double c1 = cos(0);
	double c2 = cos(0);
	double c3 = cos(0);
	double s1 = sin(0);
	double s2 = sin(0);
	double s3 = sin(0);

	// Rotate IMU data and remove biases
	CM_IMU_Data.ax_g = n * ( IMU_Data.ax_g*(c1*c3 - c2*s1*s3) + IMU_Data.ay_g*(  -c3*s1 - c1*c2*s3) + IMU_Data.az_g*( s2*s3) ) - axBias_g;
	CM_IMU_Data.ay_g = n * ( IMU_Data.ax_g*(c1*s3 + c2*c3*s1) + IMU_Data.ay_g*(c1*c2*c3 - s1*s3   ) + IMU_Data.az_g*(-c3*s2) ) - ayBias_g;
	CM_IMU_Data.az_g = n * ( IMU_Data.ax_g*(        s1*s2   ) + IMU_Data.ay_g*(           c1*s2   ) + IMU_Data.az_g*( c2   ) ) - azBias_g;
	CM_IMU_Data.gx_dps = n * ( IMU_Data.gx_dps*(c1*c3 - c2*s1*s3) + IMU_Data.gy_dps*(  -c3*s1 - c1*c2*s3) + IMU_Data.gz_dps*( s2*s3) ) - gxBias_dps;
	CM_IMU_Data.gy_dps = n * ( IMU_Data.gx_dps*(c1*s3 + c2*c3*s1) + IMU_Data.gy_dps*(c1*c2*c3 - s1*s3   ) + IMU_Data.gz_dps*(-c3*s2) ) - gyBias_dps;
	CM_IMU_Data.gz_dps = n * ( IMU_Data.gx_dps*(        s1*s2   ) + IMU_Data.gy_dps*(           c1*s2   ) + IMU_Data.gz_dps*( c2   ) ) - gzBias_dps;
}

static void ComputeLimbAngle(void)
{
	double accelAngle_deg = ( atan( CM_IMU_Data.ax_g / sqrt( pow( CM_IMU_Data.ay_g, 2 ) + pow(CM_IMU_Data.az_g, 2 ) ) ) ) * 180/3.1416;

	// Change in angle from gyro (trapezoidal used)
	dGyroAngle_deg = dt/2 * (CM_IMU_Data.gz_dps + dGyroAngle_deg);

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
	enum StateMachine_e state;

	if(isFirst)
	{
		state = Stance;
	}

	switch(state)
	{
	case Stance:
		ProsCtrl.eqPoint_deg = CM_StanceCtrl.eqPoint_deg;
		ProsCtrl.kd = CM_StanceCtrl.kd;
		ProsCtrl.kp = CM_StanceCtrl.kp;
		break;
	case Swing:
		ProsCtrl.eqPoint_deg = CM_SwingCtrl.eqPoint_deg;
		ProsCtrl.kd = CM_SwingCtrl.kd;
		ProsCtrl.kp = CM_SwingCtrl.kp;
		break;
	}
}

static void RunImpedanceControl(void)
{
	float gearRatio = 40.0f;
	float nomCurrent_amp = 8.0f;	// is this number accurate??
	float torqueConst = 0.095f;		// Units in N*m/A, is this number accurate??

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

struct IMU_Data_s IMU_read(void)
{
	struct IMU_Data_s IMU;
	uint8_t response[21];
	WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);
	WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);
	WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);

	ReadRegs(MPUREG_ACCEL_XOUT_H, response, 21);

	int16_t accel[3];
	int16_t gyro[3];

	#ifdef KNEE
	int8_t orientation[3] = {1, 2, 3};
	#else
	int8_t orientation[3] = {-1, 2, -3};
	#endif

	if(orientation[0] > 0)
	{
		accel[orientation[0]-1] = ((int16_t) response[0] << 8) | response[1];
		gyro[orientation[0]-1] = ((int16_t) response[8] << 8) | response[9];
	}
	else
	{
		int16_t tmp = ((int16_t) response[0] << 8) | response[1];
		accel[-orientation[0]-1] = -tmp;

		tmp = ((int16_t) response[8] << 8) | response[9];
		gyro[-orientation[0]-1] = -tmp;
	}

	if(orientation[1] > 0)
	{
		accel[orientation[1]-1] = ((int16_t) response[2] << 8) | response[3];
		gyro[orientation[1]-1] = ((int16_t) response[10] << 8) | response[11];
	}
	else
	{
		int16_t tmp = ((int16_t) response[2] << 8) | response[3];
		accel[-orientation[1]-1] = -tmp;

		tmp = ((int16_t) response[10] << 8) | response[11];
		gyro[-orientation[1]-1] = -tmp;
	}

	if(orientation[2] > 0)
	{
		accel[orientation[2]-1] = ((int16_t) response[4] << 8) | response[5];
		gyro[orientation[2]-1] = ((int16_t) response[12] << 8) | response[13];
	}
	else
	{
		int16_t tmp = ((int16_t) response[4] << 8) | response[5];
		accel[-orientation[2]-1] = -tmp;

		tmp = ((int16_t) response[12] << 8) | response[13];
		gyro[-orientation[2]-1] = -tmp;
	}

	IMU.ax_g = accel[0] / 4096.0;
	IMU.ay_g = accel[1] / 4096.0;
	IMU.az_g = accel[2] / 4096.0;
	IMU.gx_dps = gyro[0] / 32.8;
	IMU.gy_dps = gyro[1] / 32.8;
	IMU.gz_dps = gyro[2] / 32.8;

	return IMU;
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
