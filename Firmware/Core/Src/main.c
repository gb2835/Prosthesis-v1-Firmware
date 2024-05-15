/* USER CODE BEGIN Header */

/*******************************************************************************
 *
 * TITLE   Prosthesis Knee Control
 * AUTHOR  Greg Berkeley
 * RELEASE XX/XX/XXXX
 *
 * NOTES
 * 1. The below lines can be used to measure PB11 on oscilloscope:
 *     - LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
 *     - LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);
 *     - LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_11);
 * 2. Test programs are provided prior to the main while loop.
 * 3. Double question marks (??) are commented at locations throughout the code
 *    where possible improvements may be made.
 *
 * ABSTRACT
 * The below code XXX.
 *
 ******************************************************************************/


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lptim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/*******************************************************************************
 * USER INCLUDES
 ******************************************************************************/

#include "as5145b.h"
#include "EPOS4.h"
#include "mcp25625.h"


/******************************************************************************/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


/*******************************************************************************
 * USER DEFINITIONS
 ******************************************************************************/

	// Define statements
//	#define USE_FULL_ASSERT		// Provides assert_failure() functionality

	// Define statements with values
	#define LPTIM2_PERIOD	0x3F	// Period for LPTIM2

	// Declare variables
	AS5145B_Init_t enc;		// Magnetic encoder initialization handle

	// Declare and define variables
	uint16_t CAN_ID      = 0x601;				// CAN ID for EPOS4
	float    encBias_deg = 1325.0f * 360/4096;	// Magnetic encoder zero position bias converted to degrees (ADC value found using test program below)

	// Assign properties to magnetic encoder initialization handle
	enc.CSn_GPIOx = Enc_CSn_GPIO_Port;		// Chip select GPIO port
	enc.CSn_Pin   = Enc_CSn_Pin;			// Chip select pin number
	enc.CLK_GPIOx = Enc_CLK_GPIO_Port;		// Clock GPIO port
	enc.CLK_Pin   = Enc_CLK_Pin;			// Clock pin number
	enc.DO_GPIOx  = Enc_DO_GPIO_Port;		// Data output GPIO port
	enc.DO_Pin    = Enc_DO_Pin;				// Data output pin number


/******************************************************************************/

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_LPTIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */


/******************************************************************************
* USER INITIALIZATIONS
******************************************************************************/

	// Start LPTIM2 interrupt
	LL_LPTIM_Enable(LPTIM2);												// Initially enable timer
	LL_LPTIM_EnableIT_ARRM(LPTIM2);											// Enable timer interrupt mode
	LL_LPTIM_SetAutoReload( LPTIM2, LPTIM2_PERIOD );						// Set auto reload register with period
	LL_LPTIM_StartCounter( LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS );	// Start counter in continuous operating mode

	// Enable peripherals
	LL_SPI_Enable(SPI2);

	// Initialize devices
	CAN_configure();			// Configure MCP25625 to normal mode
	EPOS4_SetCSTMode(CAN_ID);	// Set EPOS4 to cyclic synchronous torque mode
	AS5145B_Init(&enc);			// Initialize magnetic encoder

	// Remove spikes from beginning (can we do better??)
	for ( uint16_t i = 0; i < 1000; i++ );


/******************************************************************************
* TEST PROGRAMS
******************************************************************************/

	// Comment/Uncomment to access the test programs below
	// Each test program halts the program (only one at a time is meant to be used)
//	#define TESTPROGRAM_COMMANDMOTOR	// Provides test program to command motor at a constant torque
//	#define TESTPROGRAM_AVERAGEMAGENC	// Provides test program to average the magnetic encoder
//	#define TESTPROGRAM_PDCONTROL		// Provides test program to PD control motor
	#define TESTPROGRAM_READIMU			// Provides test program to read IMU

	// 1. Command motor at a constant torque
	#ifdef TESTPROGRAM_COMMANDMOTOR
	int32_t torque = 150;				// Per thousand of motor rated torque
	EPOS4_SetTorque( CAN_ID, torque );	// Set torque in EPOS4 per thousand of motor rated torque
	while (1);							// Halt program
	#endif

	// 2. Compute average of magnetic encoder angular position in ADC (can be used to find zero angular position bias)
	#ifdef TESTPROGRAM_AVERAGEMAGENC
	uint16_t i;			// For loop iteration variable
	uint32_t sum = 0;	// Contains sum for average
	for ( i = 0; i < 1000; i++ )
	{
		struct AS5145B_Data_s data  = AS5145B_ReadData();	// Read data from magnetic encoder
		sum                        += data.pos;				// Separate angular position in ADC and sum together
	}
	uint16_t average = sum / i;		// Compute average in ADC
	(void) average;					// Void unused variable warning
	while (1);						// Halt program
	#endif

	// 3. PD control of motor (removes zero angular position bias first from magnetic encoder readings)
	#ifdef TESTPROGRAM_PDCONTROL
	float    pos_deg[2];							// Angular position in degrees ( [0] = k-0 or current position, [1] = k-1 or previous position )
	float    outTorque_nm;							// Output torque in N*m
	uint16_t i;										// For loop iteration variable
	float    desPos_deg 	  = 0.0f;				// Desired angular position in degrees
	float    Kp         	  = 2.5f;				// Proportional control gain
	float    Kd         	  = 0.0f;				// Derivative control gain
	float	 tau        	  = 1/(2*3.1416f*10);	// Time constant for practical differentiator (fc = 10 Hz)
	float	 Ts         	  = 1/512.0f;			// Sample time in seconds
	float    torqueConst_nmpa = 0.095f;				// Torque constant of motor in N*m/A (is this number accurate??)
	float    speed_dps        = 0.0f;				// Angular speed in degrees per second initialized to zero
	uint8_t  isFirst          = 1;					// Is this first execution? No derivative control on first execution
	uint8_t  gearRatio        = 40;					// Gear ratio
	uint8_t  nomCurrent_amp	  = 8;					// Nominal current of motor in amps (is this number accurate??)
	uint32_t sum              = 0;					// Contains sum for average bias
	for ( i = 0; i < 1000; i++ )
	{
		struct AS5145B_Data_s data  = AS5145B_ReadData();	// Read data from magnetic encoder
		sum                        += data.pos;				// Separate angular position from data and sum together
	}
	float bias_deg = (float) sum/i * 360/4096;	// Compute average bias in degrees
	while (1)
	{
		// Read angular position in degrees
		pos_deg[0] = AS5145B_ReadPosition_Deg() - bias_deg;

		// Compute angular position error in degrees
		float errorPos_deg = desPos_deg - pos_deg[0];

		// Check if this is first execution (no derivative control on first execution)
		if (isFirst)
		{
			// Compute required output torque in N*m
			outTorque_nm = Kp*errorPos_deg;
			isFirst = 0;
		}
		else
		{
			// Compute angular speed in degrees per second with practical differentiator (fc = 10 Hz)
			speed_dps  = ( 2*( pos_deg[0] - pos_deg[1] ) + ( 2*tau - Ts )*speed_dps ) / ( Ts + 2*tau );

			// Compute required output torque in N*m
			outTorque_nm = Kp*errorPos_deg - Kd*speed_dps;
		}

		// Compute required motor torque per thousand of motor rated torque
		int32_t motTorque = outTorque_nm / ( torqueConst_nmpa * gearRatio * nomCurrent_amp ) * 1000.0f;

		// Command torque to motor per thousand of motor rated torque
		EPOS4_SetTorque( CAN_ID, motTorque );
		LL_mDelay(1);

		// Shift current angular position to previous angular position
		pos_deg[1] = pos_deg[0];
	}
	#endif

	// 4. Read IMU
	#ifdef TESTPROGRAM_READIMU

	#endif




/******************************************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


/******************************************************************************
* USER MAIN LOOP
******************************************************************************/

  while (1)
  {
	  (void) encBias;

/******************************************************************************/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_11);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnablePLLMode();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_6, 40, LL_RCC_PLLR_DIV_4);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

  LL_Init1msTick(80000000);

  LL_SetSystemCoreClock(80000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
