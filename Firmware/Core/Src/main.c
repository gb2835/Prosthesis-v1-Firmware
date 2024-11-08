/* USER CODE BEGIN Header */

/*******************************************************************************
*
* TITLE   Prosthesis Firmware v1
* AUTHOR  Greg Berkeley
* RELEASE XX/XX/XXXX
*
* NOTES
* 1. See "Documents" directory for description of how v1 firmware is used.??
* 2. The below lines can be used to measure PB2 on oscilloscope:
*     - LL_GPIO_SetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*     - LL_GPIO_ResetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*     - LL_GPIO_TogglePin(OSCOPE_GPIO_Port, OSCOPE_Pin);
* 3. Test programs provided prior to main loop to independently test device
*    functionality.
* 4. Double question marks (??) are commented at locations throughout project
*    files where possible improvements may be made.
* 5. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*    A new magnetic encoder bias position must be found and defined whenever
*    the magnet is reassembled into the prosthesis device. A test program is
*    provided to find the bias.
*    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*
*******************************************************************************/


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "lptim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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


/*******************************************************************************
* USER PRELIMINARIES
*******************************************************************************/

#include "as5145b.h"
#include "EPOS4.h"
#include "prosthesis_control.h"
#include "mcp25625.h"
#include "mpu925x_spi.h"

#define LPTIM2_PERIOD 0x3F	// Timer frequency = timer clock frequency / (prescaler * (period + 1))


/******************************************************************************/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ADC2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


/*******************************************************************************
* USER DEFINITIONS
*******************************************************************************/

	AS5145B_Init_t MagEnc;
	MagEnc.DO_GPIOx = ENC_DO_GPIO_Port;
	MagEnc.CLK_GPIOx = ENC_CLK_GPIO_Port;
	MagEnc.CSn_GPIOx = ENC_CSn_GPIO_Port;
	MagEnc.DO_Pin = ENC_DO_Pin;
	MagEnc.CLK_Pin = ENC_CLK_Pin;
	MagEnc.CSn_Pin = ENC_CSn_Pin;

  	MCP25625_Inits_t CAN_Controller_Inits;
  	CAN_Controller_Inits.SPIx = SPI2;
  	CAN_Controller_Inits.CS_Port = SPI2_CS_GPIO_Port;
  	CAN_Controller_Inits.csPin = SPI2_CS_Pin;
  	CAN_Controller_Inits.CANCTRL_Reg.Bits.CLKPRE = clockoutDiv1;
  	CAN_Controller_Inits.CANCTRL_Reg.Bits.CLKEN = clockoutDisabled;
  	CAN_Controller_Inits.CANCTRL_Reg.Bits.OSM = oneShotModeEnabled;
  	CAN_Controller_Inits.CANCTRL_Reg.Bits.ABAT = abortAllTransmissions;
  	CAN_Controller_Inits.CANCTRL_Reg.Bits.REQOP = normalOperationMode;
  	CAN_Controller_Inits.CNF1_Reg.Bits.BRP = 0;
  	CAN_Controller_Inits.CNF1_Reg.Bits.SJW = length4xT_Q;
  	CAN_Controller_Inits.CNF2_Reg.Bits.PRSEG = 1;
  	CAN_Controller_Inits.CNF2_Reg.Bits.PHSEG1 = 1;
  	CAN_Controller_Inits.CNF2_Reg.Bits.SAM = busSampledOnceAtSamplePoint;
  	CAN_Controller_Inits.CNF2_Reg.Bits.BLTMODE = ps2LengthDeterminedByCNF3;
  	CAN_Controller_Inits.CNF3_Reg.Bits.PHSEG2 = 4;
  	CAN_Controller_Inits.CNF3_Reg.Bits.WAKFIL = wakeUpFilterIsEnabled;

	EPOS4_Inits_t Motor_Inits;
	Motor_Inits.nodeId = 2;
	Motor_Inits.Requirements.isFirstStepRequired = 1;
	Motor_Inits.Requirements.isModeOfOperationRequired = 1;
	Motor_Inits.FirstStep.CAN_BitRate = rate1000Kbps;
	Motor_Inits.FirstStep.MotorType = trapezoidalPmBlMotor;
	Motor_Inits.FirstStep.nominalCurrent = 6600;
	Motor_Inits.FirstStep.outputCurrentLimit = 29300;
	Motor_Inits.FirstStep.numberOfPolePairs = 21;
	Motor_Inits.FirstStep.thermalTimeConstantWinding = 400;
	Motor_Inits.FirstStep.torqueConstant = 95000;
	Motor_Inits.FirstStep.maxMotorSpeed = 2384;
	Motor_Inits.FirstStep.maxGearInputSpeed = 100000;
	Motor_Inits.FirstStep.sensorsConfiguration = 0x00100000;
	Motor_Inits.FirstStep.controlStructure = 0x00030111;
	Motor_Inits.FirstStep.commutationSensors = 0x00000030;
	Motor_Inits.FirstStep.axisConfigMiscellaneous = 0x00000000;
	Motor_Inits.FirstStep.currentControllerP_Gain = 643609;
	Motor_Inits.FirstStep.currentControllerI_Gain = 2791837;
	Motor_Inits.ModeOfOperation = cyclicSynchronousTorqueMode;

	struct Configuration_s Config;
	Config.Device = ankle;
	Config.Side = right;


/*******************************************************************************
* USER INITIALIZATIONS
*******************************************************************************/

	LL_SYSTICK_EnableIT();

	LL_LPTIM_Enable(LPTIM2);
	LL_LPTIM_EnableIT_ARRM(LPTIM2);
	LL_LPTIM_SetAutoReload(LPTIM2, LPTIM2_PERIOD);
	LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

	LL_SPI_Enable(SPI1);
	LL_SPI_Enable(SPI2);
	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);

	if(MPU925x_Init(SPI1, IMU_CS_GPIO_Port, IMU_CS_Pin))
		Error_Handler();
	MPU925x_SetAccelSensitivity(mpu925x_accelSensitivity_8g);
	MPU925x_SetGyroSensitivity(mpu925x_gyroSensitivity_1000dps);

	AS5145B_Init(&MagEnc);
	EPOS4_Init(&Motor_Inits, &CAN_Controller_Inits);

	InitProsthesisControl(Config);

	for(uint16_t i = 0; i < 1000; i++);		// Remove spikes from beginning


/*******************************************************************************
* USER TEST PROGRAMS
*******************************************************************************/

	RequireTestProgram(readOnly);


/*******************************************************************************
* USER MAIN LOOP
*******************************************************************************/

  while(1)
  {
	  if(isProsthesisControlRequired)
	  {
		  RunProsthesisControl();
		  isProsthesisControlRequired = 0;
	  }


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
