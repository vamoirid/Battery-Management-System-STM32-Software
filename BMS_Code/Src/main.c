/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LT_SPI.h"
#include "LTC6811.h"
#include "LTC681x.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
const uint8_t TOTAL_IC = 1;	//total number of ICs
cell_asic bms_ic[TOTAL_IC];	//the cell_asic struct objects 

//ADC VARIABLES
uint16_t adcVal_100Hz[6]; //Array that holds the ADC2 values (100Hz)
uint16_t adcVal_10Hz[5]; //Array that holds the ADC3 values (10Hz)
uint16_t CurrentMeasurement; //the dc current that ADC1 reads at 1KHz

uint16_t HVp, HVm, AIRp, AIRm, HVout, AIRout;	//the adc 100Hz values
uint16_t internalTemperature, DCDC_Thermistor, Thermistor1, Thermistor2, Thermistor3;	//the adc 10Hz values 

float Idc;	//the Idc (tractive system current)
double testTemperatures[TOTAL_IC][5];	//the temperatures for error checking (converted aux voltages)

//DIGITAL INPUT/OUTPUT VARIABLES
uint8_t BMS_State, IMD_State, AdAct_State, Precharge_State, Digital_Input1, Digital_Input2;	//the digital values read and transmitted at 100Hz
uint8_t BMS_State_Out = 1;	//the BMS state GPIO output

//CAN VARIABLES
static CAN_TxHeaderTypeDef myTxHeader;
static CAN_RxHeaderTypeDef myRxHeader;
uint32_t canTxMailbox = 0;	
uint16_t txID = 0;	//The id with which we send messages (it changes later inside the main loop)
uint16_t id_cell = 204;	//the cell voltage CAN transmit starting ID
uint16_t id_temp = 225;	//the aux voltage CAN transmit starting ID

//received variables
uint8_t rx_data[8];	//holds the received CAN data
uint8_t reset = 0;	//0-> nothing happens, 1-> system reset
float OV = 4.24; //overvoltage	4.24
float UV = 3.3;	//undervoltage	3.3
uint16_t OT = 70;	//overtemperature	70
uint16_t OC = 400;	//overcurrent 400
uint8_t N_Error = 10;	//number of allowed errors (experimental values)

//LTC CONFIGURATION VARIABLES
bool REFON = true; //!< Reference Powered Up Bit (true means Vref remains powered on between conversions)
bool ADCOPT = true; //!< ADC Mode option bit	(true chooses the second set of ADC frequencies)
bool gpioBits_a[5] = {false,false,false,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5 (false -> pull-down on)
bool dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12 (all false -> no discharge enabled)
bool dctoBits[4] = {false, false, false, false}; //!< Discharge time value // Dcto 0,1,2,3	(all false -> discharge timer disabled)

//ERROR VARIABLES
int8_t cvError = 0,auxError = 0;	//hold if an error has occured while reading cell voltage and aux voltage values
int NV [TOTAL_IC][12];	//total number of voltage measurement errors per IC per cell 
int NT [TOTAL_IC][5];	//total number of temperature measurement errors per IC per cell
int NPEC_V = 0;	//number of communication errors in cell voltage measurement
int NPEC_T = 0;	//number of communication errors in temperature voltage measurement

//TESTING VARIABLES
float voltages[12];	//holds the converted voltages of each cell for 1 BMS slave (testing)
float temperatures[5];	//holds the received aux voltage analog value for 1 BMS slave (testing)

//TIMING VARIABLES
uint16_t time = 0, time2 = 0, time3 = 0, time4 = 0, flag = 0, frequency = 0, counter = 0;	//timing and checking variables
uint16_t timeIT10 = 0, timeIT100 = 0, timeIT1K = 0, tim6counter = 0, tim7counter = 0, tim9counter = 0, tim11counter = 0;	//counters to check how many times each timer is triggered

//FLAGS
bool flag10Hz = false, flag100Hz = false;	//triggred byt TIM2 and TIM3 respectively
bool flagStartCVConversion = false, flagReadCVConversion = false, flagStartAuxConversion = false, flagReadAuxConversion = false;	//triggered by TIM6, TIM7, TIM9, TIM11 respectively every 12.5ms
bool flagErrorCheck = false;	//triggered by TIM11 (every 50ms)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void LTC6811_init(void);	//Initializes the LTC and the SPI communication
void CanFilterConfig(void);	//Configures the CAN filter
void errorCounterInit(void);	//A nested for-loop that initializes the error counting variables to 0
void errorCheck(void);	//Checks for errors
void state_change(void);	//Changes the BMS_State_Out variable if the number of errors exceed a certain number
void canTransmitCvAux(void);	//Updates the can tx arrays and sends them in the CAN bus when the TIM2 is triggered (10Hz)
void canTransmit10Hz(void); // Updates the can tx arrays and sends them in the CAN bus when the TIM2 is triggered (10Hz)
void canTransmit100Hz(void); // Updates the can tx arrays and sends them in the CAN bus when the TIM3 is triggered (100Hz)
void canTransmit1KHz(void);	//Updates the can tx array and sends it in the CAN bus when the TIM10 is triggered (1KHz)
void tempConvert(void);	//Converts the read temperature voltages to temperatures (based on the NTC thermistor datasheet)
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

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	//START CAN
	//configure outgoing CAN message template
	myTxHeader.IDE = CAN_ID_STD;
	myTxHeader.StdId = txID;	//ID changes accordingly every time a message is transmitted
	myTxHeader.RTR = CAN_RTR_DATA;
	myTxHeader.DLC = 8;	//DLC changes accordingly every time a message is transmitted
	
	CanFilterConfig();	//configures the filter for the CAN communication
	HAL_CAN_Start(&hcan1);	//starts CAN module
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	//enables the message-pending-callback for FIFO 0
	
	//initialize error counter and LTC/SPI
	errorCounterInit();	//initializes the error counting arrays
	LTC6811_init();	//initializes the LTC and SPI communication

	//START TIMERS
	HAL_TIM_Base_Start_IT(&htim2);	//10Hz
	HAL_TIM_Base_Start_IT(&htim3);	//100Hz
	HAL_TIM_Base_Start(&htim8);	//1Khz
	HAL_TIM_Base_Start_IT(&htim6);	//5Hz
	HAL_TIM_Base_Start_IT(&htim10);	//1Khz
	
	//START ADCs
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcVal_10Hz, 5);	//ADC1 is triggered by TIM2 (10Hz)
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcVal_100Hz, 6);	//ADC2 is triggered by TIM3 (100Hz)
	HAL_ADC_Start_IT(&hadc3);	//ADC3 is triggered by TIM8 (1KHz)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	time = HAL_GetTick();
  while (1)
  {
			time2 = HAL_GetTick();	//There is another timing variable at the end of the loop, both of which are used to check the time that the loop needs to run in ms
			if(counter++ > 255){
				counter=0;	//variable to check if the loop is running
			}
			flag++;
			if(HAL_GetTick() - time > 1000){
				frequency = flag;
				flag = 0;
				time = HAL_GetTick();
			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			
			//TIM6 (5Hz)
			if(flagStartCVConversion){
				wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
				LTC6811_adcv(MD_27KHZ_14KHZ, DCP_DISABLED, CELL_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
				flagStartCVConversion = false;
			}
			//TIM7 (5Hz)
			if(flagReadCVConversion){
				wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
				cvError = LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic);
				for(int i=0;i<12;i++)
					voltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001;
				flagReadCVConversion = false;
			}
			//TIM9 (5Hz)
			if(flagStartAuxConversion){
				wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
				LTC6811_adax(MD_27KHZ_14KHZ, AUX_CH_ALL); //should take 1.1 ms for 27KHz mode and 1.3 ms for 14KHz mode
				flagStartAuxConversion = false;
			}
			//TIM11 (5Hz)
			if(flagReadAuxConversion){
				wakeup_idle(TOTAL_IC);	//wake up daisy-chain from sleep mode
				auxError = LTC6811_rdaux(AUX_CH_ALL, TOTAL_IC, bms_ic);
				for(int i=0;i<5;i++)
						temperatures[i] = bms_ic[0].aux.a_codes[i];
				flagReadAuxConversion = false;
			} 
			
			//Checks if there is an error every 50ms (TIM11)
			if(flagErrorCheck){
				errorCheck();
				state_change();
				flagErrorCheck = false;
			}
			
			//CAN transmit 100Hz
			if(flag100Hz){
				canTransmit100Hz();
				flag100Hz = false;
			}
			//CAN transmit 10Hz
			if(flag10Hz){
				canTransmitCvAux();
				canTransmit10Hz();
				flag10Hz = false;
			}
		//Checks if the ECU requested a system reset
			if(reset == 1){
				HAL_NVIC_SystemReset();
				HAL_Delay(500);
			}
		time3 = HAL_GetTick();
		time4 = time3-time2;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 6;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 640;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 900;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1250;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 900;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1250;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 128;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1280;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1250;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 128;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1280;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1250;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LTC6811_CS_Pin|BMS_State_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LTC6811_CS_Pin BMS_State_Out_Pin */
  GPIO_InitStruct.Pin = LTC6811_CS_Pin|BMS_State_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Precharge_State_Pin AdAdct_State_Pin IMD_State_Pin BMS_State_Pin */
  GPIO_InitStruct.Pin = Precharge_State_Pin|AdAdct_State_Pin|IMD_State_Pin|BMS_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Digital_Input2_Pin Digital_Input1_Pin */
  GPIO_InitStruct.Pin = Digital_Input2_Pin|Digital_Input1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

////******USER FUNCTIONS******////

//Initializes the LTC's registers and the SPI communication
void LTC6811_init(){
	LTC6811_Initialize();	//Initializes the SPI communication at 1MHz
	LTC6811_init_cfg(TOTAL_IC, bms_ic);	//Initializes the confiugration registers to all 0s
	//This for loop initializes the configuration register variables 
	for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6811_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a);
  }
	LTC6811_reset_crc_count(TOTAL_IC,bms_ic);	//sets the CRC count to 0
	LTC6811_init_reg_limits(TOTAL_IC, bms_ic);	//Initializes the LTC's register limits for LTC6811 (because the generic LTC681x libraries can also be used for LTC6813 and others)
	wakeup_sleep(TOTAL_IC);
	LTC6811_wrcfg(TOTAL_IC,bms_ic);	//writes the configuration variables in the configuration registers via SPI
}

//Configures the CAN communication filter
void CanFilterConfig(){
	CAN_FilterTypeDef filterConfig;
	
	filterConfig.FilterBank = 0;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.FilterFIFOAssignment = 0;
	filterConfig.FilterIdHigh = 0x0000; //ID you want to allow to pass and shift it left by 5
	filterConfig.FilterIdLow = 0x0000;
	filterConfig.FilterMaskIdHigh = 0x0000; //if(Mask & ID == ID) --> Allow this packet to pass
	filterConfig.FilterMaskIdLow = 0x0000;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	filterConfig.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

//Initializes the error counting arrays
void errorCounterInit(){
	for (int i = 0; i < TOTAL_IC; i++){
    for (int j = 0; j < 12; j++){
      NV[i][j] = 0;
      if (j < 5)
        NT[i][j] = 0;
    }
  }
}

//Checks if the number of errors exeeds a certain limit and updates the BMS_State_Out variable accordingly and then calls state_change()
void errorCheck(){
	//Check for overvoltage or undervoltage and increase the counting arrays accordingly
			for (int i = 0; i < TOTAL_IC; i++){
				for (int j = 0; j < 12; j++){
        if (bms_ic[i].cells.c_codes[j] * 0.0001 > OV || bms_ic[i].cells.c_codes[j] * 0.0001 < UV){
					NV[i][j]++;
        }
        else{
					NV[i][j] = 0;
        }
				}
			}
			//Check for overcurrent
			if (Idc > OC){
				for (int i = 0; i < TOTAL_IC; i++){
					for (int j = 0; j < 12; j++){
						NV[i][j]++;
						if (j < 5){
							NT[i][j]++;
						}
					}
				}
			}
			//Check for OT
			tempConvert();
			for (int i = 0; i < TOTAL_IC; i++){
				for(int j = 0; j < 5; j++){
					if(testTemperatures[i][j] > OT){
						NT[i][j]++;
					}else{
						NT[i][j] = 0;
					}
				}
			}
			////******BMS State Change******////
			BMS_State_Out = 1;
			for (int i = 0; i < TOTAL_IC; i++){
        for (int j = 0; j < 12; j++){
          if ( NV[i][j] > N_Error){
           BMS_State_Out = 0;
          }
				}
       }
			for (int i = 0; i < TOTAL_IC; i++){
				for (int j = 0; j < 5; j++){
					if (NT[i][j] > N_Error * 2){
						BMS_State_Out = 0;
					}
				}
      }
			if ( (NPEC_V > N_Error * 5) || (NPEC_T > N_Error * 10) ){
				BMS_State_Out = 0;
      }
}

//Changes the BMS_State_Out pin to high if there is no problem in the BMS and low if there is
void state_change(){
	if (BMS_State_Out != 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
    return;
  }

  if (BMS_State_Out == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
    //HAL_Delay(1);
  }
}

//Converts the read temperature voltages to temperatures
void tempConvert(){
	for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    testTemperatures[current_ic][0] = (double)bms_ic[current_ic].aux.a_codes[0] * 0.0015;
    testTemperatures[current_ic][0] = (testTemperatures[current_ic][0] * 10000) / (5.275 - testTemperatures[current_ic][0]);
    testTemperatures[current_ic][0] = 1 / (0.00335401643468053 + 0.000256523550896126 * log(testTemperatures[current_ic][0] / 10000) + 2.60597012072052 * 0.000001 * (pow(log(testTemperatures[current_ic][0] / 10000), 2)) + 6.32926126487455 * 0.00000001 * (pow(log(testTemperatures[current_ic][0] / 10000), 3)) );
    testTemperatures[current_ic][0] = testTemperatures[current_ic][0] - 274.15;

    testTemperatures[current_ic][1] = (double)bms_ic[current_ic].aux.a_codes[1] * 0.0015;
    testTemperatures[current_ic][1] = (testTemperatures[current_ic][1] * 10000) / (5.275 - testTemperatures[current_ic][1]);
    testTemperatures[current_ic][1] = 1 / (0.00335401643468053 + 0.000256523550896126 * log(testTemperatures[current_ic][1] / 10000) + 2.60597012072052 * 0.000001 * (pow(log(testTemperatures[current_ic][1] / 10000), 2)) + 6.32926126487455 * 0.00000001 * (pow(log(testTemperatures[current_ic][1] / 10000), 3)) );
    testTemperatures[current_ic][1] = testTemperatures[current_ic][1] - 274.15;
		
		testTemperatures[current_ic][2] = (double)bms_ic[current_ic].aux.a_codes[2] * 0.0015;
    testTemperatures[current_ic][2] = (testTemperatures[current_ic][2] * 10000) / (5.275 - testTemperatures[current_ic][2]);
    testTemperatures[current_ic][2] = 1 / (0.00335401643468053 + 0.000256523550896126 * log(testTemperatures[current_ic][2] / 10000) + 2.60597012072052 * 0.000001 * (pow(log(testTemperatures[current_ic][2] / 10000), 2)) + 6.32926126487455 * 0.00000001 * (pow(log(testTemperatures[current_ic][2] / 10000), 3)) );
    testTemperatures[current_ic][2] = testTemperatures[current_ic][2] - 274.15;
		
		testTemperatures[current_ic][3] = (double)bms_ic[current_ic].aux.a_codes[3] * 0.0015;
    testTemperatures[current_ic][3] = (testTemperatures[current_ic][3] * 10000) / (5.275 - testTemperatures[current_ic][3]);
    testTemperatures[current_ic][3] = 1 / (0.00335401643468053 + 0.000256523550896126 * log(testTemperatures[current_ic][3] / 10000) + 2.60597012072052 * 0.000001 * (pow(log(testTemperatures[current_ic][3] / 10000), 2)) + 6.32926126487455 * 0.00000001 * (pow(log(testTemperatures[current_ic][3] / 10000), 3)) );
    testTemperatures[current_ic][3] = testTemperatures[current_ic][3] - 274.15;
		
		testTemperatures[current_ic][4] = (double)bms_ic[current_ic].aux.a_codes[4] * 0.0015;
    testTemperatures[current_ic][4] = (testTemperatures[current_ic][4] * 10000) / (5.275 - testTemperatures[current_ic][4]);
    testTemperatures[current_ic][4] = 1 / (0.00335401643468053 + 0.000256523550896126 * log(testTemperatures[current_ic][4] / 10000) + 2.60597012072052 * 0.000001 * (pow(log(testTemperatures[current_ic][4] / 10000), 2)) + 6.32926126487455 * 0.00000001 * (pow(log(testTemperatures[current_ic][3] / 10000), 3)) );
    testTemperatures[current_ic][4] = testTemperatures[current_ic][4] - 274.15;
  }
}

//Transmits all cell voltages and aux voltages(temperatures) in the CAN bus
void canTransmitCvAux(){

	id_cell = 204;
	id_temp = 225;
	myTxHeader.StdId = id_cell;
	myTxHeader.DLC = 8;
			
	for (int l = 0; l < TOTAL_IC; l++){
		uint8_t data[8] = {bms_ic[l].cells.c_codes[0] >> 8,  bms_ic[l].cells.c_codes[0] & 0xFF,  bms_ic[l].cells.c_codes[1] >> 8,  bms_ic[l].cells.c_codes[1] & 0xFF,  bms_ic[l].cells.c_codes[2] >> 8,  bms_ic[l].cells.c_codes[2] & 0xFF,  bms_ic[l].cells.c_codes[3] >> 8,  bms_ic[l].cells.c_codes[3] & 0xFF};
		HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data, &canTxMailbox);
		while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
		id_cell++;
		myTxHeader.StdId = id_cell;
				
		uint8_t data1[8] = {bms_ic[l].cells.c_codes[4] >> 8,  bms_ic[l].cells.c_codes[4] & 0xFF,  bms_ic[l].cells.c_codes[5] >> 8,  bms_ic[l].cells.c_codes[5] & 0xFF,  bms_ic[l].cells.c_codes[6] >> 8,  bms_ic[l].cells.c_codes[6] & 0xFF,  bms_ic[l].cells.c_codes[7] >> 8,  bms_ic[l].cells.c_codes[7] & 0xFF};
		HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data1, &canTxMailbox);
		while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
		id_cell++;
		myTxHeader.StdId = id_cell;
					
		uint8_t data2[8] = {bms_ic[l].cells.c_codes[8] >> 8,  bms_ic[l].cells.c_codes[8] & 0xFF,  bms_ic[l].cells.c_codes[9] >> 8,  bms_ic[l].cells.c_codes[9] & 0xFF,  bms_ic[l].cells.c_codes[10] >> 8,  bms_ic[l].cells.c_codes[10] & 0xFF,  bms_ic[l].cells.c_codes[11] >> 8,  bms_ic[l].cells.c_codes[11] & 0xFF};
		HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data2, &canTxMailbox);
		while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
		id_cell++;
		myTxHeader.StdId = id_cell; 
	}
	
	myTxHeader.StdId = id_temp;
	myTxHeader.DLC = 8;
	for (int i = 0; i < TOTAL_IC; i++){
		uint8_t datatemp[8] = {bms_ic[i].aux.a_codes[0] >> 8,  bms_ic[i].aux.a_codes[0] & 0xFF,  bms_ic[i].aux.a_codes[1] >> 8,  bms_ic[i].aux.a_codes[1] & 0xFF, bms_ic[i].aux.a_codes[2] >> 8,  bms_ic[i].aux.a_codes[2] & 0xFF,  bms_ic[i].aux.a_codes[3] >> 8,  bms_ic[i].aux.a_codes[3] & 0xFF};
		HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, datatemp, &canTxMailbox);
		while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
		id_temp++;
		myTxHeader.StdId = id_temp;
			
		uint8_t datatemp1[8] = {bms_ic[i].aux.a_codes[4] >> 8,  bms_ic[i].aux.a_codes[4] & 0xFF, 0, 0, 0,  0,  0, 0};
		HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, datatemp1, &canTxMailbox);
		while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
		id_temp++;
		myTxHeader.StdId = id_temp;
	}  
}

//Transmits the Thermistor values and the digital values read at 10Hz
void canTransmit10Hz(){
	txID = 301;
	myTxHeader.StdId = txID;
	myTxHeader.DLC = 8;
	uint8_t data4[8] = {adcVal_10Hz[0] >> 8, adcVal_10Hz[0] & 0xFF, adcVal_10Hz[1] >> 8, adcVal_10Hz[1] & 0xFF, adcVal_10Hz[2] >> 8, adcVal_10Hz[2] & 0xFF, adcVal_10Hz[3] >> 8, adcVal_10Hz[3] & 0xFF};			
	HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data4, &canTxMailbox);
	while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
	
	//Digital values
	txID = 305;
	myTxHeader.StdId = txID;
	myTxHeader.DLC = 8;
	BMS_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	IMD_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	AdAct_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
	Precharge_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	Digital_Input1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	Digital_Input2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	
	uint8_t data5[8] = {adcVal_10Hz[4] >> 8, adcVal_10Hz[4] & 0xFF, BMS_State, IMD_State, AdAct_State, Precharge_State, Digital_Input1, Digital_Input2};
	HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data5, &canTxMailbox);
	while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
}


//Transmits the analog values read at 100Hz
void canTransmit100Hz(){
	txID = 258;
	myTxHeader.StdId = txID;
	myTxHeader.DLC = 8;
	uint8_t data3[8] = {adcVal_100Hz[0] >> 8, adcVal_100Hz[0] & 0xFF, adcVal_100Hz[1] >> 8, adcVal_100Hz[1] & 0xFF, adcVal_100Hz[2] >> 8, adcVal_100Hz[2] & 0xFF, adcVal_100Hz[3] >> 8, adcVal_100Hz[3] & 0xFF};
	HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data3, &canTxMailbox);
	while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
				
	txID = 267;
	myTxHeader.StdId = txID;
	myTxHeader.DLC = 4;
	uint8_t data4[4] = {adcVal_100Hz[4] >> 8, adcVal_100Hz[4] & 0xFF, adcVal_100Hz[5] >> 8, adcVal_100Hz[5] & 0xFF};
	HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data4, &canTxMailbox);
	while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
}

//Transmits the analog values read at 1KHz
void canTransmit1KHz(){
		txID = 401;
		myTxHeader.StdId = txID;
		myTxHeader.DLC = 2;
		uint8_t data1K[8] = {CurrentMeasurement >> 8, CurrentMeasurement & 0xFF};
		HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data1K, &canTxMailbox);
		while(HAL_CAN_IsTxMessagePending(&hcan1, canTxMailbox));
}
////******INTERRUPT SERVICE ROUTINES******////

//ADC conversion complete callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &hadc3) {
		CurrentMeasurement = HAL_ADC_GetValue(&hadc3);
		Idc = (float)(CurrentMeasurement) * (3.3 / 4095);
		Idc = (Idc - 2.66) / 5.7 * 1000.0;	//2.66 or 2.5 check
	}
	
}

//When half the timer's period is complete we send the messages in the CAN bus
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//100Hz transmit flag enable
	if (htim->Instance == TIM3){
		flag100Hz = true;
		if(timeIT100 ++ > 65535) timeIT100 = 0;
	}
	//10Hz CAN transmit
	else if(htim->Instance == TIM2){
		flag10Hz = true; 
		if(timeIT10 ++ > 65535) timeIT10 = 0;
		
	}
	//1KHz transmit (1Khz CAN transmit happens here because the while loop runs at more than 1ms at certain points in time which will not allow the 1kHz transmit to happen in time)
	else if(htim->Instance == TIM10){ 
		canTransmit1KHz();
		if(timeIT1K ++ > 65535) timeIT1K = 0;
		
	}
	//TIM6 starts first and then triggers TIM7, which triggers TIM9, which triggers TIM11, which triggers TIM6. These 4 timers run in a loop at 5Hz each, and 20Hz overall
	//TIM6 starts cell voltage conversion
	else if(htim->Instance == TIM6){
		HAL_TIM_Base_Stop_IT(&htim6);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim6 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim6, 0);	//resets the timer's counter to 0
		flagStartCVConversion = true;	//enables the cell voltages conversion code block
		if(tim6counter ++ > 65535) tim6counter = 0;
		HAL_TIM_Base_Start_IT(&htim7);	//starts the next timer (TIM7)
		
	}//TIM7 reads cell voltages
	else if(htim->Instance == TIM7){
		HAL_TIM_Base_Stop_IT(&htim7);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim7 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim7, 0);	//resets the timer's counter to 0
		flagReadCVConversion = true;	//enables the read cell voltages code block
		if(tim7counter ++ > 65535) tim7counter = 0;
		HAL_TIM_Base_Start_IT(&htim9);	//starts the next timer (TIM9)
		
	}//TIM9 starts aux voltage conversion
	else if(htim->Instance == TIM9){
		HAL_TIM_Base_Stop_IT(&htim9);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim9 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim9, 0);	//resets the timer's counter to 0
		flagStartAuxConversion = true;	//enables the aux voltages conversion code block
		if(tim9counter ++ > 65535) tim9counter = 0;
		HAL_TIM_Base_Start_IT(&htim11);	//starts the next timer (TIM11)
		
	}//TIM11 reads aux voltage conversion
	else if(htim->Instance == TIM11){
		HAL_TIM_Base_Stop_IT(&htim11);	//stops this timer
		__HAL_TIM_CLEAR_IT(&htim11 ,TIM_IT_UPDATE);	//clears the IT flag
		__HAL_TIM_SET_COUNTER(&htim11, 0);	//resets the timer's counter to 0
		flagReadAuxConversion = true;	//enables the read aux voltages code block
		flagErrorCheck = true;	//enables the error chack and state change code block
		if(tim11counter ++ > 65535) tim11counter = 0;
		HAL_TIM_Base_Start_IT(&htim6);	//starts the next timer (TIM6)
	}
}

//Receives data from the ECU and sets certain parameters for the BMS in general
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &myRxHeader, rx_data) != HAL_OK){
		Error_Handler();
	}
	//receive overvoltage, undervoltage, overcurrent, overtemperature values
	if(myRxHeader.StdId == 56){
			OV = (float)(((rx_data[0] << 8) | (rx_data[1])) / 100.0);
			UV = (float)(((rx_data[2] << 8) | (rx_data[3])) / 100.0);
			OC = (rx_data[4] << 8) | (rx_data[5]);
			OT = (rx_data[6] << 8) | (rx_data[7]);
	}
	//receive number of errors and the reset variable
	if(myRxHeader.StdId == 57){
			N_Error = (rx_data[0] << 8) | (rx_data[1]);
			reset = rx_data[2];
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
