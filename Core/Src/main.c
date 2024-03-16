/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEGMENT_BASE_CAN_ID 0x10

#define SERVO_OFFSET_ARRAY_LENGTH 4

#define ENABLE_LINEAR_SLEW

// TODO: Determine these values experimentally by homing each segment

// Define offset array based on CAN ID
#if SEGMENT_BASE_CAN_ID == 0x10
int8_t servo_home_offsets[SERVO_OFFSET_ARRAY_LENGTH] = {1, 6, 0, 0};
#elif SEGMENT_BASE_CAN_ID == 0x20
int8_t servo_home_offsets[SERVO_OFFSET_ARRAY_LENGTH] = {30, 90, 0, 0};
#elif SEGMENT_BASE_CAN_ID == 0x30
int8_t servo_home_offsets[SERVO_OFFSET_ARRAY_LENGTH] = {50, -50, 0, 0};
#elif SEGMENT_BASE_CAN_ID == 0x40
int8_t servo_home_offsets[SERVO_OFFSET_ARRAY_LENGTH] = {115, -115, 0, 0};
#elif SEGMENT_BASE_CAN_ID == 0x50
int8_t servo_home_offsets[SERVO_OFFSET_ARRAY_LENGTH] = {-90, 90, 0, 0};
#endif

#define PI 3.1415926535897932384626433

#define q31_to_f32(x) ldexp((int32_t) x, -31)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CORDIC_HandleTypeDef hcordic;

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
// 15 degree phase difference between each segment
uint16_t phaseAngle = 0 + 15 * ((SEGMENT_BASE_CAN_ID >> 4) - 1);

uint16_t targetServoPWMAngle[4];

FDCAN_RxHeaderTypeDef rxHeader;
FDCAN_TxHeaderTypeDef txHeader;
uint8_t rxData[8];
uint8_t txData[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
uint32_t degreesToPWM(float degrees);
uint16_t speedToWaveTimerPeriod(int8_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Converts a 32-bit float to a q1.31 notation int
static inline int f32_to_q31(double input)
{
	const float Q31_MAX_F = 0x0.FFFFFFp0F;
	const float Q31_MIN_F = -1.0F;
	return (int)roundf(scalbnf(fmaxf(fminf(input, Q31_MAX_F), Q31_MIN_F), 31));
}

// Computes the sin of x in radians
static inline float cordic_q31_sinf(float x)
{
	CORDIC_ConfigTypeDef sConfig;
	int32_t input_q31 = f32_to_q31(fmod(x, 2.0f * PI) / (2.0f * PI)) << 1;
	int32_t output_q31;

	sConfig.Function = CORDIC_FUNCTION_SINE;
	sConfig.Precision = CORDIC_PRECISION_6CYCLES;
	sConfig.Scale = CORDIC_SCALE_0;
	sConfig.NbWrite = CORDIC_NBWRITE_1;
	sConfig.NbRead= CORDIC_NBREAD_1;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize = CORDIC_OUTSIZE_32BITS;
	HAL_CORDIC_Configure(&hcordic, &sConfig);
	HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31, 1, 0);

	return q31_to_f32(output_q31);
}

// Computes the cos of x in radians
static inline float cordic_q31_cosf(float x)
{
	CORDIC_ConfigTypeDef sConfig;
	int32_t input_q31 = f32_to_q31(fmod(x, 2.0f * PI) / (2.0f * PI)) << 1;
	int32_t output_q31;

	sConfig.Function = CORDIC_FUNCTION_COSINE;
	sConfig.Precision = CORDIC_PRECISION_6CYCLES;
	sConfig.Scale = CORDIC_SCALE_0;
	sConfig.NbWrite = CORDIC_NBWRITE_1;
	sConfig.NbRead= CORDIC_NBREAD_1;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize = CORDIC_OUTSIZE_32BITS;
	HAL_CORDIC_Configure(&hcordic, &sConfig);
	HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31, 1, 0);

	return q31_to_f32(output_q31);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// Check for new messages
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		// Retrieve message
		if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
		{
			Error_Handler();
		}

		// Check header length
		// Will likely need to be converted to a switch statement as more commands are implemented
		if(rxHeader.DataLength == FDCAN_DLC_BYTES_2)
		{
			// Stop metachronal timer if necessary
			HAL_TIM_Base_Stop_IT(&htim6);

#ifdef ENABLE_LINEAR_SLEW
			// Change all target servo PWM angles to current values of TIM1->CCRx to maintain their positions
			targetServoPWMAngle[0] = TIM1->CCR1;
			targetServoPWMAngle[1] = TIM1->CCR2;
			targetServoPWMAngle[2] = TIM1->CCR3;
			targetServoPWMAngle[3] = TIM1->CCR4;
#endif

			// Get angle from message
			uint16_t tmp = 0;

			tmp += rxData[0];
			tmp <<= 8;
			tmp += rxData[1];

			// Extract servo number from LSB of identifier
			switch(rxHeader.Identifier & 0x00F)
			{
				case 0x0:
					tmp += servo_home_offsets[0];
#ifdef ENABLE_LINEAR_SLEW
					targetServoPWMAngle[0] = degreesToPWM(tmp);
#else
					TIM1->CCR1 = degreesToPWM(tmp);
#endif
					break;
				case 0x1:
					tmp += servo_home_offsets[1];
#ifdef ENABLE_LINEAR_SLEW
					targetServoPWMAngle[1] = degreesToPWM(tmp);
#else
					TIM1->CCR2 = degreesToPWM(tmp);
#endif
					break;
				case 0x2:
					tmp += servo_home_offsets[2];
#ifdef ENABLE_LINEAR_SLEW
					targetServoPWMAngle[2] = degreesToPWM(tmp);
#else
					TIM1->CCR3 = degreesToPWM(tmp);
#endif
					break;
				case 0x3:
					tmp += servo_home_offsets[3];
#ifdef ENABLE_LINEAR_SLEW
					targetServoPWMAngle[3] = degreesToPWM(tmp);
#else
					TIM1->CCR4 = degreesToPWM(tmp);
#endif
					break;
			}
#ifdef ENABLE_LINEAR_SLEW
			// Start linear slew timer
			HAL_TIM_Base_Start_IT(&htim7);
#endif

			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
		else if(rxHeader.DataLength == FDCAN_DLC_BYTES_1)
		{
			if(rxHeader.RxFrameType == FDCAN_DATA_FRAME)
			{
				// Stop metachronal wave when a single zero is received
				if(rxData[0] == 0)
				{
					HAL_TIM_Base_Stop_IT(&htim6);
				}
				else
				{
#ifdef ENABLE_LINEAR_SLEW
					// Stop linear slew timer
					HAL_TIM_Base_Stop_IT(&htim7);

					//TODO: If one or more individual servos have been moved, move all servos
						// to last position in wave before resuming using linear slew to reduce current spikes
						// This would fix an edge case where a current spike could be caused by many servos suddenly
						// jumping to their position in the metachronal wave when it is resumed.
						// Should only matter if a significant number of servos are moved from their position in the wave.
#endif

					// Update metachronal wave period
					TIM6->ARR = speedToWaveTimerPeriod((int8_t) rxData[0]);

					HAL_TIM_Base_Start_IT(&htim6);
				}

				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
			else if(rxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
			{
				// Send heart beat response
				txData[0] = SEGMENT_BASE_CAN_ID;
				txHeader.DataLength = FDCAN_DLC_BYTES_1;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
		}

		// Re-activate message notifications
		// HAL disables them when a message is received, so they need to be reactivated after every message
		if(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		{
			Error_Handler();
		}
	}
}

uint32_t degreesToPWM(float degrees)
{
	// Validate range
	if(degrees < 0)
	{
		degrees = 0.0f;
	}
	else if(degrees > 270)
	{
		degrees = 270.0f;
	}

	// newValue = (-1 if flipped, 1 if not) * oldValue * (newRange / oldRange) + newRangeOffset
	return ((float) degrees) * (2000.0f / 270.0f) + 500;
}

uint16_t speedToWaveTimerPeriod(int8_t speed)
{
	if(speed <= 0)
	{
		return 0;
	}

	// newValue = (-1 if flipped, 1 if not) * oldValue * (newRange / oldRange) + newRangeOffset
	return -1 * ((int8_t) speed) * (7000.0 / 127.0) + 10000;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim6)
	{
		double radian = 0.0f;
		float cosResult = 0.0f;
		float sinResult = 0.0f;

		phaseAngle++;
		phaseAngle %= 360;

		// Convert degrees to radians
		radian = (double)(phaseAngle) * 2.0 * PI / 360.0;

		cosResult = cordic_q31_cosf(radian);
		sinResult = cordic_q31_sinf(radian);

		// TODO: Add stride length parameter

		// TODO: Optimize these assignments to avoid repeated computation
		// Right horizontal servo
		TIM1->CCR1 = degreesToPWM(floor(cosResult * 22.5f) + 135.0f + servo_home_offsets[0]);
		// Right vertical servo
		TIM1->CCR2 = degreesToPWM(floor(sinResult * 22.5f) + 135.0f + servo_home_offsets[1]);
		// Left horizontal servo
		TIM1->CCR3 = degreesToPWM(floor(cosResult * 22.5f) + 135.0f + servo_home_offsets[2]);
		// Left vertical servo
		TIM1->CCR4 = degreesToPWM(floor(sinResult * 22.5f) + 135.0f + servo_home_offsets[3]);

//		// Quadrature mode
//		TIM1->CCR1 = degreesToPWM(floor(sinResult * 90.0f) + 90.0f);
//		TIM1->CCR2 = degreesToPWM(floor(cosResult * 90.0f) + 90.0f);
//		TIM1->CCR3 = degreesToPWM(floor(sinResult * -90.0f) + 90.0f);
//		TIM1->CCR4 = degreesToPWM(floor(cosResult * -90.0f) + 90.0f);
	}
#ifdef ENABLE_LINEAR_SLEW
	else if(htim == &htim7)
	{
		// Nudge the TIM1->CCRx registers towards their target values
		if(TIM1->CCR1 < targetServoPWMAngle[0])
		{
			TIM1->CCR1++;
		}
		else if(TIM1->CCR1 > targetServoPWMAngle[0])
		{
			TIM1->CCR1--;
		}

		if(TIM1->CCR2 < targetServoPWMAngle[1])
		{
			TIM1->CCR2++;
		}
		else if(TIM1->CCR2 > targetServoPWMAngle[1])
		{
			TIM1->CCR2--;
		}

		if(TIM1->CCR3 < targetServoPWMAngle[2])
		{
			TIM1->CCR3++;
		}
		else if(TIM1->CCR3 > targetServoPWMAngle[2])
		{
			TIM1->CCR3--;
		}

		if(TIM1->CCR4 < targetServoPWMAngle[3])
		{
			TIM1->CCR4++;
		}
		else if(TIM1->CCR4 > targetServoPWMAngle[3])
		{
			TIM1->CCR4--;
		}
	}
#endif
}
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
  MX_LPUART1_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_CORDIC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  // Initialize PWM to 0
  // This should be zero because the servo will interpret this as no signal
  	  // and stay where it is until it is changed to a valid (non-zero) value
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  // Same with the target values
  targetServoPWMAngle[0] = 0;
  targetServoPWMAngle[1] = 0;
  targetServoPWMAngle[2] = 0;
  targetServoPWMAngle[3] = 0;

  // Start servo PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // Metachronal update timer
//  HAL_TIM_Base_Start_IT(&htim6);

  // Initialize txHeader
  txHeader.Identifier = SEGMENT_BASE_CAN_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  // ************************************************************************
  // HEY YOU!! Changing this from FDCAN_ESI_ACTIVE to FDCAN_ESI_PASSIVE might be the solution to the interface freezing
  // Research this bit field before changing it!
  // HAL_FDCAN_EnableTxDelay might also be worth exploring
  // ************************************************************************
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 17;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 17;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 14;
  hfdcan1.Init.DataTimeSeg2 = 5;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;

  	// Filter out all messages not meant for this segment
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = SEGMENT_BASE_CAN_ID;
	sFilterConfig.FilterID2 = 0x7FC;

    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
  	  Error_Handler();
    }

    // Accept messages from 0xFF (broadcast)
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x0FF;
	sFilterConfig.FilterID2 = 0x7FC;

	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

    // ****************************************
	// Reject all packets not match by a filter
    // Both data and remote frames can be received
    // FILTERS WILL NOT REJECT PACKETS WITHOUT THIS LINE
    // ****************************************
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 170-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim6.Init.Prescaler = 170-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2999;
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
  htim7.Init.Prescaler = 170-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1499;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
