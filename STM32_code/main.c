/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 //Frequency Register f_main = 160MHz  ARR = 65535  -> f_main/ARR ~2444
float P_factor = 0.3;
float D_factor = 0.01;
float I_factor = 0.01;

int16_t cumulativeError_M1 = 0;
int16_t cumulativeError_M2 = 0;
int16_t cumulativeError_M3 = 0;



typedef struct Command { //
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t throwerSpeed;
  uint16_t delimiter; //
} Command;
Command command = {.speed1 = 0, .speed2 = 0, .speed3 = 0, .throwerSpeed = 0, .delimiter = 0}; // (4)

typedef struct Feedback { //
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t delimiter;
} Feedback;

typedef struct Setpoints { //
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
} Setpoints;
Setpoints setpoints = {.speed1 = 0, .speed2 = 0, .speed3 = 0};

typedef struct Motor_pose { //
  int16_t pose_M1;
  int16_t pose_M2;
  int16_t pose_M3;
} Motor_pose;
Motor_pose motor_pose = {.pose_M1 = 0, .pose_M2 = 0, .pose_M3 = 0};

typedef struct Motor_pose_prev { //
  int16_t pose_prev_M1;
  int16_t pose_prev_M2;
  int16_t pose_prev_M3;
} Motor_pose_prev;
Motor_pose_prev motor_pose_prev = {.pose_prev_M1 = 0, .pose_prev_M2 = 0, .pose_prev_M3 = 0};

typedef struct Motor_pose_change { //
  int16_t pose_change_M1;
  int16_t pose_change_M2;
  int16_t pose_change_M3;
} Motor_pose_change;
Motor_pose_change motor_pose_change = {.pose_change_M1 = 0, .pose_change_M2 = 0, .pose_change_M3 = 0};

typedef struct Motor_speed_send { //
  int16_t speed_send_M1;
  int16_t speed_send_M2;
  int16_t speed_send_M3;
} Motor_speed_send;
Motor_speed_send motor_speed_send = {.speed_send_M1 = 0, .speed_send_M2 = 0, .speed_send_M3 = 0};


void PID_Controller(){

	float time_call = 1/100; // this is the time between function calls

	motor_pose_change.pose_change_M1 = motor_pose.pose_M1 - motor_pose_prev.pose_prev_M1;
	motor_pose_change.pose_change_M2 = motor_pose.pose_M2 - motor_pose_prev.pose_prev_M2;
	motor_pose_change.pose_change_M3 = motor_pose.pose_M3 - motor_pose_prev.pose_prev_M3;

	int16_t currentError_M1 = setpoints.speed1 - motor_pose_change.pose_change_M1;
	int16_t currentError_M2 = setpoints.speed2 - motor_pose_change.pose_change_M2;
	int16_t currentError_M3 = setpoints.speed3 - motor_pose_change.pose_change_M3;

	cumulativeError_M1 += currentError_M1;
	cumulativeError_M2 += currentError_M2;
	cumulativeError_M3 += currentError_M3;

	motor_speed_send.speed_send_M1 = currentError_M1 * P_factor + \
			        D_factor * motor_pose_change.pose_change_M1 + \
					I_factor * cumulativeError_M1;

	motor_speed_send.speed_send_M2 = currentError_M2 * P_factor + \
			        D_factor * motor_pose_change.pose_change_M2 + \
					I_factor * cumulativeError_M2;

	motor_speed_send.speed_send_M3 = currentError_M3 * P_factor + \
			        D_factor * motor_pose_change.pose_change_M3 + \
					I_factor * cumulativeError_M3;



//	motor_speed_send.speed_send_M1 = (setpoints.speed1 - motor_pose_change.pose_change_M1/time_call)*P_factor;
	//
	////				+ (setpoints.speed1 - motor_pose_change.pose_change_M1/time_call)/time_call*D_factor
	////				+ (setpoints.speed1 - motor_pose_change.pose_change_M1/time_call)*time_call*I_factor;

//	motor_speed_send.speed_send_M1 = (setpoints.speed1 - motor_pose_change.pose_change_M1/time_call)*P_factor;
//
////				+ (setpoints.speed1 - motor_pose_change.pose_change_M1/time_call)/time_call*D_factor
////				+ (setpoints.speed1 - motor_pose_change.pose_change_M1/time_call)*time_call*I_factor;
//
//	motor_speed_send.speed_send_M2 = (setpoints.speed2 - motor_pose_change.pose_change_M2/time_call)*P_factor \
//				+ (setpoints.speed2 - motor_pose_change.pose_change_M2/time_call)/time_call*D_factor \
//				+ (setpoints.speed2 - motor_pose_change.pose_change_M2/time_call)*time_call*I_factor;
//
//	motor_speed_send.speed_send_M3 = (setpoints.speed3 - motor_pose_change.pose_change_M3/time_call)*P_factor \
//				+ (setpoints.speed3 - motor_pose_change.pose_change_M3/time_call)/time_call*D_factor \
//				+ (setpoints.speed3 - motor_pose_change.pose_change_M3/time_call)*time_call*I_factor;
}

void processing_values(){

	if (motor_speed_send.speed_send_M1 >= 0){
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 1);
		if (motor_speed_send.speed_send_M1 > 65535){
			motor_speed_send.speed_send_M1 = 65535;
		}
	}
	else{
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin, 0);
		if (motor_speed_send.speed_send_M1 < -65535){
			motor_speed_send.speed_send_M1 = -65535;
		}
		motor_speed_send.speed_send_M1 = -motor_speed_send.speed_send_M1;
	}

	if (motor_speed_send.speed_send_M2 >= 0){
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 1);
		if (motor_speed_send.speed_send_M2 > 65535){
			motor_speed_send.speed_send_M2 = 65535;
		}
	}
	else{
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin, 0);
		if (motor_speed_send.speed_send_M2 < -65535){
			motor_speed_send.speed_send_M2 = -65535;
		}
		motor_speed_send.speed_send_M2 = -motor_speed_send.speed_send_M2;
	}

	if (motor_speed_send.speed_send_M3 >= 0){
		HAL_GPIO_WritePin(DIR_M3_GPIO_Port, DIR_M3_Pin, 1);
		if (motor_speed_send.speed_send_M3 > 65535){
			motor_speed_send.speed_send_M3 = 65535;
		}
	}
	else{
		HAL_GPIO_WritePin(DIR_M3_GPIO_Port, DIR_M3_Pin, 0);
		if (motor_speed_send.speed_send_M3 < -65535){
			motor_speed_send.speed_send_M3 = -65535;
		}
		motor_speed_send.speed_send_M3 = -motor_speed_send.speed_send_M3;
	}
}

void out_put_sleep (){
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  for (int i = 0; i < 350; i++) {
		  __asm("nop");
	  }
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  for (int i = 0; i < 350; i++) {
		  __asm("nop");
	   }
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	// ################# read Encoder values ##################
	motor_pose.pose_M1 = (int16_t)TIM1->CNT;
	motor_pose.pose_M2 = (int16_t)TIM4->CNT;
	motor_pose.pose_M3 = (int16_t)TIM8->CNT;

	// scale the value
	out_put_sleep();	 // nSleep control
	PID_Controller();	 // calculate velocities based on a PID controller
	processing_values(); // direction control

//	// changes duty cycle of the motors
//	TIM2->CCR1 = motor_speed_send.speed_send_M1;
// 	TIM2->CCR2 = motor_speed_send.speed_send_M2;
//	TIM2->CCR3 = motor_speed_send.speed_send_M3;

	motor_pose_prev.pose_prev_M1 = motor_pose.pose_M1;
	motor_pose_prev.pose_prev_M2 = motor_pose.pose_M2;
	motor_pose_prev.pose_prev_M3 = motor_pose.pose_M3;
}

volatile uint8_t isCommandReceived = 0; // (5)

void CDC_On_Receive(uint8_t* buffer, uint32_t* length) {
	// for testing the serial communication
  if (*length == sizeof(Command)) {
    memcpy(&command, buffer, sizeof(Command));

    if (command.delimiter == 0xAAAA) {
      isCommandReceived = 1;
    }
  }
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_USB_Device_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  Feedback feedback = { // (1)
      .speed1 = 0,
      .speed2 = 0,
      .speed3 = 0,
      .delimiter = 0xAAAA
  };
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2); // Encoder 1
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2); // Encoder 2
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1 | TIM_CHANNEL_2);	// Encoder 3
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// Motor 1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Motor 2
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Motor 3
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); // thrower motor
  HAL_TIM_Base_Start_IT(&htim6); // Timer for which indicates when to update the motor speeds



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




HAL_GPIO_TogglePin(Debug_LED_GPIO_Port, Debug_LED_Pin); // (3)
HAL_Delay(1000);
HAL_GPIO_TogglePin(Debug_LED_GPIO_Port, Debug_LED_Pin); // (3)
HAL_Delay(1000);
HAL_GPIO_TogglePin(Debug_LED_GPIO_Port, Debug_LED_Pin); // (3)


//HAL_GPIO_WritePin(DIR_M1_GPIO_Port, DIR_M1_Pin,1);
//HAL_GPIO_WritePin(DIR_M2_GPIO_Port, DIR_M2_Pin,1);
//HAL_GPIO_WritePin(DIR_M3_GPIO_Port, DIR_M3_Pin,1);

//TIM2->CCR1 = 9000;
//TIM2->CCR2 = 9600;
//TIM2->CCR3 = 9500;


TIM15->CCR2 = 3000; // for initialization of the ESP 3000 -> below 1 ms

  while (1)
  {
	HAL_Delay(20);

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
//      	TIM15->CCR1 = 9000;
	    if (isCommandReceived) {
	      isCommandReceived = 0;
	      TIM15->CCR2 = command.throwerSpeed;

	      HAL_GPIO_TogglePin(Debug_LED_GPIO_Port, Debug_LED_Pin); // (3)
	      HAL_Delay(500);
	      HAL_GPIO_TogglePin(Debug_LED_GPIO_Port, Debug_LED_Pin); // (3)

	      setpoints.speed1 = command.speed1;
	      setpoints.speed2 = command.speed2;
	      setpoints.speed3 = command.speed3;

	      feedback.speed1 = motor_pose_change.pose_change_M1;
	      feedback.speed2 = motor_speed_send.speed_send_M1;
	      feedback.speed3 = setpoints.speed1 - motor_pose_change.pose_change_M1*100;

//	      TIM2->CCR1  = motor_speed_send.speed_send_M1;
//	      TIM2->CCR2  = motor_speed_send.speed_send_M2;
//	      TIM2->CCR3  = motor_speed_send.speed_send_M3;

		  TIM2->CCR1 = command.speed1;
		  TIM2->CCR2 = command.speed2;
		  TIM2->CCR3 = command.speed3;
//	      feedback.speed1 = motor_speed_send.speed_send_M1;
//	      feedback.speed2 = motor_speed_send.speed_send_M2;
//	      feedback.speed3 = motor_speed_send.speed_send_M3;

	      CDC_Transmit_FS(&feedback, sizeof(feedback));
	    }
	    out_put_sleep();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim6.Init.Prescaler = 24;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 63999;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 49;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 63999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_M2_Pin|DIR_M1_Pin|DIR_M3_Pin|Debug_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_M2_Pin DIR_M1_Pin DIR_M3_Pin Debug_LED_Pin */
  GPIO_InitStruct.Pin = DIR_M2_Pin|DIR_M1_Pin|DIR_M3_Pin|Debug_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
