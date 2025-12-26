/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h> // thêm dòng này vào đầu file main.c

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stm32h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PWA TIM_CHANNEL_1
#define PWB TIM_CHANNEL_2
#define BIN1 GPIO_PIN_4
#define BIN2 GPIO_PIN_5
#define AIN1 GPIO_PIN_6
#define AIN2 GPIO_PIN_7
#define STB GPIO_PIN_8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint32_t _millis = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t sensor[7];
// PID config
float Kp = 140;	// 130	//90	// 130
float Ki = 0;
float Kd = 30;	//30	//30

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int16_t initial_motor_speed = 380;	// 500	// 375
int16_t left_motor_speed, right_motor_speed;

uint8_t stop_flag;
volatile uint8_t count = 0;
uint8_t pre_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//void SysTick_Handler(void) {
//    _millis++;
//}
void millis_init(void) {
	// Reload = (HCLK / 1000) - 1  -> mỗi 1ms
	// Nếu HCLK = 250 MHz -> Reload = 250000 - 1
	uint32_t reload = (SystemCoreClock / 1000) - 1;

	SysTick->LOAD = reload;                // Giá trị reload
	SysTick->VAL = 0;                     // Reset counter
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | // Nguồn clock = HCLK
			SysTick_CTRL_TICKINT_Msk | // Enable ngắt
			SysTick_CTRL_ENABLE_Msk;     // Bật SysTick
}

uint32_t millis(void) {
	return _millis;
}

void sensor_read() {
	sensor[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	sensor[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	sensor[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	sensor[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	sensor[4] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	sensor[5] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	sensor[6] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	//stop_flag = HAL_GPIO_ReadPin(GPIOC,  GPIO_PIN_15);
}


void calculate_error() {
	sensor_read();
// white line

	if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0)
				&& (sensor[3] == 0) && sensor[4] == 1 && sensor[5] == 1
				&& sensor[6] == 1)
			error = -0.15;	//-0.2

		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
				&& (sensor[3] == 0) && sensor[4] == 1 && sensor[5] == 1
				&& sensor[6] == 1)  // go straight
			error = 0;

		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
				&& (sensor[3] == 0) && sensor[4] == 0 && sensor[5] == 1
				&& sensor[6] == 1)
			error = 0.15;
		else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
			&& (sensor[6] == 1))
		error = -5;		// -5

		else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1)
				&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
				&& (sensor[6] == 1))
			error = -4;		//-4

		else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1)
				&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
				&& (sensor[6] == 1))
			error = -3;		//-3

		else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0)
				&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
				&& (sensor[6] == 1))
			error = -1.2;		//-1.5

		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0)
				&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
				&& (sensor[6] == 1))
			error = -0.8;		//-0.8


		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
				&& (sensor[3] == 1) && sensor[4] == 0 && sensor[5] == 1
				&& sensor[6] == 1)
			error = 0.8;

		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
				&& (sensor[3] == 1) && sensor[4] == 0 && sensor[5] == 0
				&& sensor[6] == 1)
			error = 1.2;

		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
				&& (sensor[3] == 1) && sensor[4] == 1 && sensor[5] == 0
				&& sensor[6] == 1)
			error = 3;

		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
				&& (sensor[3] == 1) && sensor[4] == 1 && sensor[5] == 0
				&& sensor[6] == 0)
			error = 4;

		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
				&& (sensor[3] == 1) && sensor[4] == 1 && sensor[5] == 1
				&& sensor[6] == 0)
			error = 5;

/*
	if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
			&& (sensor[6] == 1))
		error = -5;		// -5

	else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1)
			&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
			&& (sensor[6] == 1))
		error = -4;		//-4

	else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1)
			&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
			&& (sensor[6] == 1))
		error = -3;		//-3

	else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0)
			&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
			&& (sensor[6] == 1))
		error = -1.2;		//-1.5

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0)
			&& (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 1)
			&& (sensor[6] == 1))
		error = -0.8;		//-0.8

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0)
			&& (sensor[3] == 0) && sensor[4] == 1 && sensor[5] == 1
			&& sensor[6] == 1)
		error = -0.15;	//-0.2

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 0) && sensor[4] == 1 && sensor[5] == 1
			&& sensor[6] == 1)  // go straight
		error = 0;

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 0) && sensor[4] == 0 && sensor[5] == 1
			&& sensor[6] == 1)
		error = 0.15;

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 1) && sensor[4] == 0 && sensor[5] == 1
			&& sensor[6] == 1)
		error = 0.8;

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 1) && sensor[4] == 0 && sensor[5] == 0
			&& sensor[6] == 1)
		error = 1.2;

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 1) && sensor[4] == 1 && sensor[5] == 0
			&& sensor[6] == 1)
		error = 3;

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 1) && sensor[4] == 1 && sensor[5] == 0
			&& sensor[6] == 0)
		error = 4;

	else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1)
			&& (sensor[3] == 1) && sensor[4] == 1 && sensor[5] == 1
			&& sensor[6] == 0)
		error = 5;

		*/
	/*
	 // black line
	 if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = -3;

	 else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = -2.5;

	 else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = -2;

	 else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = -1.5;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = -1;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = -0.5;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = 0;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = 0.5;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 0) && (sensor[6] == 0))
	 error = 1;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1) && (sensor[5] == 1) && (sensor[6] == 0))
	 error = 1.5;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 1) && (sensor[6] == 0))
	 error = 2;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 1) && (sensor[6] == 1))
	 error = 2.5;

	 else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) && (sensor[5] == 0) && (sensor[6] == 1))
	 error = 3;
	 */
}

void forward() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
}

void set_speed(int16_t speed1, int16_t speed2) {

	if (speed1 < 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	}
	if (speed2 < 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(speed1));
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(speed2));
	/*
	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, abs(speed1));
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(speed2));
	 */

}

int constrain(int x, int a, int b) {
	if (x < a)
		return a;
	else if (x > b)
		return b;
	else
		return x;
}

void calculate_pid() {
	P = error;
	I = I + error;
	D = error - previous_error;

	PID_value = (Kp * P) + (Ki * I) + (Kd * D);

	previous_I = I;
	previous_error = error;
}

void motor_control() {
	// Calculating the effective motor speed:

	left_motor_speed = initial_motor_speed + PID_value;   //200
	right_motor_speed = initial_motor_speed - PID_value;  //0

	left_motor_speed = constrain(left_motor_speed, -300, 999);    //0
	right_motor_speed = constrain(right_motor_speed, -300, 999);  //210

	set_speed(left_motor_speed, right_motor_speed);

	//forward();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	UNUSED(GPIO_Pin);

	count++;
//	if (count == 14) {
//		Kp = 150;	// 130	//90	// 130
//		Ki = 0;
//		Kd = 30;
//		initial_motor_speed = 380;
//	}

	if (count == 14) {
		set_speed(500, 500);
		forward();
		//HAL_Delay(300);
		for (int i = 0; i < 1000000; i++)
			;
		set_speed(0, 0);

		while (1)
			;
	}

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM1_Init();
	MX_TIM2_Init();
	millis_init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);	// enable STBY
	//int i=0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);	// enable STBY
	while (1) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0)
		{
			Kp = 160;	// 130	//90	// 130	//175
			Ki = 0;
			Kd = 110;		//70	//100
			initial_motor_speed = 600;	//600
			break;
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 0)
		{
			Kp = 175;	// 130	//90	// 130	//175
			Ki = 0;
			Kd = 115;		//70	//100
			initial_motor_speed = 650;	//600
			break;
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0)
		{
			Kp = 180;	// 130	//90	// 130	//175
			Ki = 0;
			Kd = 120;		//70	//100
			initial_motor_speed = 700;	//600
			break;
		}

	}
	HAL_Delay(1000);
	set_speed(400, 400);
	forward();
	HAL_Delay(200);
	while (1) {
		calculate_error();
		calculate_pid();
		motor_control();

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
	RCC_OscInitStruct.CSIState = RCC_CSI_ON;
	RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 125;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the programming delay
	 */
	__HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 79;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 499;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 79;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 199;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA2 PA3
	 PA4 PA5 PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
			| GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB4 PB5 PB6 PB7
	 PB8 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
			| GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
