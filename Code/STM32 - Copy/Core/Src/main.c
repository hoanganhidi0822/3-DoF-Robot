/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "as5600.h"
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BASE_INTERRUPT_PERIOD_MICROS 50
#define FASTEST_PERIOD_MICROS 200  // Fastest pulse period = 250 microseconds
#define HOMING_PULSE_LIMIT  50000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
AS5600_TypeDef *angleL1;
AS5600_TypeDef *angleL2;
AS5600_TypeDef *angleL3;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
float angle1, angle2, angle3 = 0;
int angle11, angle21, angle31 = 0;
int HOME = 0;

// Define counters for each motor
int countPulseL1 = 0;
int countPulseL2 = 0;
int countPulseL3 = 0;


int a,b,c = 0;
float d = 0;
float e = 0, f = 0,g = 0;
int dir1, dir2, dir3 = 1;
float a3Ratio = 0;
// Soft counters and periods
int softCounter1 = 0;
int softCounter2 = 0;
int softCounter3 = 0;

int periodMotor1 = 0;
int periodMotor2 = 0;
int periodMotor3 = 0;

float target_angle_1 = 0;
float target_angle_2 = 0;
float target_angle_3 = 0;
float target_angle_1_temp = 1;
float target_angle_2_temp = 1;
float target_angle_3_temp = 1;

int steps_motor_1 = 0;
int steps_motor_2 = 0;
int steps_motor_3 = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void pulseCreater(void);
void angleControl(float target_angle_1, float target_angle_2, float target_angle_3);
void update_current_angle(int motor_index, float steps,int direction);
void checkLimitSwitches(void) ;
void setHomeAngles();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//----------------------Ngat Sensor----------------------//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	//------------
	if (htim->Instance==htim2.Instance)
	{


		if (HOME == 1){
			g+=1;

		if(g > 21000){
			target_angle_1 = -45;
			target_angle_2 = -45;
			target_angle_3 = 0;
		}else if(g > 16000){
			target_angle_1 = 0;
			target_angle_2 = -45;
			target_angle_3 = -45;
		}
		else if(g > 12000){
			target_angle_1 = 0;
			target_angle_2 = -45;
			target_angle_3 = -45;
		}else if (g > 5000){
			target_angle_1 = 0;
			target_angle_2 = -45;
			target_angle_3 = 0;
		}else{
			target_angle_1 = 0;
			target_angle_2 = -45;
			target_angle_3 = -45;
		}





			if ((target_angle_1 != target_angle_1_temp)||(target_angle_2 != target_angle_2_temp)||(target_angle_3 != target_angle_3_temp)){

				target_angle_1_temp = target_angle_1;
				target_angle_2_temp = target_angle_2;
				target_angle_3_temp = target_angle_3;


				a3Ratio = fabs((target_angle_3 - angle3)/((target_angle_2 - angle2) + (target_angle_3 - angle3)));

				angleControl(target_angle_1,target_angle_2,target_angle_3);
			}
		}
	}

	if (htim->Instance == TIM4)
	{  // Ensure the correct timer is being checked
		// Handle Motor 1
		if (countPulseL1 > 0) {
			softCounter1++;  // Increment soft counter for motor 1
			if (softCounter1 >= periodMotor1) {  // If soft counter reaches the desired period
				HAL_GPIO_TogglePin(GPIOB, motorL1_Pin);  // Toggle Motor 1 pin (generate pulse)
				softCounter1 = 0;  // Reset the soft counter for motor 1
				countPulseL1--;    // Decrement the pulse count
				//e = 360/(44800*2);
				update_current_angle(1,1,dir1);
			}
		} else {
			HAL_GPIO_WritePin(GPIOB, motorL1_Pin, GPIO_PIN_RESET);  // Stop Motor 1
		}

		// Handle Motor 2
		if (countPulseL2 > 0) {
			softCounter2++;  // Increment soft counter for motor 2
			if (softCounter2 >= periodMotor2) {  // If soft counter reaches the desired period
				HAL_GPIO_TogglePin(GPIOB, motorL2_Pin);  // Toggle Motor 2 pin (generate pulse)
				softCounter2 = 0;  // Reset the soft counter for motor 2
				countPulseL2--;    // Decrement the pulse count
				update_current_angle(2,1,dir2);
			}
		} else {
			HAL_GPIO_WritePin(GPIOB, motorL2_Pin, GPIO_PIN_RESET);  // Stop Motor 2
		}

		// Handle Motor 3
		if (countPulseL3 > 0) {
			softCounter3++;  // Increment soft counter for motor 3
			if (softCounter3 >= periodMotor3) {  // If soft counter reaches the desired period
				HAL_GPIO_TogglePin(GPIOB, motorL3_Pin);  // Toggle Motor 3 pin (generate pulse)
				softCounter3 = 0;  // Reset the soft counter for motor 3
				countPulseL3--;    // Decrement the pulse count
				update_current_angle(3,a3Ratio, dir3);

			}
		} else {
			HAL_GPIO_WritePin(GPIOB, motorL3_Pin, GPIO_PIN_RESET);  // Stop Motor 3
		}
	}
}
//----------------------Pulse Creater--------------------//


void Mag_Control(int state)
{
	HAL_GPIO_WritePin(Mag_GPIO_Port, Mag_Pin, state);
}

void Read_Angles(void)
{
    // Read raw angle from angleL1 sensor
    AS5600_GetRawAngle(angleL1, &angle1);

    // Read raw angle from angleL2 sensor
    AS5600_GetRawAngle(angleL2, &angle2);

    // Read raw angle from angleL3 sensor
    AS5600_GetRawAngle(angleL3, &angle3);

}

void update_current_angle(int motor_index, float steps, int direction) {
    float angle_change = steps * 360.0 / (44800.0 * 2); // Calculate the change in angle

    // If direction is negative, reverse the angle change
    if (direction < 0) {
        angle_change = - angle_change;
    }

    switch (motor_index){
        case 1:
            angle1 += angle_change; // Update current angle for motor 1
            break;
        case 2:
            angle2 += angle_change; // Update current angle for motor 2
            break;
        case 3:
            angle3 += angle_change; // Update current angle for motor 3
            break;
    }
}

int calculate_steps(float current_angle, float target_angle)
{
    // Calculate the angular difference

    float angular_difference ;
    int temp;

    temp = target_angle - current_angle;
	if (abs(temp) < 0.01){
		angular_difference = 0;
	}else{
		angular_difference = target_angle - current_angle;
	}


    // Convert angular difference to steps
    int steps_needed = (int)((angular_difference / 360) * 44800*2);

    // Return the absolute number of steps (stepper motors can't move negative steps)
    return steps_needed;
}

int maxSteps(int steps_motor_1, int steps_motor_2, int steps_motor_3)
{
    int max_value = steps_motor_1;  // Assume motor 1 has the maximum steps

    if (steps_motor_2 > max_value) {
        max_value = steps_motor_2;  // Update if motor 2 has more steps
    }

    if (steps_motor_3 > max_value) {
        max_value = steps_motor_3;  // Update if motor 3 has more steps
    }

    return max_value;  // Return the largest number of steps
}


void angleControl(float target_angle_1, float target_angle_2, float target_angle_3) {
	// Read the current angles from the sensors
	//Read_Angles();

	// Calculate the required steps for each motor based on current and target angles
	int steps_motor_1 = calculate_steps(angle1, (target_angle_1));
	int steps_motor_2 = calculate_steps(angle2, (target_angle_2));

	int steps_motor_3 = calculate_steps(angle3, target_angle_3 + target_angle_2);

	// Set the pulse counts for each motor
	countPulseL1 = abs(steps_motor_1);
	countPulseL2 = abs(steps_motor_2);
	countPulseL3 = abs(steps_motor_3);


	// Determine the direction for each motor and set control pins accordingly
	if (target_angle_1 > angle1) {
		// Move motor 1 clockwise
		dir1 = 1;
		HAL_GPIO_WritePin(GPIOF, M1dir_Pin, GPIO_PIN_SET);

	} else {
		dir1 = -1;
		// Move motor 1 counterclockwise
		HAL_GPIO_WritePin(GPIOF, M1dir_Pin, GPIO_PIN_RESET);
	}

	if (target_angle_2 > angle2) {
		// Move motor 2 clockwise
		dir2 = 1;
		HAL_GPIO_WritePin(GPIOF, M2dir_Pin, GPIO_PIN_SET);
	} else {
		// Move motor 2 counterclockwise
		dir2 = -1;
		HAL_GPIO_WritePin(GPIOF, M2dir_Pin, GPIO_PIN_RESET);
	}

	if (target_angle_3 > angle3) {
		// Move motor 3 clockwise
		dir3 = 1;
		//HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_RESET);
	} else {
		// Move motor 3 counterclockwise
		dir3 = -1;
		//HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_SET);
	}

	if (dir3 != dir2){
		if (abs(target_angle_3  - angle3) <= abs(target_angle_2  - angle2))
		{
			HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_RESET);
			//countPulseL3 = abs(steps_motor_3) + countPulseL2;
		}
	}else{

		if (dir2 < 0){
			countPulseL3 = abs(steps_motor_3);
			steps_motor_3 = countPulseL3;
			HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_SET);

		}else{
			countPulseL3 = abs(steps_motor_3 + steps_motor_2);
			steps_motor_3 = countPulseL3;
			HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_RESET);
		}

	}


	// Determine the maximum steps needed to synchronize the motors
	int max_steps = maxSteps(abs(steps_motor_1), abs(steps_motor_2), abs(steps_motor_3));

	// Calculate the pulse periods for each motor based on the maximum steps
	periodMotor1 = (int)(((float)max_steps / abs(steps_motor_1)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS));
	periodMotor2 = (int)(((float)max_steps / abs(steps_motor_2)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS));
	periodMotor3 = (int)(((float)max_steps / abs(steps_motor_3)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS));

	// Start the timer if it is not already running (ensure the timer has been initialized)
	//HAL_TIM_Base_Start_IT(&htim4);
}

void autoHomeMotors(void) {
    // Set initial movement direction for homing (assumes motors move towards the limit switches)
    HAL_GPIO_WritePin(GPIOF, M1dir_Pin, GPIO_PIN_RESET); // Move Motor 1 in the homing direction
    HAL_GPIO_WritePin(GPIOF, M2dir_Pin, GPIO_PIN_SET); // Move Motor 2 in the homing direction
    HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_SET); // Move Motor 3 in the homing direction


    // Start moving the motors
    countPulseL1 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing
    countPulseL2 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing
    countPulseL3 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing

    periodMotor1 = 4;
	periodMotor2 = 4;
	periodMotor3 = 4;

    // Wait until all limit switches are triggered
    while (a == 0 || b == 0 || c == 0) {
        checkLimitSwitches(); // Continuously check the limit switches
    }

    // Set the home angle once all limit switches are activated
    HOME = 1;
    setHomeAngles();
}

void setHomeAngles(void) {
    // Set the current angle to zero or a specific home angle as needed
    angle1 = 0.0; // Home position for Motor 1
    angle2 = 0.0; // Home position for Motor 2
    angle3 = -68.0; // Home position for Motor 3

}

void checkLimitSwitches(void) {
    // Check and handle limit switch 1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
        a = 1;
        countPulseL1 = 0;  // Stop Motor 1 if limit switch 1 is triggered
    } else {
        a = 0;
    }

    // Check and handle limit switch 2, ensuring that it only works if limit switch 3 is not active
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
        b = 1;
        countPulseL2 = 0;  // Stop Motor 2 if limit switch 2 is triggered
    } else {
        b = 0;
    }

    // Check and handle limit switch 3
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
        c = 1;
        countPulseL3 = countPulseL2;  // Stop Motor 3 if limit switch 3 is triggered
        HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_RESET);  // Stop motor direction for M3
    } else {
        c = 0;
        HAL_GPIO_WritePin(GPIOF, M3dir_Pin, GPIO_PIN_SET);  // Set motor direction for M3
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
//	angleL1 = AS5600_New();
//	angleL1->i2cHandle= &hi2c1;
//	AS5600_Init(angleL1);
//
//	angleL2 = AS5600_New();
//	angleL2->i2cHandle= &hi2c2;
//	AS5600_Init(angleL2);
//
//	angleL3 = AS5600_New();
//	angleL3->i2cHandle= &hi2c3;
//	AS5600_Init(angleL3);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  autoHomeMotors();


//
//


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //calc_angle_3();
	  //angleControl(angle1,angle2, angle3);

	    // Stop Motor 3
	  //checkLimitSwitches();
	 // HAL_GPIO_WritePin(GPIOB, motorL3_Pin, GPIO_PIN_RESET);  // Stop Motor 3
//	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
//	  HAL_Delay(0.1);
//	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
//	  HAL_Delay(0.1);
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB1;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00A0A3F7;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00A0A3F7;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00A0A3F7;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 840;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Mag_GPIO_Port, Mag_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, motorL1_Pin|motorL2_Pin|motorL3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, M1dir_Pin|M2dir_Pin|M3dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : LimL1_Pin LimL2_Pin LimL3_Pin */
  GPIO_InitStruct.Pin = LimL1_Pin|LimL2_Pin|LimL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Mag_Pin */
  GPIO_InitStruct.Pin = Mag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Mag_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : motorL1_Pin motorL2_Pin */
  GPIO_InitStruct.Pin = motorL1_Pin|motorL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : motorL3_Pin */
  GPIO_InitStruct.Pin = motorL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(motorL3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1dir_Pin M2dir_Pin M3dir_Pin */
  GPIO_InitStruct.Pin = M1dir_Pin|M2dir_Pin|M3dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_CRS_SYNC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
