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
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BASE_INTERRUPT_PERIOD_MICROS 25
#define FASTEST_PERIOD_MICROS 100  // Fastest pulse period = 250 microseconds
#define HOMING_PULSE_LIMIT  50000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float angle1, angle2, angle3 = 0.0, error = 0;
int angle11, angle21, angle31 = 0;
int HOME = 0;

// Define counters for each motor
int countPulseL1 = 0;
int countPulseL2 = 0;
int countPulseL3 = 0;

int8_t a,b,c = 0;
float d = 0;
float e = 0, f = 0,g = 0,h =0,i=0,p=0,m=0,z=0;
int dir1, dir2, dir3 = 1;
float a3Ratio = 0;
int count = 0;
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

char tx_data[30] = "TRIET DEPTRAI \n";
char rx_data[21];  // Buffer for incoming data (adjust size as needed)
float rx_angle1, rx_angle2, rx_angle3 = 0;
int receiveComplete;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void pulseCreater(void);
void angleControl(float target_angle_1, float target_angle_2, float target_angle_3);
void update_current_angle(int motor_index, float steps,int direction);
void checkLimitSwitches(void) ;
void setHomeAngles();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	//------------
	if (htim->Instance==htim2.Instance)
	{

		if (HOME == 1){
			g+=1;

			if (g > 47000){
				g = 0;
			}

			if(1){
				target_angle_1 = rx_angle1;
				target_angle_2 = rx_angle2;
				target_angle_3 = rx_angle3;
			}

			if ((target_angle_1 != target_angle_1_temp)||(target_angle_2 != target_angle_2_temp)||(target_angle_3 != target_angle_3_temp)){

				target_angle_1_temp = target_angle_1;
				target_angle_2_temp = target_angle_2;
				target_angle_3_temp = target_angle_3;
				countPulseL1 = 0;
				countPulseL2 = 0;
				countPulseL3 = 0;

				angleControl(target_angle_1,target_angle_2,target_angle_3);

			}
			//angle3 = - angle2+ angle3;
			angleControl(target_angle_1,target_angle_2,target_angle_3);

		}
	}

	if (htim->Instance == htim3.Instance)
	{  // Ensure the correct timer is being checked
		// Handle Motor 1
		if (countPulseL1 > 0) {
			softCounter1++;  // Increment soft counter for motor 1
			if (softCounter1 >= periodMotor1) {  // If soft counter reaches the desired period
				HAL_GPIO_TogglePin(GPIOA, PUL1_Pin);  // Toggle Motor 1 pin (generate pulse)
				softCounter1 = 0;  // Reset the soft counter for motor 1
				countPulseL1--;    // Decrement the pulse count
				//e = 360/(44800*2);
				update_current_angle(1,1.0,dir1);
			}
		} else {
			HAL_GPIO_WritePin(GPIOA, PUL1_Pin, GPIO_PIN_RESET);  // Stop Motor 1
		}

		// Handle Motor 2
		if (countPulseL2 > 0) {
			softCounter2++;  // Increment soft counter for motor 2
			if (softCounter2 >= periodMotor2) {  // If soft counter reaches the desired period
				HAL_GPIO_TogglePin(GPIOA, PUL2_Pin);  // Toggle Motor 2 pin (generate pulse)
				softCounter2 = 0;  // Reset the soft counter for motor 2
				countPulseL2--;    // Decrement the pulse count
				update_current_angle(2,1.0,dir2);

			}
		} else {
			HAL_GPIO_WritePin(GPIOA, PUL2_Pin, GPIO_PIN_RESET);  // Stop Motor 2
		}

		// Handle Motor 3
		if (countPulseL3 > 0) {
			softCounter3++;  // Increment soft counter for motor 3
			if (softCounter3 >= periodMotor3) {  // If soft counter reaches the desired period
				HAL_GPIO_TogglePin(GPIOA, PUL3_Pin);  // Toggle Motor 3 pin (generate pulse)
				softCounter3 = 0;  // Reset the soft counter for motor 3
				countPulseL3--;    // Decrement the pulse count
				update_current_angle(3,1.0, dir3);

			}
		} else {
			HAL_GPIO_WritePin(GPIOA, PUL3_Pin, GPIO_PIN_RESET);  // Stop Motor 3
		}
	}
}
//----------------------Pulse Creater--------------------//
int char_to_int(char c)
{
	return (int)(c) - 48;
}

void Mag_Control(int state)
{
	HAL_GPIO_WritePin(GPIOA, RELAY_Pin, state);
}

void update_current_angle(int motor_index, float steps, int direction) {
    float angle_change = steps * 360.0 / (43840.0 * 2.0); // Calculate the change in angle

    // If direction is negative, reverse the angle change
    if (direction < 0) {
        angle_change = -angle_change;
    }

    switch (motor_index){
        case 1:
            angle1 += angle_change; // Update current angle for motor 1
            break;
        case 2:
            angle2 += angle_change; // Update current angle for motor 2
            angle3 -= angle_change;
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
    float temp;

    temp = target_angle - current_angle;
    if (temp > 0){
    	i = 1;
    }else{
    	i = -1;
    }
	if (fabs(temp) == 0){
		angular_difference = 0;
	}else{
		angular_difference = (target_angle) - current_angle;
	}


    // Convert angular difference to steps
	int steps_needed = (((angular_difference) / 360.0) * 43840.0 * 2.0);
	d = (int)round((0.98 / 360.0) * 43840 * 2.0);
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
	//f = steps_motor_2;
	int steps_motor_3 = calculate_steps(angle3, target_angle_3);

	// Set the pulse counts for each motor
	countPulseL1 = abs(steps_motor_1);
	countPulseL2 = abs(steps_motor_2) ;
	error = angle3 - target_angle_3;

	//countPulseL3 =abs(abs(steps_motor_3) + countPulseL2);
	//steps_motor_3 = countPulseL3;
	//steps_motor_3 = countPulseL3;





	// Determine the direction for each motor and set control pins accordingly
	if (target_angle_1 > angle1) {
		// Move motor 1 clockwise
		dir1 = 1;
		HAL_GPIO_WritePin(GPIOA, DIR1_Pin, GPIO_PIN_SET);

	} else {
		dir1 = -1;
		// Move motor 1 counterclockwise
		HAL_GPIO_WritePin(GPIOA, DIR1_Pin, GPIO_PIN_RESET);
	}

	if (target_angle_2 >= angle2) {
		// Move motor 2 clockwise
		dir2 = 1;
		HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_SET);
	} else {
		// Move motor 2 counterclockwise
		dir2 = -1;
		HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_RESET);
	}

	if (steps_motor_2 == 0){
		dir2 =1;
	}else if (steps_motor_1 == 0){
		dir3 = -1;
	}

	if (target_angle_3 > angle3 && dir2 == 1) {
		// Motor 3 moves clockwise when Motor 2 is moving clockwise
		h = 1;
		dir3 = 1;
		f = steps_motor_3;
		countPulseL3 = abs(steps_motor_3);
		steps_motor_3 = countPulseL3;
		HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_RESET);
	}
	else if (target_angle_3 < angle3 && dir2 == 1) {
		// Motor 3 moves counterclockwise when Motor 2 is moving counterclockwise
		h = 2;
		dir3 = -1;
		countPulseL3 =abs(steps_motor_3);
		steps_motor_3 = countPulseL3;
		HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_SET);
	} else if (target_angle_3 < angle3 && dir2 == -1) {
		// Motor 3 moves counterclockwise when Motor 2 is moving counterclockwise
		h = 3;
		dir3 = -1;
		countPulseL3 =abs(steps_motor_3);
		steps_motor_3 = countPulseL3;
		HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_SET);
	}
	else if (target_angle_3 > angle3 && dir2 == -1) {
		// Motor 3 moves counterclockwise when Motor 2 is moving counterclockwise
		h = 4;
		dir3 = 1;
		f = steps_motor_3;
		countPulseL3 = abs( steps_motor_3);
		steps_motor_3 = countPulseL3;
		HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_RESET);
	} else {
		// Maintain angle3 in sync with angle2’s direction
		h =5;
		dir3 = dir2;
		HAL_GPIO_WritePin(GPIOA, DIR3_Pin, dir3 == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}


	// Determine the maximum steps needed to synchronize the motors
	int max_steps = maxSteps(abs(steps_motor_1), abs(steps_motor_2), abs(steps_motor_3));

	// Calculate the pulse periods for each motor based on the maximum steps
	periodMotor1 = (int)(((float)max_steps / abs(steps_motor_1)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS)) + 2;
	periodMotor2 = (int)(((float)max_steps / abs(steps_motor_2)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS)) + 2;
	periodMotor3 = (int)(((float)max_steps / abs(steps_motor_3)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS)) + 2;

	// Start the timer if it is not already running (ensure the timer has been initialized)
	//HAL_TIM_Base_Start_IT(&htim4);
}

void autoHomeMotors(void) {
    // Set initial movement direction for homing (assumes motors move towards the limit switches)
    HAL_GPIO_WritePin(GPIOA, DIR1_Pin, GPIO_PIN_RESET); // Move Motor 1 in the homing direction
    HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_SET); // Move Motor 2 in the homing direction
    HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_SET); // Move Motor 3 in the homing direction


    // Start moving the motors
    countPulseL1 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing
    countPulseL2 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing
    countPulseL3 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing

    periodMotor1 = 8;
	periodMotor2 = 8;
	periodMotor3 = 16;
	int i = 0;

    // Wait until all limit switches are triggered
    while (a == 0 || b == 0 || c == 0 || count <1) {
        checkLimitSwitches(); // Continuously check the limit switches
    }
    while (count == 1){
    	HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_RESET);

    	if (countPulseL3 == 0 && i == 0){
    		countPulseL3 = 10060;
    		i = 1;
    	}

    	if (countPulseL3 == 0 && i == 1){
			count = 2;
			a = 0;
			b=0;
			c=0;
			countPulseL3 = 50000;
		}
    }

    while (a == 0 || b == 0 || c == 0 ) {
		checkLimitSwitches(); // Continuously check the limit switches
	}

    // Set the home angle once all limit switches are activated
    HOME = 1;
    setHomeAngles();
}

void setHomeAngles(void) {
    // Set the current angle to zero or a specific home angle as needed
    angle1 = 5.0; // Home position for Motor 1
    angle2 = 86.0; // Home position for Motor 2
    angle3 = -151.0; // Home position for Motor 3

}

void checkLimitSwitches(void) {
    // Check and handle limit switch 1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) {
        a = 1;
        countPulseL1 = 0;  // Stop Motor 1 if limit switch 1 is triggered
    } else {
        a = 0;
    }

    // Check and handle limit switch 2, ensuring that it only works if limit switch 3 is not active
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) {
        b = 1;
        countPulseL2 = 0;  // Stop Motor 2 if limit switch 2 is triggered
    } else {
        b = 0;
    }


    if (a == 1 && b == 1 && c == 1 && count != 1){
      count = 1;

	}

    // Check and handle limit switch 3
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
        c = 1;

		periodMotor3 = periodMotor2;
		countPulseL3 = countPulseL2;  // Stop Motor 3 if limit switch 3 is triggered

        HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_RESET);  // Stop motor direction for M3
    } else {
        c = 0;
        HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_SET);  // Set motor direction for M3
    }

}
void decode_angles(const char *input, float *angle1, float *angle2, float *angle3) {
	// Decode angle1
	    int sign1 = (input[0] == '1') ? 1 : -1;  // Check sign at position 0
	    *angle1 = sign1 * ((input[1] - '0') * 100 + (input[2] - '0') * 10 + (input[3] - '0')
	                      + (input[4] - '0') * 0.1 + (input[5] - '0') * 0.01);

	    // Decode angle2
	    int sign2 = (input[7] == '1') ? 1 : -1;  // Check sign at position 7
	    *angle2 = sign2 * ((input[8] - '0') * 100 + (input[9] - '0') * 10 + (input[10] - '0')
	                      + (input[11] - '0') * 0.1 + (input[12] - '0') * 0.01);

	    // Decode angle3
	    int sign3 = (input[14] == '1') ? 1 : -1;  // Check sign at position 14
	    *angle3 = sign3 * ((input[15] - '0') * 100 + (input[16] - '0') * 10 + (input[17] - '0')
	                      + (input[18] - '0') * 0.1 + (input[19] - '0') * 0.01);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
  if (huart->Instance == huart3.Instance){
	  // Ensure received data is null-terminated

	  //rx_data[29] = '\0';

	  // Ensure the received data is null-terminated



	  // Set the flag indicating data is ready for use
	  receiveComplete = 1;

	  // Restart reception for the next incoming data
	  HAL_UART_Receive_IT(&huart3, rx_data, 21);
	  decode_angles(rx_data, &rx_angle1, &rx_angle2, &rx_angle3);

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Transmit(&huart3, (uint8_t *)tx_data, sizeof(tx_data), 10);
  HAL_UART_Receive_IT(&huart3, (uint8_t *)rx_data, 21);
  autoHomeMotors();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PUL3_Pin|DIR3_Pin|PUL2_Pin|DIR2_Pin
                          |PUL1_Pin|DIR1_Pin|EN_Pin|RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PUL3_Pin DIR3_Pin PUL2_Pin DIR2_Pin
                           PUL1_Pin DIR1_Pin EN_Pin RELAY_Pin */
  GPIO_InitStruct.Pin = PUL3_Pin|DIR3_Pin|PUL2_Pin|DIR2_Pin
                          |PUL1_Pin|DIR1_Pin|EN_Pin|RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_LINK3_Pin LIMIT_LINK1_Pin LIMIT_LINK2_Pin */
  GPIO_InitStruct.Pin = LIMIT_LINK3_Pin|LIMIT_LINK1_Pin|LIMIT_LINK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
