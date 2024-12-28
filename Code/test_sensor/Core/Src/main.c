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
typedef struct {
    float Kp;           // Hệ số P
    float Ki;           // Hệ số I
    float Kd;           // Hệ số D
    float prev_error;   // Lỗi ở lần trước
    float integral;     // Tích phân lỗi
} PIDController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BASE_INTERRUPT_PERIOD_MICROS 25
#define FASTEST_PERIOD_MICROS 150 // Fastest pulse period = 250 microseconds
#define HOMING_PULSE_LIMIT  50000

#define MAX_COUNT 65535 // Giá trị tối đa của bộ đếm (16-bit)
#define PULSES_PER_REV 8910 // Số xung mỗi vòng quay
#define HOME_M1 -84.5 // Số xung mỗi vòng quay
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

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

int dt = 0;
int delay = 0;

float angles[18];

float angle1p1 = 0.0;
float angle2p1 = 0.0;
float angle3p1 = 0.0;

float angle1p2 = 0.0;
float angle2p2 = 0.0;
float angle3p2 = 0.0;

float angle1p3 = 0.0;
float angle2p3 = 0.0;
float angle3p3 = 0.0;

float angle1p4 = 0.0;
float angle2p4 = 0.0;
float angle3p4 = 0.0;

float angle1p5 = 0.0;
float angle2p5 = 0.0;
float angle3p5 = 0.0;

float angle1pt = 0.0;
float angle2pt = 0.0;
float angle3pt = 0.0;

float angle1err = 0.0;
float angle2err = 0.0;
float angle3err = 0.0;

//const float alpha = 0.5; // Adjust alpha as needed (e.g., 0.1 for strong smoothing)
//float filtered_angle1 = 0.0; // Initial filtered value

// Soft counters and periods
int softCounter1 = 0;
int softCounter2 = 0;
int softCounter3 = 0;

int periodMotor1 = 0;
int periodMotor2 = 0;
int periodMotor3 = 0;

float target_angle_1 = 0.0;
float target_angle_2 = 70.0;
float target_angle_3 = -135.0;
float target_angle_1_temp = 1;
float target_angle_2_temp = 1;
float target_angle_3_temp = 1;

int steps_motor_1 = 0;
int steps_motor_2 = 0;
int steps_motor_3 = 0;

char tx_data[30] = "abcd";
//char rx_data[21];  // Buffer for incoming data (adjust size as needed)
uint8_t rx_data[127];


float rx_angle1 = 0.0, rx_angle2 = 70.0, rx_angle3 = -153.0;
int receiveComplete;

float setpoint = -25.0f;  // Góc đích (90 độ)
float current_angle = 0.0f;
float delta_time = 0.01f; // Chu kỳ vòng lặp (10 ms)
int m1speed = 0;


uint16_t duty =100, count = 0;

volatile int16_t prev_count = 0; // Giá trị bộ đếm trước đó
volatile int16_t delta = 0;      // Số xung thay đổi
volatile int32_t total_count = 0; // Tổng số xung (có thể âm)
volatile float angle = 0.0;      // Góc quay hiện tại
int home_count = 0;
int need_to_set_home = 0;
int prvstate = 0, curstate = 0;


PIDController pid;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void pulseCreater(void);
void angleControl(PIDController *pid, float target_angle_1, float target_angle_2, float target_angle_3);
void update_current_angle(int motor_index, float steps,int direction);
void checkLimitSwitches(void) ;
void setHomeAngles();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==htim2.Instance)
	{
		if ( HOME == 1){
			dt++;
		}
	}

	if (htim->Instance == htim3.Instance)
	{
		// Handle Motor 2
		if (countPulseL2 > 0) {
			softCounter2++;  // Increment soft counter for motor 2
			if (softCounter2 >= periodMotor2)
			{  // If soft counter reaches the desired period
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
			if (softCounter3 >= periodMotor3)
			{  // If soft counter reaches the desired period
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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	uint16_t counter = __HAL_TIM_GET_COUNTER(htim);
	count = (int16_t)counter;

	// Kiểm tra nếu cần đặt lại "home"
	if (need_to_set_home) {

		need_to_set_home = 0; // Đặt lại flag
	}
	delta = count - prev_count;

	// Xử lý trường hợp tràn
	if (delta > (MAX_COUNT / 2)) {
	    delta -= (MAX_COUNT + 1); // Tràn ngược
	} else if (delta < -(MAX_COUNT / 2)) {
	    delta += (MAX_COUNT + 1); // Tràn thuận
	}

	// Cập nhật tổng số xung
	total_count += delta;
	// Tính góc quay (đếm cả cạnh lên và xuống, chia 2 để ra số xung thực tế)
	angle1 = (float)total_count * 360.0f / PULSES_PER_REV/4.0;
	// Cập nhật giá trị bộ đếm trước đó
	prev_count = count;
	// Đảm bảo góc quay trong khoảng [-180, 180)
	if (angle1 < -180.0f) {
	    angle1 += 360.0f;
	} else if (angle1 >= 180.0f) {
	    angle1 -= 360.0f;
	}
}

// Hàm khởi tạo PID
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

// Hàm tính toán PID
float PID_Compute(PIDController *pid, float setpoint, float current_angle, float delta_time) {
    // Tính toán lỗi
    float error = setpoint - current_angle;
    float Ki = 0;
    if (fabs(error) > 5.0){
    	Ki = pid->Ki/100.0;
    }else{
    	Ki = pid->Ki;
    }

    // Đảm bảo lỗi nằm trong khoảng [-180, 180] (xử lý vòng tròn 360 độ)
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    // Tích phân lỗi (chỉ tích lũy khi delta_time > 0)
    if (delta_time > 0) {
        pid->integral += error * delta_time;
    }

    // Đạo hàm lỗi (chỉ tính khi delta_time > 0)
    float derivative = 0.0f;
    if (delta_time > 0) {
        derivative = (error - pid->prev_error) / delta_time;
    }

    // Tính tín hiệu điều khiển
    float output = pid->Kp * error + Ki * pid->integral + pid->Kd * derivative;

    // Lưu lỗi hiện tại cho lần tính sau
    pid->prev_error = error;

    if (output > 115.0f) {
		output = 115.0f;
	} else if (output < -115.0f) {
		output =-115.0f;
	}

    return output;
}

void Motor1_SetSpeed(float output) {
    if (output > 0) {
        // Quay thuận
    	HAL_GPIO_WritePin(GPIOA, DIR1A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, DIR1B_Pin, GPIO_PIN_SET);
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,output);
    } else {
        // Quay ngược
    	HAL_GPIO_WritePin(GPIOA, DIR1A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, DIR1B_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,fabs(output));
    }
}

void update_current_angle(int motor_index, float steps, int direction) {
    float angle_change = steps * 360.0 / (43840.0 * 2.0); // Calculate the change in angle

    // If direction is negative, reverse the angle change
    if (direction < 0) {
        angle_change = -angle_change;
    }
    switch (motor_index){

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
    // Return the absolute number of steps (stepper motors can't move negative steps)
    return steps_needed;
}

int maxSteps(int steps_motor_2, int steps_motor_3)
{
    int max_value = steps_motor_2;  // Assume motor 1 has the maximum steps

    if (steps_motor_2 > max_value) {
        max_value = steps_motor_2;  // Update if motor 2 has more steps
    }

    if (steps_motor_3 > max_value) {
        max_value = steps_motor_3;  // Update if motor 3 has more steps
    }

    return max_value;  // Return the largest number of steps
}

void angleControl(PIDController *pid, float target_angle_1, float target_angle_2, float target_angle_3) {
	// Calculate the required steps for each motor based on current and target angles
	int steps_motor_2 = calculate_steps(angle2, target_angle_2);
	int steps_motor_3 = calculate_steps(angle3, target_angle_3);

	// Set the pulse counts for each motor
	countPulseL2 = abs(steps_motor_2) ;
	error = angle3 - target_angle_3;

	float output = PID_Compute(pid, target_angle_1, angle1, 0.01);
	Motor1_SetSpeed(output);

	if (target_angle_2 >= angle2) {
		dir2 = 1;
		HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_SET);
	} else {
		dir2 = -1;
		HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_RESET);
	}

	if (steps_motor_2 == 0){
		dir2 =1;
	}else if (steps_motor_1 == 0){
		dir3 = -1;
	}

	if (target_angle_3 > angle3 && dir2 == 1) {
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
	int max_steps = maxSteps(abs(steps_motor_2), abs(steps_motor_3));

	// Calculate the pulse periods for each motor based on the maximum steps
	periodMotor2 = (int)(((float)max_steps / abs(steps_motor_2)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS)) + 2;
	periodMotor3 = (int)(((float)max_steps / abs(steps_motor_3)) * (FASTEST_PERIOD_MICROS / BASE_INTERRUPT_PERIOD_MICROS)) + 2;

}

void autoHomeMotors(void) {
    // Set initial movement direction for homing (assumes motors move towards the limit switches)
    HAL_GPIO_WritePin(GPIOA, DIR2_Pin, GPIO_PIN_SET); // Move Motor 2 in the homing direction
    HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_SET); // Move Motor 3 in the homing direction

    // Start moving the motors
    m1speed = -20;
    countPulseL2 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing
    countPulseL3 = HOMING_PULSE_LIMIT;  // Arbitrary large value for homing

	periodMotor2 = 10;
	periodMotor3 = 10;
	int i = 0;

    // Wait until all limit switches are triggered
    while (a == 0 || b == 0 || c == 0 || home_count < 1)
    {
        checkLimitSwitches(); // Continuously check the limit switches
    }
    while (home_count == 1){
    	HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_RESET);

    	if (countPulseL3 == 0 && i == 0){
    		countPulseL3 = 5000;
    		i = 1;
    	}

    	if (countPulseL3 == 0 && i == 1){
			home_count = 2;
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

    setHomeAngles();
    while (fabs(angle1) > 1.0){
		angleControl(&pid, 	0.0, 87.0, -149.0);
		HAL_Delay(0.01);
	}
	HOME = 1;
}

void setHomeAngles(void) {
    // Set the current angle to zero or a specific home angle as needed
	need_to_set_home = 1;
	TIM4->CNT = (int16_t)((HOME_M1 * 2.0 * PULSES_PER_REV * 2.0f) / 360.0f);
	angle1 = (HOME_M1);
    angle2 = 88.0; // Home position for Motor 2
    angle3 = -149.0; // Home position for Motor 3

}

void checkLimitSwitches(void) {
    // Check and handle limit switch 1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) {
        a = 1;
        m1speed = 0;
    }
    curstate = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
    if ( curstate == 1 && prvstate == 0)
	{
		TIM4->CNT = (int16_t)((HOME_M1 * 2.0 * PULSES_PER_REV * 2.0f) / 360.0f);
		angle1 = (HOME_M1);
	}

    prvstate = curstate;
    Motor1_SetSpeed(m1speed);
    // Check and handle limit switch 2, ensuring that it only works if limit switch 3 is not active
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) {
        b = 1;
        countPulseL2 = 0;  // Stop Motor 2 if limit switch 2 is triggered
    }

    if (a == 1 && b == 1 && c == 1 && home_count != 1){
    	home_count = 1;

	}

    // Check and handle limit switch 3
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
        c = 1;

		periodMotor3 = periodMotor2;
		countPulseL3 = countPulseL2;  // Stop Motor 3 if limit switch 3 is triggered

        HAL_GPIO_WritePin(GPIOA, DIR3_Pin, GPIO_PIN_RESET);  // Stop motor direction for M3
    } else {
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

void decode_angles_auto(const char *input, float *angles) {
    /*
     * Giải mã 18 góc từ chuỗi mã hóa.
     * `input` là chuỗi chứa dữ liệu mã hóa.
     * `angles` là mảng float để lưu giá trị góc đã giải mã.
     */

    for (int i = 0; i < 18; i++) {
        int offset = i * 7;  // Mỗi góc chiếm 7 ký tự
        int sign = (input[offset] == '1') ? 1 : -1;  // Xác định dấu (1: dương, 0: âm)

        // Tính góc bằng cách giải mã phần nguyên và phần thập phân
        angles[i] = sign * ((input[offset + 1] - '0') * 100 +
                            (input[offset + 2] - '0') * 10 +
                            (input[offset + 3] - '0') +
                            (input[offset + 4] - '0') * 0.1 +
                            (input[offset + 5] - '0') * 0.01 +
                            (input[offset + 6] - '0') * 0.001);
    }
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
	  HAL_UART_Receive_IT(&huart3, (uint8_t *)rx_data, 127);
	  decode_angles_auto(rx_data,&angles);
		angle1p1 = angles[0];
		angle2p1 = angles[1];
		angle3p1 = angles[2];

		angle1p2 = angles[3];
		angle2p2 = angles[4];
		angle3p2 = angles[5];

		angle1p3 = angles[6];
		angle2p3 = angles[7];
		angle3p3 = angles[8];

		angle1p4 = angles[9];
		angle2p4 = angles[10];
		angle3p4 = angles[11];

		angle1p5 = angles[12];
		angle2p5 = angles[13];
		angle3p5 = angles[14];

		angle1pt = angles[15];
		angle2pt = angles[16];
		angle3pt = angles[17];
		g = 0;

//	  decode_angles(rx_data, &rx_angle1, &rx_angle2, &rx_angle3);

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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_UART_Receive_IT(&huart3, (uint8_t *)rx_data, 127);

  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // Biến góc đích và góc hiện tại

//  PID_Init(&pid, 70.0f, 0.0f, 15.0f);  // Tùy chỉnh các hệ số Kp, Ki, Kd
  PID_Init(&pid, 2.0f, 0.0f, 0.0f);
  autoHomeMotors();
  PID_Init(&pid, 70.0f, 1.0f, 15.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	   if (dt > 10)
	   {

		   if (receiveComplete == 1){
			   g+=10;

			   if ( g >= 76000){
				   target_angle_1 = 0.0;
					target_angle_2 = 80.0;
					target_angle_3 = -135.0;
				}

			   else if (g > 73000){
					target_angle_1 = angle1pt;
					target_angle_2 = angle2pt;
					target_angle_3 = angle3pt;
					if (fabs(angle1 - target_angle_1) < 0.5 &&
					   fabs(angle2 - target_angle_2) < 0.5 &&
					   fabs(angle3 - target_angle_3) < 0.5) {
					   HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_RESET);} // Thả vật

				}else if (g > 69000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

				}else if (g > 65000){
					target_angle_1 = angle1p5;
					target_angle_2 = angle2p5;
					target_angle_3 = angle3p5;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

				}else if (g > 61000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;


				}else if (g > 57000){
						target_angle_1 = angle1pt;
						target_angle_2 = angle2pt;
						target_angle_3 = angle3pt;
						if (fabs(angle1 - target_angle_1) < 0.5 &&
						   fabs(angle2 - target_angle_2) < 0.5 &&
						   fabs(angle3 - target_angle_3) < 0.5) {
						   HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_RESET);} // Thả vật

			   	}else if (g > 53000){
			   		target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

			   	}else if (g > 49000){
					target_angle_1 = angle1p4;
					target_angle_2 = angle2p4;
					target_angle_3 = angle3p4;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

			   	}else if (g > 45000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;


				}else if (g > 41000){
					target_angle_1 = angle1pt;
					target_angle_2 = angle2pt;
					target_angle_3 = angle3pt;
					if (fabs(angle1 - target_angle_1) < 0.5 &&
					   fabs(angle2 - target_angle_2) < 0.5 &&
					   fabs(angle3 - target_angle_3) < 0.5) {
					   HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_RESET);} // Thả vật

				}else if (g > 37000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

				}else if (g > 33000){
					target_angle_1 = angle1p3;
					target_angle_2 = angle2p3;
					target_angle_3 = angle3p3;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

				}else if (g > 29000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;


				}else if (g > 25000){
					target_angle_1 = angle1pt;
					target_angle_2 = angle2pt;
					target_angle_3 = angle3pt;
					// Kiểm tra nếu đã đạt góc mục tiêu
				   if (fabs(angle1 - target_angle_1) < 0.5 &&
					   fabs(angle2 - target_angle_2) < 0.5 &&
					   fabs(angle3 - target_angle_3) < 0.5) {
					   HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_RESET);} // Thả vật

				}else if (g > 21000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

				}else if (g > 17000){
					target_angle_1 = angle1p2;
					target_angle_2 = angle2p2;
					target_angle_3 = angle3p2;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

				}else if (g > 13000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;


				}else if (g > 9000){
					target_angle_1 = angle1pt;
					target_angle_2 = angle2pt;
					target_angle_3 = angle3pt;
					if (fabs(angle1 - target_angle_1) < 0.5 &&
					   fabs(angle2 - target_angle_2) < 0.5 &&
					   fabs(angle3 - target_angle_3) < 0.5) {
					   HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_RESET);} // Thả vật
				}else if (g > 5000){
					target_angle_1 = 0.0;
					target_angle_2 = 77.0;
					target_angle_3 = -106.0;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);

				}else {
					target_angle_1 = angle1p1;
					target_angle_2 = angle2p1;
					target_angle_3 = angle3p1;
					HAL_GPIO_WritePin(GPIOA, MAG_Pin, GPIO_PIN_SET);
				}

		   angleControl(&pid, target_angle_1, target_angle_2, target_angle_3);
		   angle1err = fabs(angle1 - target_angle_1);
		   angle2err = fabs(angle2 - target_angle_2);
		   angle3err = fabs(angle3 - target_angle_3);
		   }

//		   HAL_UART_Transmit(&huart3, tx_data, sizeof(tx_data), 10);
		   dt = 0;
	   }
	   if (HOME == 0){
		   angleControl(&pid, 	0.0, 87.0, -149.0);
	   }


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
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 255;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
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
  HAL_GPIO_WritePin(GPIOA, MAG_Pin|PUL3_Pin|DIR3_Pin|PUL2_Pin
                          |DIR2_Pin|GPIO_PIN_5|DIR1A_Pin|DIR1B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MAG_Pin PA5 DIR1B_Pin */
  GPIO_InitStruct.Pin = MAG_Pin|GPIO_PIN_5|DIR1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL3_Pin DIR3_Pin PUL2_Pin DIR2_Pin
                           DIR1A_Pin */
  GPIO_InitStruct.Pin = PUL3_Pin|DIR3_Pin|PUL2_Pin|DIR2_Pin
                          |DIR1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_LINK3_Pin LIMIT_LINK2_Pin */
  GPIO_InitStruct.Pin = LIMIT_LINK3_Pin|LIMIT_LINK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
