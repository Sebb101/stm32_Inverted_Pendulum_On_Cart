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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define derivative_sample_size 5 //derivative sample size
#define sample_size 50 //sample size for data acquisition

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//encoder functions
void init_hardware_timer_version(void);
///void init_interrupt_version(void);

// Error function
static inline void calculate_error();
static inline void moving_average_derivative_calculation();

// end of copy and paste code

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// loop counter
static long loop_count = 0;

// USB Send
struct XL_Data{
  double ang,volt;
}usb_data;

// SIN WAVE variables
double a = 100; // amplitude ; a / 100 * 12 = Motor Voltage
double F1 = 2.001; // Frequency 1
double Fmin = 2; // Min Frequency
double Fmax = 13; // Max Frequency
double t = 0;
double Fs = 333.3; // Sample Frequency
double MAXPHASE = 65536;
double iphase = 0;
double delta = 0;

// Rotary Pendulum Encoder Variables
double angle = 0; //0 == down position ; 3.14 == up position
double angular_velocity = 0;
double previous_angle = 0;
//float radian_angle = 3.14;

// Rotary Cart Encoder Variables
int cartcounter = 0;
float cartpos = 0;

const float angle_radians_factor = 0.002617993; // Rotary encoder rotation 2*pi == 2400 ticks
const float radians_meters_factor = 0.08; // Rotary encoder radius is 0.08 m

//moving average arrays
double angular_velocity_ma[derivative_sample_size];
double oldest_angular_velocity;



//set point variables
double x_desired = 0.0;
double xdot_desired = 0.0;
double theta_desired = 3.14159265;
double thetadot_desired = 0;

//error variables
double x_error = 0;
double xdot_error = 0;
double theta_error = 0;
double thetadot_error = 0;

//time variables
double current_time;
double previous_time;
double dt = 0;


// Motor Variables
double PIDvalue = 0;
double mtr_volt = 0;
static int minimum_motor_feedback = 0; //minimum PWM signal to move motor
int motorDir = 0;
int motorSpeed=0;

void init_hardware_timer_version(void)
{
 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
 RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //AFIO might not even be needed?

 //GPIO must be input floating which is default so no code to write for that

 // value to count up to : 16 bit so max is 0xFFFF = 65535
 //TIM4->ARR = 0xFFFF;
 TIM4->ARR = 0x0960;

 //per datasheet instructions
 TIM4->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 );  //step 1 and 2
 TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);  // step 3 and 4
 TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;   //step 5
 TIM4->CR1 |= TIM_CR1_CEN ;     //step 6
}


void init_interrupt_version(void)
{
 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
 EXTI->IMR |= 1<<8; //enable interrupt on EXTI-8
 //EXTI->IMR |= 1<<9; //enable interrupt on EXTI-9
 NVIC_EnableIRQ(EXTI9_5_IRQn); //enable the IRQ line that corresponds to EXT-5 to 9
 EXTI->RTSR |= 1<<8; //enable rising edge interrupt to EXTI 8
 //EXTI->FTSR |= 1<<9; //enable falling edge interrupt to EXTI 9
 AFIO->EXTICR[3] |= 1; // set interrupt EXTI 8 to be on port B
 //AFIO->EXTICR[3] |= 1<<4; // set interrupt EXTI 9 to be on port B
}

static inline void calculate_error(){
	//calculate errors
	theta_error = angle - theta_desired;
	thetadot_error = angular_velocity - thetadot_desired;
}

static inline void moving_average_derivative_calculation(){
	//moving average for derivative calculations

	oldest_angular_velocity = angular_velocity_ma[loop_count % derivative_sample_size];
	angular_velocity_ma[loop_count % derivative_sample_size] = (angle - previous_angle) / (dt);
	angular_velocity += ((angular_velocity_ma[loop_count % derivative_sample_size] - oldest_angular_velocity) / derivative_sample_size);

}
//struct XL_Data * data


void sendusbData(double timeinterval,double pend_pos, double mtr_volt){
    uint8_t bufferOne[50];
    //uint8_t *bufferOne = "this is a testr\r\n";
    sprintf(bufferOne, "%0.7f,%0.7f,%0.7f\r\n", timeinterval, pend_pos, mtr_volt);
    CDC_Transmit_FS( (uint8_t *)bufferOne, strlen(bufferOne));

}

double hz_to_delta( double hz )
{
    return hz/Fs;
}

double sample_phase( double phase )
{
    return a*sin(phase*M_PI );
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // PWMA - MotorA PWM setup
  // Duty% = CCR / COUNTERPERIOD
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

  // Encoder timer input
  init_hardware_timer_version();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Test board LED PB1
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
	  //HAL_Delay(1000);
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
	  //HAL_Delay(1000);


	  //Get time
	  //current_time = (HAL_GetTick() );
	  current_time = (HAL_GetTick() )* 0.001;
	  dt = current_time - previous_time;
	  //HAL_Delay(5);

	  // Send Data via USB
	  sendusbData(dt,angle,PIDvalue);
	  //sendusbData(dt,angle,mtr_volt);
	  HAL_Delay(2);

	  //Encoder Pendulum Angle
	  angle = (TIM4->CNT)*angle_radians_factor;

	  // Encoder Cart Position
	  cartpos = (cartcounter)*(0.00007);

	  // Calculate Angular Velocity
	  //moving_average_derivative_calculation();
	  angular_velocity = (angle - previous_angle) / (dt);

	  // Error between set point and measurement
	  calculate_error();

	  // Motor  Control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	  // PID Controller
	  PIDvalue = (PID(&theta_error, &dt, &angle, &previous_angle));

	  // Save previous values
	  previous_time = current_time;
	  previous_angle = angle;

	  // Sin Wave Input (F1 = angular velocity)
	  //PIDvalue = a*sin(F1*t);

	  // Cont. Freq changing Sin Wave Input
	  /*
	  if (t>=5 && F1 >= Fmin && F1 <= Fmax){
	  delta = hz_to_delta( F1 );
	  PIDvalue = sample_phase( iphase += delta );
	  F1+=0.0005;
	  }else{
		  PIDvalue = 0;
	  }
	*/
	  // Step function 1sec, 2sec & 3sec
	  /*
	  if (t>=5 && t<=5.03){
		  PIDvalue+=10;
	  }
	  if (t>=5.5 && t<=5.53){
		  PIDvalue-=10;
	  }
	*/

	  // Motor Speed & Direction Code ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	  if (PIDvalue < minimum_motor_feedback && PIDvalue > 0){
		  // Set motor PWM to min signal & move right
		  motorSpeed = minimum_motor_feedback;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		  motorDir=1; //CW & Right = 1 ; CCW & Left = 0
	  }
	  else if (PIDvalue > minimum_motor_feedback && PIDvalue > 0) {
		  // Set motor PWM to PIDvalue & move right
		  motorSpeed = PIDvalue;
		  if (PIDvalue > 100){
			  motorSpeed = 100;
		  }
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
		  motorDir=1;
	  }
	  else if (PIDvalue > -minimum_motor_feedback && PIDvalue < 0) {
		  // Set motor PWM to min signal & move left
		  motorSpeed = minimum_motor_feedback;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  motorDir=0;
	  }
	  else if (PIDvalue < -minimum_motor_feedback && PIDvalue < 0) {
		  // Set motor PWM to min signal & move left
		  motorSpeed = -PIDvalue;
		  if (PIDvalue < -100){
			  motorSpeed = 100;
		  }
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  motorDir=0;
	  }
	  else if (PIDvalue == 0){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	  }


	  // MotorSpeed value between 25-100
	  TIM2->CCR1 = motorSpeed;
	  mtr_volt = (motorSpeed / 100.0) * 12;

	  t+=0.003;


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
