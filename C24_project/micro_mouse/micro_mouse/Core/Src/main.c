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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

	float error;
	float previous_error;
	float P_part;
	float D_part;
	float I_part;
	float PID_value;

	float errorL;
	float errorR;
	float P_part_L;
	float P_part_R;
	float D_part_L;
	float D_part_R;
	float I_part_L;
	float I_part_R;
	float PID_valueR;
	float PID_valueL;
	float previous_errorR;
	float previous_errorL;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PIN GPIO_PIN_12
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_13
#define ECHO_PORT GPIOB
#define TRIG_PIN_1 GPIO_PIN_10
#define TRIG_PORT_1 GPIOA
#define ECHO_PIN_1 GPIO_PIN_9
#define ECHO_PORT_1 GPIOA
#define TRIG_PIN_2 GPIO_PIN_14
#define TRIG_PORT_2 GPIOB
#define ECHO_PIN_2 GPIO_PIN_15
#define ECHO_PORT_2 GPIOB
#define		MAX_CNT			899
#define 	MIN_CNT			10
//#define ENCODER_PIN_1 GPIO_PIN_1
//#define ENCODER_PIN_1 GPIO_PIN_2
//#define ENCODER_PIN_2 GPIO_PIN_3
//#define ENCODER_PIN_2 GPIO_PIN_4
//#define ENCODER_PORT GPIOA

double 	v_desire=25;
double khoang_cach_giua =5;
double duty_base=1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//void turn_right;
//void turn_left;
void Forward (float duty_right, float duty_left);
//void caculate_pid(PID_t *pid);
//void caculate_error( PID_t *pid);
//void caculate_error_encoder( PID_encoder_t *pid_encoder);
//void control_motor(PID_t *pid, PID_encoder_t *pid_encoder);
//void reset_pid(PID_t *pid);
void sensor_right();
void sensor_left();
void sensor_forward();
static void Set_duty(float duty, TIM_HandleTypeDef *htim, int channel);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
float Kp;
float Kd;
float Ki;
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint32_t Value3 = 0;
uint32_t Value4 = 0;
uint32_t Value5 = 0;
uint32_t Value6 = 0;
uint16_t Distance  = 0;
uint16_t Distance_1=0;// Distance_left
uint16_t Distance_2=0;// Distance_right
uint16_t Tim_example=0.01;//10ms
double van_toc_R;
double van_toc_L;
int count_R=0;
int count_L=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//reset pid
/*void reset_pid(PID_t *pid){
	pid->error=0;
	pid->previous_error=0;
	pid->P_part=0;
	pid->I_part=0;
	pid->D_part=0;
}*/
void sensor_right(){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		      __HAL_TIM_SET_COUNTER(&htim1, 0);
		      while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		      HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

		      pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		      // wait for the echo pin to go high
		      while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
		      Value1 = __HAL_TIM_GET_COUNTER (&htim1);

		      pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		      // wait for the echo pin to go low
		      while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
		      Value2 = __HAL_TIM_GET_COUNTER (&htim1);

		      Distance = (Value2-Value1)* 0.034/2;
		     // if ( Distance <= 12 ) led_right=1;
		      //else led_right=0;

}
void sensor_left(){
	HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
		  	    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		  	    HAL_GPIO_WritePin(TRIG_PORT_1, TRIG_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin low

		  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		  	    // wait for the echo pin to go high
		  	    while (!(HAL_GPIO_ReadPin (ECHO_PORT_1, ECHO_PIN_1)) && pMillis + 10 >  HAL_GetTick());
		  	    Value3 = __HAL_TIM_GET_COUNTER (&htim1);

		  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		  	    // wait for the echo pin to go low
		  	    while ((HAL_GPIO_ReadPin (ECHO_PORT_1, ECHO_PIN_1)) && pMillis + 50 > HAL_GetTick());
		  	    Value4 = __HAL_TIM_GET_COUNTER (&htim1);

		  	    Distance_1 = (Value4-Value3)/2/29.412;
		  	   // if (Distance_1 <= 12) led_left=1;
		  	    //else led_left=0;
}
void sensor_forward(){
	 HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		  	  	  	    __HAL_TIM_SET_COUNTER(&htim1, 0);
		  	  	  	    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		  	  	  	    HAL_GPIO_WritePin(TRIG_PORT_2, TRIG_PIN_2, GPIO_PIN_RESET);  // pull the TRIG pin low

		  	  	  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		  	  	  	    // wait for the echo pin to go high
		  	  	  	    while (!(HAL_GPIO_ReadPin (ECHO_PORT_2, ECHO_PIN_2)) && pMillis + 10 >  HAL_GetTick());
		  	  	  	    Value5 = __HAL_TIM_GET_COUNTER (&htim1);

		  	  	  	    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		  	  	  	    // wait for the echo pin to go low
		  	  	  	    while ((HAL_GPIO_ReadPin (ECHO_PORT_2, ECHO_PIN_2)) && pMillis + 50 > HAL_GetTick());
		  	  	  	    Value6 = __HAL_TIM_GET_COUNTER (&htim1);

		  	  	  	   Distance_2 = (Value6-Value5)/2/29.412;
		  	  	  	  // if (Distance_2 <= 12) led_front=1;
		  	  	  	   //else led_front=0;

}
//void caculate_error(PID_t *pid){}
void Init_tim_pwm(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}
void Init_sensor_sound(){
	 HAL_GPIO_WritePin(GPIOB, TRIG_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, TRIG_PIN_2, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}
static void Set_duty(float duty, TIM_HandleTypeDef *htim, int channel){
	if (channel == 1){
		htim->Instance->CCR1 =  (duty*MAX_CNT) + MIN_CNT;
	}
	if (channel == 2){
		htim->Instance->CCR2 =  (duty*MAX_CNT) + MIN_CNT;
	}
}

void Forward (float duty_right, float duty_left){
	Set_duty(0, &htim3, 1);
	Set_duty(duty_right, &htim3, 2);
	Set_duty(0, &htim4, 1);
	Set_duty(duty_left, &htim4, 2);
}
void stop (float duty_right, float duty_left){
	Set_duty(0, &htim3, 1);
	Set_duty(0, &htim3, 2);
	Set_duty(0, &htim4, 1);
	Set_duty(0, &htim4, 2);
}
void turn_right (float duty_right, float duty_left){
	Set_duty(0, &htim3, 1);
	Set_duty(duty_right, &htim3, 2);
	Set_duty(0, &htim4, 1);
	Set_duty(0, &htim4, 2);
}
void turn_left (float duty_right, float duty_left){
	Set_duty(0, &htim3, 1);
	Set_duty(0, &htim3, 2);
	Set_duty(0, &htim4, 1);
	Set_duty(duty_left, &htim4, 2);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==GPIO_PIN_0){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==0){
			count_R++;
		}
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1){
					count_R--;
				}
	}
	if(GPIO_Pin==GPIO_PIN_1){
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0){
				count_R--;
			}
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==1){
						count_R++;
					}
		}
	if(GPIO_Pin==GPIO_PIN_3){
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==0){
				count_L++;
			}
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==1){
						count_L--;
					}
		}
		if(GPIO_Pin==GPIO_PIN_4){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==0){
					count_L--;
				}
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==1){
							count_L++;
						}
			}

}
void control_motor()
	{
		double motor_right=(PID_valueR+PID_value)/100;
		double motor_left=(PID_valueL-PID_value)/100;
		if(motor_right>duty_base){
			motor_right=1;
			motor_left=0;
		}
		if(motor_left>duty_base){
					motor_right=0;
					motor_left=1;
				}
		if(Distance_2<10&&Distance_1<10){
			Forward(motor_right, motor_left);
		}

		else if(Distance<5&&Distance_2>10){
			stop(motor_right, motor_left);
			HAL_Delay(50);
			turn_right(motor_right, motor_left);
		}
		else if(Distance<5&&Distance_1>10){
					stop(motor_right, motor_left);
					HAL_Delay(50);
					turn_left(motor_right, motor_left);
				}

	}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		van_toc_R=(count_R/100)*3.14*6.5;
			van_toc_L=(count_L/100)*3.14*6.5;
			count_R=0;
			count_L=0;
		//void caculate_error( PID_t *pid){

		    if(Distance_1<10&&Distance_2<10){
		    	error=Distance_1-Distance_2;
		    }
		    else if(Distance_2>10){
		    	error=2;
		    }
		    else if(Distance_1>10){
		    	error=-2;
		   // }
previous_error=error;
		}
	//	void caculate_pid(PID_t *pid){
			P_part=Kp*error;
			D_part=Kd*(error-previous_error)/Tim_example;
			I_part=Ki*(error+previous_error)*Tim_example;
			PID_value=D_part+I_part+P_part;
		//}
		//void caculate_error_encoder( PID_encoder_t *pid_encoder){
				errorR=v_desire-van_toc_R;
				D_part_R=Kd*errorR;
				P_part_R=Kp*(errorR-previous_errorR)/Tim_example;
				I_part_R=Ki*(errorR+previous_errorR)*Tim_example*0.5;
				PID_valueR=D_part_R+I_part_R+P_part_R;

				errorR=v_desire-van_toc_L;
				D_part_L=Kd*errorL;
				P_part_L=Kp*(errorL-previous_errorL)/Tim_example;
				I_part_L=Ki*(errorL+previous_errorL)*Tim_example*0.5;
				PID_valueL=D_part_L+I_part_L+P_part_L;
			//}
				control_motor();

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
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
Init_tim_pwm();
Init_sensor_sound();
HAL_TIM_Base_Start(&htim1);

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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
