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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
int trc1, trc2, trc3, trc4, trc5;
int case1_1, case1_2, case1_3;
int case2_1, case2_2, case2_3, case2_4, case2_5, case2_6;
int case3_1, case3_2, case3_3, case3_4, case3_5, case3_6;
int case4, case5;
int StartFlag = 1;
int EndFlag = 0;
int Flag_Right = 0, Flag_Left = 0;
int endpoint = 0;
float PD_feedback = 0;

//* UART Variables
uint8_t RxBuff[1] = {0};
uint8_t DataBuff[10] = {0};
int RxLine = 0;
uint8_t Recieve_flag = 0;
float Recieve_val = 0;
int scale = 10;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void StartMotor(void);
void TurnRight(void);
void TurnLeft(void);
void TurnBack(void);
void RotateRight(void);
void RotateLeft(void);
void Foward(void);

//* UART Functions
void Process_Buffer();
void Communicate_Upper();
void SetVelocity(float diff);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_UART_Transmit_IT(&huart3, (uint8_t *)"END\n", 5); 
  while (1)
  {	
		
		//Read the trace sensor data
		trc1 = HAL_GPIO_ReadPin(Trace1_GPIO_Port, Trace1_Pin);
		trc2 = HAL_GPIO_ReadPin(Trace2_GPIO_Port, Trace2_Pin);
		trc3 = HAL_GPIO_ReadPin(Trace3_GPIO_Port, Trace3_Pin);
		trc4 = HAL_GPIO_ReadPin(Trace5_GPIO_Port, Trace5_Pin);
		trc5 = HAL_GPIO_ReadPin(Trace4_GPIO_Port, Trace4_Pin);
		
		//printf("out1:%d\r\nout2:%d\r\nout3:%d\r\nout4:%d\r\nout5:%d\r\n",trc1, trc2, trc3, trc4, trc5);
		
		//Situation Forward:
		case1_1 = trc1==1 && trc2==0 && trc3==0 && trc4==1 && trc5==1;
		case1_2 = trc1==1 && trc2==1 && trc3==0 && trc4==0 && trc5==1;
		case1_3 = trc1==1 && trc2==0 && trc3==0 && trc4==0 && trc5==1;
		//Situation Right:
		case2_1 = trc1==0 && trc2==0 && trc3==1 && trc4==1 && trc5==1;
		case2_2 = trc1==0 && trc2==1 && trc3==1 && trc4==1 && trc5==1;
	  case2_3 = trc1==1 && trc2==0 && trc3==1 && trc4==1 && trc5==1; //out2 black - small correction 
	  case2_4 = trc1==0 && trc2==0 && trc3==0 && trc4==1 && trc5==1;
		case2_5 = trc1==0 && trc2==0 && trc3==0 && trc4==0 && trc5==1;
		case2_6 = trc1==0 && trc2==1 && trc3==0 && trc4==1 && trc5==1;
		//Situation Left:
		case3_1 = trc1==1 && trc2==1 && trc3==1 && trc4==0 && trc5==0;
		case3_2 = trc1==1 && trc2==1 && trc3==1 && trc4==1 && trc5==0;
		case3_3 = trc1==1 && trc2==1 && trc3==1 && trc4==0 && trc5==1; //out4 black - small correction 
		case3_4 = trc1==1 && trc2==1 && trc3==0 && trc4==0 && trc5==0;
		case3_5 = trc1==1 && trc2==0 && trc3==0 && trc4==0 && trc5==0;
		case3_6 = trc1==1 && trc2==1 && trc3==0 && trc4==0 && trc5==0;
    //other SItuation
		case4 = trc1==0 && trc2==0 && trc3==0 && trc4==0 && trc5==0;
		case5 = trc1==1 && trc2==1 && trc3==1 && trc4==1 && trc5==1;
		
		if (StartFlag){
			StartMotor();
			StartFlag = 0;
			HAL_UART_Transmit_IT(&huart3, (uint8_t *)"STA\n", 5); 
		}
		
		if (EndFlag==0 && endpoint >=5){
			EndFlag = 1;
			HAL_UART_Transmit_IT(&huart3, (uint8_t *)"END\n", 5); 
		}
		
		
		if (EndFlag){
			//printf("End!");
			// HAL_UART_Transmit_IT(&huart3, (uint8_t *)"END\n", 5); 
			Motor_Rotate(1,1500,2000);
			Motor_Rotate(2,1500,2000);
			Motor_Rotate(3,1500,2000);
			Motor_Rotate(4,1500,2000);
		}else{
			
			// Communicate_Upper();
			
			if (case1_1 || case1_2 || case1_3){
				Foward();
				//printf("Foward!\r\n");
			}else if(case2_1 || case2_2 || case2_3 || case2_4 || case2_5 ){
				RotateLeft();
				//printf("Left!\r\n");
			}else if(case3_1 || case3_2 || case3_3 || case3_4 || case3_5 ){
				RotateRight();
				//printf("Right!\r\n");
			}else if(case5){
				TurnBack();
				printf("Back!\r\n");
			}else if(case4){
				endpoint = endpoint + 1;
				Foward();
				HAL_Delay(100);
				//printf("!!!!!!!!!!!!!!!!!!!!!!!, end: %d\r\n", endpoint);
			}else{
				Foward();
			}
			
			HAL_Delay(1); //Frequency: 1000Hz
				
		}
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
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
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Communicate_Upper()
{
	if (Recieve_flag == 1 && RxLine != 0)
	{
		Process_Buffer();
		memset(DataBuff, 0, sizeof(DataBuff));
		RxLine = 0;
		Recieve_flag = 0;
	}
}

void Process_Buffer()
{
	// no turnning while translation, no translation while turnning
	sscanf((const char *)(&DataBuff[0]), "%f", &Recieve_val);
	SetVelocity(Recieve_val);
	HAL_UART_Transmit_IT(&huart3, (uint8_t *)"REC\n", 4); 
	Recieve_val = 0;
}

void SetVelocity(float diff){
	/*ID: 1-RightFront 2-RightRear 3-LeftRear 4-LeftFront*/
	/*Velocity: 1500-2500_clockwise 500-1500_counterclocwist*/
	Motor_Rotate(1,1000+diff*scale,1000);
	Motor_Rotate(2,1000+diff*scale,1000);
	Motor_Rotate(3,2000+diff*scale,1000);
	Motor_Rotate(4,2000+diff*scale,1000);
}

// bluetooth communication format: val + A
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart3.Instance)
	{
		if (RxBuff[0] == 'A')
		{
			Recieve_flag = 1;
		}
		else
		{
			RxLine++;
			DataBuff[RxLine - 1] = RxBuff[0];
		}
		RxBuff[0] = 0;

		HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff, 1);
	}
}

//Start our Motors
void StartMotor(void){
	Motor_Rotate(1,800,2000);
	Motor_Rotate(2,800,2000);
	Motor_Rotate(3,2000,2000);
	Motor_Rotate(4,2000,2000);
}
//Move Right
void TurnRight(void)
{
	/*ID: 1-RightFront 2-RightRear 3-LeftRear 4-LeftFront*/
	/*Velocity: 1500-2500_clockwise 500-1500_counterclocwist*/
	Motor_Rotate(1,1100,2000);
	Motor_Rotate(2,1100,2000);
	Motor_Rotate(3,2000,2000);
	Motor_Rotate(4,2000,2000);
}

void RotateRight(void){
	/*ID: 1-RightFront 2-RightRear 3-LeftRear 4-LeftFront*/
	/*Velocity: 1500-2500_clockwise 500-1500_counterclocwist*/
	if (case3_3){ //small correction - Just turn not rotate
		Motor_Rotate(1,1000,1000);
		Motor_Rotate(2,1000,1000);
		Motor_Rotate(3,2020,1000);
		Motor_Rotate(4,2020,1000);
	}else if (case3_1){
		Motor_Rotate(1,1900,1000);
		Motor_Rotate(2,1900,1000);
		Motor_Rotate(3,1900,1000);
		Motor_Rotate(4,1900,1000);
	}else if (case3_2){
		Motor_Rotate(1,2200,2000);
		Motor_Rotate(2,2200,2000);
		Motor_Rotate(3,2200,2000);
		Motor_Rotate(4,2200,2000);
	}else{
		Motor_Rotate(1,2400,2000);
		Motor_Rotate(2,2400,2000);
		Motor_Rotate(3,2400,2000);
		Motor_Rotate(4,2400,2000);
	}
}

//Move left
void TurnLeft(void)
{
	/*ID: 1-RightFront 2-RightRear 3-LeftRear 4-LeftFront*/
	/*Velocity: 1500-2500_clockwise 500-1500_counterclocwist*/
	Motor_Rotate(1,1000,2000);
	Motor_Rotate(2,1000,2000);
	Motor_Rotate(3,1700,2000);
	Motor_Rotate(4,1700,2000);
}

void RotateLeft(void){
	/*ID: 1-RightFront 2-RightRear 3-LeftRear 4-LeftFront*/
	/*Velocity: 1500-2500_clockwise 500-1500_counterclocwist*/
	if (case2_3){ //small correction - Just turn not rotate
		Motor_Rotate(1,980,2000);
		Motor_Rotate(2,980,2000);
		Motor_Rotate(3,2000,2000);
		Motor_Rotate(4,2000,2000);
	}else if (case2_1){
		Motor_Rotate(1,560,2000);
		Motor_Rotate(2,560,2000);
		Motor_Rotate(3,560,2000);
		Motor_Rotate(4,560,2000);
	}else if(case2_2){
		Motor_Rotate(1,530,2000);
		Motor_Rotate(2,530,2000);
		Motor_Rotate(3,530,2000);
		Motor_Rotate(4,530,2000);
	}else {
		Motor_Rotate(1,500,2000);
		Motor_Rotate(2,500,2000);
		Motor_Rotate(3,500,2000);
		Motor_Rotate(4,500,2000);
	}
}


//Move Foward
void Foward(void)
{
	/*ID: 1-RightFront 2-RightRear 3-LeftRear 4-LeftFront*/
	/*Velocity: 1500-2500_clockwise 500-1500_counterclocwist*/
	Motor_Rotate(1,1000,2000);
	Motor_Rotate(2,1000,2000);
	Motor_Rotate(3,2000,2000);
	Motor_Rotate(4,2000,2000);
}

//Move back
void TurnBack(void)
{
	/*ID: 1-RightFront 2-RightRear 3-LeftRear 4-LeftFront*/
	/*Velocity: 1500-2500_clockwise 500-1500_counterclocwist*/
	Motor_Rotate(1,2100,2000);
	Motor_Rotate(2,2100,2000);
	Motor_Rotate(3,900,2000);
	Motor_Rotate(4,900,2000);
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
