/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ZERO 0b11000000
#define ONE 0b11111000

#define MAX_LED_NUMBER 6
#define GREEN 0
#define RED 255
#define BLUE 0



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buff[72];
uint8_t reset_buffer[48];
RGB_color single_color;
uint8_t switch_mode=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	for(int i=0;i<72;i++)
		{
			buff[i]=0;
		}
	for(int i=0;i<49;i++)
	{
		reset_buffer[i]=0;
	}
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  single_color.blue=0;
  single_color.green=0;
  single_color.red=255;

  HAL_SPI_Init(&hspi2);

  Load_Color_Into_Buffer(single_color,&buff[0]);
  //Set_One_Color(single_color);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Set_LED_Queue(single_color,1);

	  switch(switch_mode){
	  case 0:
		  Reset_LED_color();
		  Set_One_Color(single_color);
		  break;

	  case 1:
		  Set_LED_Queue(single_color,1);
		  break;

	  case 2:
		  Reset_LED_color();
		  break;

	  case 3:
		  single_color.red=single_color.red+10;
		  Reset_LED_color();
		  Set_One_Color(single_color);
		  break;

	  case 4:
		  single_color.green=single_color.green+10;
		  Reset_LED_color();
		  Set_One_Color(single_color);
		  break;

	  case 5:
	 	  single_color.blue=single_color.blue+10;
	 	 Reset_LED_color();
	 	  Set_One_Color(single_color);
	 	 break;
	  }

	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	  	HAL_Delay(500);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/* USER CODE BEGIN 4 */
void Blink_LED()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	HAL_Delay(500);
}
void Send_Reset()
{
	for(int i=0;i<49;i++)
		{
			reset_buffer[i]=0;
		}
	HAL_SPI_Transmit(&hspi2, &reset_buffer[0], 48, 1000);
}
void Set_One_Color(RGB_color color)
{
	Load_Color_Into_Buffer(color,&buff[0]);
	for(int i=0;i<MAX_LED_NUMBER;i++)
		  	{
			  HAL_SPI_Transmit(&hspi2, &buff[0], 28, 1000);
		  	}

			Send_Reset();
		  //HAL_SPI_Transmit(&hspi2, &reset_buffer[0], 48, 1000);

}
void Set_LED_Queue(RGB_color color, uint8_t mode)
{
	uint8_t current_LED=0;
	while(current_LED<=MAX_LED_NUMBER){
		for(int i=0;i<current_LED;i++)
			  	{
				  HAL_SPI_Transmit(&hspi2, &buff[0], 28, 1000);

			  	}
		current_LED++;
		HAL_Delay(500);

		Send_Reset();


}
	 Reset_LED_color();
}
void Reset_LED_color()
{	RGB_color c;
	c.red=0;
	c.green=0;
	c.blue=0;
	Load_Color_Into_Buffer(c,&reset_buffer[0]);
	for(int i=0;i<MAX_LED_NUMBER;i++)
			  	{
				  HAL_SPI_Transmit(&hspi2, &reset_buffer[0], 28, 1000);
			  	}
	Send_Reset();

}
void Load_Color_Into_Buffer(RGB_color set_color, uint8_t *buffer)
{
		uint8_t color_t;
		uint8_t color[3];
		uint8_t k=0;
		//char msg[64];
		color[0]=set_color.green;
		color[1]=set_color.red;
		color[2]=set_color.blue;
		for(uint8_t i = 0; i < 3; i++){
			color_t=color[i];
			for(uint8_t j = 0; j < 8; j++)
			{
				if(color_t&0b10000000)
				{buffer[k]=ONE;
				}
				else
				{
					buffer[k]=ZERO;
				}
				k++;
				color_t=color_t<<1;
			}
		}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_13){
		switch_mode++;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		if(switch_mode>5){switch_mode=0;}


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
