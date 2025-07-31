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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#define RT_MAIN_DEBUG
#define DBG_TAG               "mian"    //dbg的表头
#ifdef RT_MAIN_DEBUG                    //dbg宏定义
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_ERROR
#endif
#include <rtdbg.h>						 //dbg头文件


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
 #else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
 #endif
 PUTCHAR_PROTOTYPE
 {
 HAL_UART_Transmit(&huart4 , (uint8_t *)&ch, 1, 0xFFFF);
 return ch;
 }

float u_1,u_2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


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
	int result;
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
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);  // 启动定时器
  HAL_ADC_Start(&hadc1);  // 启动ADC  
  HAL_ADCEx_InjectedStart_IT(&hadc1);
		result = init_motor_control();
				if( result == RT_EOK)
		{
			LOG_D("motor is init \r\n");
		}else
		{
			LOG_E("motor is init faild \r\n");
		}
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LOG_P("rt-thread is runing!\r\n");

		
	rt_thread_mdelay(100);	
	as5047_start();	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		LOG_P("Angle: %f radians, Multi-turn angle: %f radians\r\n", angle, angle_Multi);
//		LOG_P("Angle: %d radians, Multi-turn angle: %d radians\r\n", angle, angle_Multi);
//		LOG_D("Angle: %d radians, Multi-turn angle: %d radians\r\n", angle, angle_Multi);
		LOG_D("Angle: %d radians, Multi-turn angle: %d radians\r\n", angle, angle_Multi);
//    printf("adc1 u_1: %f V, u_2: %f V\r\n", u_1, u_2);
//		printf("adc1 u_1: %f V, u_2: %f V\r\n", u_1, u_2);
		rt_thread_mdelay(1000);


	//	rt_kprintf("rt-thread is run in while!\r\n");
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	 if (hspi->Instance == SPI3)
//	 {
//	//drv8301_reg1_rx_data  = drv8301_reg1_rx_data & DRV8301_DATA_MASK;

//	printf("drv8301_reg1_rx_data is %x /r/n",drv8301_reg1_rx_data);
//		 
//	printf("drv8301_reg2_rx_data is %x /r/n",drv8301_reg2_rx_data);

//	DRV8301_CS_H;
//	 }
//	
//}



//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//  if (hadc->Instance == ADC1)  
//  {   
//    u_1 = 3.3f * (float)hadc->Instance->JDR1 / (1<<12);  // Convert ADC value to voltage
//    u_2 = 3.3f * (float)hadc->Instance->JDR2 / (1<<12);  // Convert ADC value to voltage
//  }
//  
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
