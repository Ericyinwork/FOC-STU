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
#include "stdio.h"
#include "math.h"
#include "as5047p.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Polar 7     //极对数
#define PI 3.14159265358979f
#define _2PI 6.28318530717958f

uint16_t as5047_rx_data;
unsigned char angle_mon_flag,angle_start_mon;///初始角度记录
#define rotor_phy_angle (angle - angle_start_mon)     // 转子物理角度
#define rotor_logic_angle rotor_phy_angle * Polar          // 转子多圈角度  极对数
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

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
float angle;
uint16_t as5047_rx_data;
float angle_add,angle_Multi,angle_mon;////多圈角度变量



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
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);  // 启动定时器
  HAL_ADC_Start(&hadc1);  // 启动ADC  
  HAL_ADCEx_InjectedStart_IT(&hadc1);

	 AS5047P_CS_L;  // 设置CS低电平开始通信

  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)0x7fff,(uint8_t *)&as5047_rx_data,2);  // 启动SPI接收

  angle_mon_flag=1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   // printf("Angle: %f radians, Multi-turn angle: %f radians\n", angle, angle_Multi);
   // printf("system running...\n");
   // HAL_Delay(1000);  // 延时1秒
		printf("Angle: %f radians, Multi-turn angle: %f radians\r\n", angle, angle_Multi);
    printf("adc1 u_1: %f V, u_2: %f V\r\n", u_1, u_2);
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//** callback function **//
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // Transmission complete callback
  if (hspi->Instance == SPI3)
  {
    // Handle SPI3 transmission complete
    AS5047P_CS_H;  // Set CS high to end transmission
  //  printf("SPI3 Transmission Complete\n");
   as5047_rx_data = as5047_rx_data & 0x3FFF;  // Mask to get 14-bit angle data
   angle = _2PI * as5047_rx_data / 0x3FFF;  // Convert to angle in radians 
   if (angle_mon_flag == 1)  // If angle monitoring is enabled
   {
     angle_start_mon = angle;  // Record the initial angle
     angle_mon_flag = 0;  // Reset the flag
   //  printf("Initial angle recorded: %f radians\n", angle_start_mon);        
   }
   float angle_deff = (float)as5047_rx_data - angle_mon;  // Calculate angle difference
   if(abs(angle_deff) > (0.8*16383))  // If angle difference exceeds threshold
   {
     angle_add += (angle_deff > 0) ? -_2PI : _2PI;  // Adjust angle_add based on direction
   } 
   angle_mon = as5047_rx_data;
   angle_Multi = angle_add + angle;  // Calculate the multi-turn angle
 //  printf("Angle: %f radians, Multi-turn angle: %f radians\n", angle);

   AS5047P_CS_L;  // Set CS low to start next transmission
   HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)0x7fff,(uint8_t *)&as5047_rx_data,2);  // 启动SPI接收
  // printf("Next spi Transmit\n");
	}
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)  
  {   
    u_1 = 3.3f * (float)hadc->Instance->JDR1 / (1<<12);  // Convert ADC value to voltage
    u_2 = 3.3f * (float)hadc->Instance->JDR2 / (1<<12);  // Convert ADC value to voltage
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
