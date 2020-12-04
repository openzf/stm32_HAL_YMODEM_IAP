/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stmflash.h"
#include "kymodem.h"
#include "dev_com.h"
#include "drv_com.h"
#include "iap.h"
#include "flash_if.h"



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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//#define USE_RX_MODE
#define KAPP_ADDR  (uint32_t)0x08004000
// 串口接受回调
void g_com_rx_callBack(unsigned char* data,uint32_t size)
{
	#ifdef USE_RX_MODE
	ymodem_rx_handle(data,size);
	#else
	ymodem_tx_handle(data,size);
	#endif
}

// 串口错误回调
void g_ymodem_rx_error_handle(int error_code)
{
	printf("--error code :%d--\r\n",error_code);
}
// 接受head回调
char g_ymodem_rx_head_handle(char *file_name,uint16_t file_name_len, uint32_t file_len)
{
	printf("file:%s %d\r\n",file_name,file_len);
	FLASH_If_Erase( KAPP_ADDR);
	return 0;
}
uint32_t app_addr = KAPP_ADDR;

// 接受数据回调
void g_ymodem_rx_data_handle(char *data, uint16_t len,uint32_t download_byte,uint8_t percent)
{
	printf("data len:%d  %d %d\r\n",len,download_byte ,percent);
	if (FLASH_If_Write(app_addr,(uint32_t*) data,len/4) == FLASHIF_OK)
	{
		app_addr += len;
		printf("data write ok\r\n");
	}else{
		printf("data write error\r\n");
	}
}

// 接受完成回调
void g_ymodem_rx_finish_handle(int state)
{
	if(state ==0){
		printf("--file end--\r\n");
		iap_load_app(KAPP_ADDR);
	}else{
		printf("--file end error1 :%d--\r\n",state);
	}
}

//--- sent 
char name[] = "testupload.txt";
char file[] = "asdjlfaj129384719823749817239847198273498sdflajsldfjalsdjflasa134917239419823749817298347918237haksjdhfkahsdfkjhaskdjfhkahsd123456789";
// 发送数据处理
void g_ymodem_tx_data_handle(uint8_t **file_read_addr, uint32_t  file_read_size, uint32_t file_has_read_size,  uint32_t file_remain_size,uint8_t percent)
{
	printf("read size:%d  has_read:%d  remain:%d  per:%d\r\n",file_read_size,file_has_read_size,  file_remain_size,percent);
			// 指针指向的地址 重新指向
		*file_read_addr = &file[file_has_read_size];
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_PE(USART1);   
	
		
	dev_comctrl_init();
	dev_comctrl_regist_rx_callback(g_com_rx_callBack);
	
	
	Ymodem_TypeDef ymodem;
	ymodem.ymodem_write_byte = drv_com1_write;
	ymodem.ymodem_rx_error_handle = g_ymodem_rx_error_handle;
	ymodem.ymodem_rx_head_handle = g_ymodem_rx_head_handle;
	ymodem.ymodem_rx_data_handle = g_ymodem_rx_data_handle;
	ymodem.ymodem_rx_finish_handle = g_ymodem_rx_finish_handle;
	ymodem.ymodem_tx_data_handle = g_ymodem_tx_data_handle;
	
	
	ymodem_init(&ymodem);

	printf("init\r\n");
	
	FLASH_If_Init();
	ymodem_tx_init(name,sizeof(name),sizeof(file)-1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		dev_comctrl_handle();
		#ifdef USE_RX_MODE
		ymodem_rx_time_handle();
		#endif
		

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
