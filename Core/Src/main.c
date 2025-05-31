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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MCP4725.h"
#include "nrf24l01p.h"
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
uint8_t tx_buff[] = {0,1,2,3,4,5,6,7,8,9};
uint8_t rx_buff[1] = {0};
uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = {0};



uint8_t led = 0;
uint16_t ledTimer = 0;
uint16_t voltageSent = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t setValue(uint16_t value);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// First, create an MCP4725 object:
MCP4725 myMCP4725;

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, rx_buff, 1);


  // initialize radio
  nrf24l01p_rx_init(2500, _1Mbps);

  // Second, initialize the MCP4725 object:
  myMCP4725 = MCP4725_init(&hi2c1, MCP4725A0_ADDR_A00, 3.30);

  // Check the connection:
  if(MCP4725_isConnected(&myMCP4725)){

	  /* Print that the DAC is connected */
	  uint8_t success_arr[] = {'g','o','o','d'};
	  HAL_UART_Transmit_IT(&huart2, success_arr, 4);
  }
  else{

	 /* Print that the DAC is NOT connected */
		uint8_t fail_arr[] = {'b','a','d'};
		HAL_UART_Transmit_IT(&huart2, fail_arr, 3);

}

	// default off
	setValue(0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on

	// set radio address
//	uint8_t radio_status = set_address("2Node");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  HAL_UART_Transmit_IT(&huart2, rx_data, 1);


//	  HAL_UART_Receive_IT(&huart1, rx_buff, 1);
//
//	  if (rx_buff[0] == 'a') {
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		  HAL_Delay(50);
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		  HAL_Delay(50);
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		  HAL_Delay(50);
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		  HAL_Delay(50);
//	  }
//
//
//	  else if (rx_buff[0] == 'b') {
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		  HAL_Delay(300);
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		  HAL_Delay(300);
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//		  HAL_Delay(300);
//		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//		  HAL_Delay(300);
//	  }



	  if(rx_buff[0] != 0){
		  if(rx_buff[0] <= '9' && rx_buff[0] >= '0'){
			  voltageSent = (uint16_t)((int)(rx_buff[0] - '0') / 9.0 * 4095);
			  setValue(voltageSent);

		  }else if(rx_buff[0] == 'M') { // Motor on Brake off
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Motor on
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
		  }else if(rx_buff[0] == 'm') { // Motor off Brake off
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
		  }else if(rx_buff[0] == 'b') { // Motor off Brake on
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on
		  }else if(rx_buff[0] == 'd') { // sending voltage
			  setValue(2048); // half of 3.3v
		  }else if(rx_buff[0] == 'e') { // sending voltage
			  setValue(0); // a fourth of 3.3v
		  }else if(rx_buff[0] == 'f') { // sending voltage
			  setValue(4095); // a fourth of 3.3v
		  }


		  rx_buff[0] = 0;
	  }

	  ledTimer ++;
	  if(ledTimer > 200){
		  ledTimer = 0;
		  led = !led;
		  if(led){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		  }else{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  }
	  }
	  HAL_Delay(1);
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

/* USER CODE BEGIN 4 */
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		  HAL_UART_Receive_IT(&huart2, rx_buff, 1);
		  HAL_UART_Transmit_IT(&huart2, rx_buff, 1);
	}

	uint8_t setValue(uint16_t value){
		return MCP4725_setValue(&myMCP4725, value, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
	}

	// for radio recieve
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER) {
		    nrf24l01p_rx_receive(rx_data); // read data when data ready flag is set
		    uint8_t success_arr[] = {'y','e','s'};
		    HAL_UART_Transmit_IT(&huart2, success_arr, 3);
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
