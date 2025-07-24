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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
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
#define MAX_POINTS 200
#define RX_BUFF_SIZE 500

uint8_t tx_buff[] = {0,1,2,3,4,5,6,7,8,9};
uint8_t rx_buff[RX_BUFF_SIZE] = {0};
uint8_t rx_buff_arm = 0;
uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = {0};

// imu datas
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float temperature = 0;
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
float current_g = 0;

float voltage_to_be_sent = 0;

float avgRPM = 0;
float numPoints = 0;
volatile uint32_t counter = 0;
uint32_t previousCountMillis = 0;
const uint32_t countMillis = 500; // half a second
float rpm = 0;
float sum = 0;

float des_rpm = 0;

bool braking = false;
bool dataNew = false;
volatile bool uart_ready = true;
bool inputState = false;
bool lastInputState = false;

uint8_t led = 0;
uint16_t ledTimer = 0;
uint16_t voltageSent = 0;
uint32_t time_start = 0;
uint32_t time_elapsed = 0;
uint16_t printTimer = 0;

uint32_t profile_ms[MAX_POINTS] = {0};
float profile_g[MAX_POINTS] = {0};
int profile_arm = 0;

//i | idle
//p | has profile (ready)
//m | manual
//u | uploading mode (reading)
//o | on mode (running)
//e | evaluate (aka parsing)
char state = 'i';

int upload_pointer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t setValue(uint16_t value);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void collect_data();
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

	// begins listening for a byte. When byte is recieved, it calls the callback function
	HAL_UART_Receive_IT(&huart2, rx_buff, 1);


	// initialize radio
	nrf24l01p_rx_init(2500, _250kbps);

	// Second, initialize the MCP4725 object:
	myMCP4725 = MCP4725_init(&hi2c1, MCP4725A0_ADDR_A00, 5.0);

	// Check the connection:
	if(MCP4725_isConnected(&myMCP4725)){

		/* Print that the DAC is connected */
		uint8_t success_arr[] = {'D','A','C',' ','g','o','o','d','\n'};
		HAL_UART_Transmit_IT(&huart2, success_arr, 9);
	}
	else{

		/* Print that the DAC is NOT connected */
		uint8_t fail_arr[] = {'b','a','d','\n'};
		HAL_UART_Transmit_IT(&huart2, fail_arr, 4);
	}

	// default centrifuge off
	setValue(0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on

	// set radio address
	// uint8_t radio_status = set_address("2Node");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		inputState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
		// measure frequency of pulse from speed pin
	    if (inputState != lastInputState) {
			counter++;
			lastInputState = inputState;
	    }


		// if imu data is coming in
		if(dataNew){
			dataNew = false;

			// calibrate incoming data using offset
			//			accel_x -= 0.04;
			//			accel_y += 0.01;
			//			accel_z -= 0.05;
			//			gyro_x += 4.7;
			//			gyro_y -= 1.9;
			//			gyro_z += 0.5;
			//
			//			current_g = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));

			// send radio data to serial monitor
			//			char msg[128];
			//			int len = snprintf(msg, sizeof(msg),
			//					"%.2f %.2f %.2f | "
			//					"%.2f %.2f %.2f | "
			//					"%.2fC\n",
			//					accel_x, accel_y, accel_z,
			//					gyro_x, gyro_y, gyro_z,
			//					temperature);
			//
			//			// Send over UART using interrupt
			//			HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, len);
		}

		// evaluation mode (parsing the profile)
		if (state == 'e') {
			int i = upload_pointer; // where the instruction starts in our rx_buffer
			char parseChar = *(rx_buff + i); // grab the first character our rx_buffer
			char numberBuffer[40] = {0}; // create a number buffer to atoi into our ms and g arrays
			int numberBufferArm = 0;     // keep track of the current index
			char evaluateState = 'm'; //m or g
			profile_arm = 0; // current step of the profile

			// our profile data is sandwiched between a start u and a stop u
			while(parseChar != 'u'){
				// parse the first column of the csv (ms)
				if(evaluateState == 'm'){
					// if its a number, store it into numberBuffer
					if((parseChar <= '9' && parseChar >= '0')){
						numberBuffer[numberBufferArm++] = parseChar;

						// if its a comma, flush it all to the ms array and switch to evaluating g
					}else if(parseChar == ','){
						profile_ms[profile_arm] = atoi(numberBuffer);
						memset(numberBuffer, 0, 40);
						numberBufferArm = 0;
						evaluateState = 'g';
					}

					// parse the second column of the csv (g)
				}else if(evaluateState == 'g'){

					// if its a number, store it into numberBuffer
					if((parseChar <= '9' && parseChar >= '0') || parseChar == '.'){
						numberBuffer[numberBufferArm++] = parseChar;

						// upon reaching a new line, flush everything in numberBuffer to g array and go back to m
					}else if(parseChar == '\n'){
						profile_g[profile_arm++] = atof(numberBuffer);
						memset(numberBuffer, 0, 40);
						numberBufferArm = 0;
						evaluateState = 'm';
					}
				}

				// increment our upload arm and make sure it wraps back to the start to grab the next char
				i++;
				i %= RX_BUFF_SIZE;
				parseChar = *(rx_buff + i);
			}

			// reset buffer for clean data
			memset(rx_buff, 0, 500);

			// reached stop "u" so switch to ready mode
			state = 'p';
		}

		// ON state (centrifuge will start spinning)
		else if(state == 'o') {
			time_elapsed = HAL_GetTick() - time_start;
			int index = 0;

			// increment the index of our profile if the elapsed time is greater than the desired runtime
			while(index < profile_arm) {
				if (profile_ms[index] <= time_elapsed) {
					index++;
				}
				else {
					break;
				}
			}

			// we've reached the end of our instructions, so turn everything off and go to idl mode
			if(index == profile_arm){
				state = 'i';
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off

				// run the current profile instruction step
			}else{
				float prev_desired_g = profile_g[index-1];
				float prev_desired_ms = profile_ms[index-1];
				float next_desired_g = profile_g[index];
				float next_desired_ms = profile_ms[index];

				// end case/error detection (next time should never be lower than prev)
				if (next_desired_ms < prev_desired_ms) {
					state = 'i';
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
				}

				// calculate descent slope for braking on sharp changes
				float slope_gps = (next_desired_g - prev_desired_g) / ((next_desired_ms - prev_desired_ms) / 1000);
				float desired_g = ((next_desired_g - prev_desired_g) * (time_elapsed - prev_desired_ms) / (next_desired_ms - prev_desired_ms)) + prev_desired_g;
				float descent_no_brake = -0.368*desired_g + 0.311;

				// brake when motor coasting isnt enough
				if(slope_gps < descent_no_brake) {
					braking = true;
				} else {
					braking = false;
				}

				// when brake mode on, brake if our desired_g is greater than our current_g
				if (braking) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off when in brake mode
					if (desired_g > current_g) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on
					}

					// let the motor coast to reduce speed instead
					else if(desired_g <= current_g) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
					}
					setValue(0);
				}

				// brake mode is off so use voltage to rpm equation to set speed of the centrifuge
				else {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Motor on
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
//					float desired_mps = desired_g * 9.81; // meters per second for the equation
//					float c = (9.81/desired_mps);
//					float a = 1.4; // centrifuge arm radius meters
//					float b = 0; // gondola radius meters (0.25 if gondola exists)
//					float omega = sqrt((desired_mps - 9.81) / (a + (b *sqrt(1-(c*c))))); // equation to get desired omega from desired mps

					float a = 1.4; // centrifuge arm radius meters
					float omega = sqrt( (9.81 * sqrt(desired_g*desired_g - 1)) / a);
					des_rpm = (omega /3.1415) * 30;
					voltage_to_be_sent = des_rpm * 25 * ((5-0.2)/(4000-160)); // equation for the motor that translates rpm to voltage to be sent
					if (voltage_to_be_sent > 5) {
						voltage_to_be_sent = 5;
					}
					int scaled_voltage = (uint16_t)((voltage_to_be_sent / 5) * 4095); // scale the voltage from 0-3.3 to 0-4095 to be sent though the DAC
					setValue(scaled_voltage);

					//				char msg[128];
					//				int len = snprintf(msg, sizeof(msg),
					//						"%u %.2f\n %d",
					//						time_elapsed, desired_g, scaled_voltage);
					//
					//				// Send over UART using interrupt
					//				HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, len);

				}
				printTimer++;
				if (printTimer > 100) {
					printTimer = 0;
					char msg[128];
					int len = snprintf(msg, sizeof(msg),
							"%lu %.2f %.2f %.2f\n",
							time_elapsed, desired_g, current_g, voltage_to_be_sent);

					// Send over UART using interrupt
					HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, len);
				}

//				if ((HAL_GetTick() - previousCountMillis) >= countMillis) {
//					previousCountMillis = HAL_GetTick();
//					collect_data();
//					char msg[128];
//					int len = snprintf(msg, sizeof(msg),
//							"%lu %.2f %.2f %.2f %.2f %.2f\n",
//							time_elapsed, desired_g, current_g, voltage_to_be_sent, rpm, des_rpm);
//
//					// Send over UART using interrupt
//					HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, len);
//				}
			}
		}

		// LED loop to check board status
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

// For serial monitor
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	char recentChar = *(rx_buff + rx_buff_arm); // the most recent character that has not been read yet

	/** fsm for the centrifuge:
	-------------------------------
	 i | idle
	 p | has profile (ready)
	 m | manual
	 u | uploading mode (reading)
	 o | on mode (running)
	 e | evaluate (aka parsing) **/

	switch (state){
	case 'i':
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on
		// go to upload mode
		if(recentChar == 'u'){
			upload_pointer = rx_buff_arm + 1;
			state = 'u';
		}
		// go to manual mode
		else if(recentChar == 'm') {
			state = 'm';
		}
		break;



	case 'm':
		//accept commands from a serial monitor to control the centrifuge
		if(recentChar <= '9' && recentChar >= '0'){
			voltageSent = (uint16_t)((int)(recentChar - '0') / 9.0 * 4095);
			setValue(voltageSent);
		}else if(recentChar == 'C') { // Motor on Brake off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Motor on
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
		}else if(recentChar == 'c') { // Motor off Brake off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
		}else if(recentChar == 'b') { // Motor off Brake on
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on
		}

		// go to idle mode
		else if(recentChar == 'm') {
			state = 'i';
		}
		break;



	case 'p':
		// go to ON mode
		if (recentChar == 'o'){
			time_start = HAL_GetTick();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // Motor on
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //Brake off
			state = 'o';
		}
		// go to upload mode
		else if(recentChar == 'u') {
			upload_pointer = rx_buff_arm + 1;
			state = 'u';
		}
		// go to manual mode
		else if(recentChar == 'm') {
			state = 'm';
		}
		break;



	case 'u':
		// go to evaluation mode
		if(recentChar == 'u'){
			state = 'e';
		}
		break;



	case 'o':
		// go to idle mode
		if(recentChar == 'q') {
			state = 'i';
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on
		}
		break;

	default:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Motor off
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //Brake on
		state = 'i';
	}

	// move to next index in the circular buffer to be read
	rx_buff_arm ++;

	// make sure we stay within the buffer
	if(rx_buff_arm >= RX_BUFF_SIZE){
		rx_buff_arm = 0;
	}

	// flush our command history through UART
	//	HAL_UART_Transmit_IT(&huart2, rx_buff, RX_BUFF_SIZE);

	//	uint8_t char_arr[1] = {recentChar};
	//	HAL_UART_Transmit_IT(&huart2, char_arr, 1);

	// necessary to prime the next callback
	HAL_UART_Receive_IT(&huart2, rx_buff + rx_buff_arm, 1); // the next character will be stored in the next index
}

// for DAC
uint8_t setValue(uint16_t value){
	return MCP4725_setValue(&myMCP4725, value, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);
}

// for radio recieve
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER) {
		nrf24l01p_rx_receive(rx_data); // read data when data ready flag is set
		accel_x = (int16_t)(rx_data[0] | (rx_data[1] << 8)) / 2048.f;
		accel_y = (int16_t)(rx_data[2] | (rx_data[3] << 8)) / 2048.f;
		accel_z = (int16_t)(rx_data[4] | (rx_data[5] << 8)) / 2048.f;
		temperature = (float)((int16_t)(rx_data[6] | (rx_data[7] << 8))) / 340 + 36.53;
		gyro_x = (int16_t)(rx_data[8] | (rx_data[9] << 8)) / 65.5f;
		gyro_y = (int16_t)(rx_data[10] | (rx_data[11] << 8)) / 65.5f;
		gyro_z = (int16_t)(rx_data[12] | (rx_data[13] << 8)) / 65.5f;

		accel_x -= 0.04;
		accel_y += 0.01;
		accel_z -= 0.05;
		gyro_x += 4.7;
		gyro_y -= 1.9;
		gyro_z += 0.5;

		current_g = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));
		dataNew = true;
	}
}

// grabs data from the motor and averages it
void collect_data() {
	rpm = (counter * 20) / 4.0;
	sum = avgRPM * numPoints;
	sum += rpm;
	numPoints++;
	avgRPM = sum/numPoints;

	// reset count for next sample
	counter = 0;
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
