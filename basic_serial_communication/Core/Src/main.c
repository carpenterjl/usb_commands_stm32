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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h" 	//Used for USB communication
#include "string.h" 		//Used for string formating
#include "ctype.h"			//Used to compare strings
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
uint8_t usbBuf[64]; 			//Rx Buffer
char dataToSend[64]; 			//Tx Buffer
uint8_t dataLength; 			//length of Tx Buffer
char cmdIn[64]; 				//Reading command from Rx Buffer
char current_cmd[64];			//Store Current Command
int bt1, bt2, bt3; 				//Button status
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void readCommand(void);			//Read usb buffer and check if it is a valid command
int checkButton(int button);	//Check button status, returns 1 if pressed, 0 if not pressed
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  memset(cmdIn, '\0', 64); 						// initialize buffers
  memset(current_cmd, '\0', 64);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  readCommand(); 							// read command from USB buffer and process
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); 	// Blink board LED
	  HAL_Delay(300);							// small delay
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BT1_Pin BT2_Pin BT3_Pin */
  GPIO_InitStruct.Pin = BT1_Pin|BT2_Pin|BT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void readCommand()
{


	char cmd_led1[] = "led1",														//Setting valid command arguments
		 cmd_led2[] = "led2",
		 cmd_led3[] = "led3",
		 cmd_all_leds[] = "all leds",
		 cmd_bt1[] = "button1",
		 cmd_bt2[] = "button2",
		 cmd_bt3[] = "button3",
		 cmd_all_buttons[] = "all buttons",
		 cmd_help[] = "help";


	if(cmdIn[0] != usbBuf[0])
	{

	for(int i = 0; i < 63; i++){ 													//Copies USB buffer to end of input and sets lower case
		cmdIn[i] = tolower(usbBuf[i]);
	}



	strcpy(current_cmd, cmdIn);														//Sets current command as new incoming command

	if(strcmp(current_cmd, cmd_led1) == 0)											//Check for valid command
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		dataLength = snprintf(dataToSend, 64, "LED 1 toggled!\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

	}else if(strcmp(current_cmd, cmd_led2) == 0)
	{
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		dataLength = snprintf(dataToSend, 64, "LED 2 toggled!\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

	}else if(strcmp(current_cmd, cmd_led3) == 0)
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		dataLength = snprintf(dataToSend, 64, "LED 3 toggled!\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

	}else if(strcmp(current_cmd, cmd_all_leds) == 0)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		dataLength = snprintf(dataToSend, 64, "All leds toggled!\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

	}else if(strcmp(current_cmd, cmd_bt1) == 0)
	{
		bt1 = checkButton(1);
		if(bt1 == 0)
		{
			dataLength = snprintf(dataToSend, 64, "Button 1 Status: Off\n");
		}else if(bt1 == 1)
		{
			dataLength = snprintf(dataToSend, 64, "Button 1 Status: On\n");
		}
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

	}else if(strcmp(current_cmd, cmd_bt2) == 0)
	{
		bt2 = checkButton(2);
		if(bt2 == 0)
		{
			dataLength = snprintf(dataToSend, 64, "Button 2 Status: Off\n");
		}else if(bt2 == 1)
		{
			dataLength = snprintf(dataToSend, 64, "Button 2 Status: On\n");
		}
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

	}else if(strcmp(current_cmd, cmd_bt3)== 0)
	{
		bt3 = checkButton(3);
		if(bt3 == 0)
		{
			dataLength = snprintf(dataToSend, 64, "Button 3 Status: Off\n");
		}else if(bt3 == 1)
		{
			dataLength = snprintf(dataToSend, 64, "Button 3 Status: On\n");
		}
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

	}else if(strcmp(current_cmd, cmd_all_buttons) == 0)
	{
		bt1 = checkButton(1);
		if(bt1 == 0)
		{
			dataLength = snprintf(dataToSend, 64, "Button 1 Status: Off\n");
		}else if(bt1 == 1)
		{
			dataLength = snprintf(dataToSend, 64, "Button 1 Status: On\n");
		}
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

		bt2 = checkButton(2);
		if(bt2 == 0)
		{
			dataLength = snprintf(dataToSend, 64, "Button 2 Status: Off\n");
		}else if(bt2 == 1)
		{
			dataLength = snprintf(dataToSend, 64, "Button 2 Status: On\n");
		}
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);

		bt3 = checkButton(3);
		if(bt3 == 0)
		{
			dataLength = snprintf(dataToSend, 64, "Button 3 Status: Off\n");
		}else if(bt3 == 1)
		{
			dataLength = snprintf(dataToSend, 64, "Button 3 Status: On\n");
		}
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);
	}else if(strcmp(current_cmd, cmd_help) == 0)
	{
		dataLength = snprintf(dataToSend, 64, "Available Commands:\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);
		dataLength = snprintf(dataToSend, 64, "led1, led2, led3\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);
		dataLength = snprintf(dataToSend, 64, "button1, button2, button3\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);
		dataLength = snprintf(dataToSend, 64, "all leds, all buttons\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);
	}else
	{
		dataLength = snprintf(dataToSend, 64, "Invalid Command! Type \'help\'\n");
		CDC_Transmit_FS((uint8_t *)dataToSend, dataLength);
		HAL_Delay(100);
	}
	memset(cmdIn, '\0', 64);														//Clear Incoming Command
	memset(usbBuf, '\0', 64);														//Clear USB Rx Buffer
	memset(current_cmd, '\0', 64);													//Clear Current command
	}

}

int checkButton(int button) 														//Returns button status if a valid choice is requested, otherwise returns 255 as error val
{
	switch(button)
	{
	case 1:
		return !HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin);
	case 2:
		return !HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin);
	case 3:
		return !HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin);
	default:
		return 255;
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
