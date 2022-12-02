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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint8_t AdvancedButton(GPIO_TypeDef *port, uint16_t pin, uint8_t active_state, uint16_t time_ms_short, uint16_t time_ms_long);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
// Define macro to read pin state
#define TM_GPIO_GetOutputPinValue(GPIOA, GPIO_Pin)	(((GPIOA)->ODR & (GPIO_Pin)) == 0 ? 0 : 1);
#define TM_GPIO_GetInputPinValue(GPIOE, GPIO_Pin)   (((GPIOE)->IDR & (GPIO_Pin)) == 0 ? 0 : 1));
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t *pclock = (uint32_t *)0x40023830;        //(0x40023800 + 0x30)
	uint32_t *pmodeA =(uint32_t *)0x40020000;      //(0x40020000 + 0x00)
	uint32_t *ppinA =(uint32_t *)0x40020014;         //(0x40020000 + 0x10)
	uint32_t *pmodeE =(uint32_t *)0x40021000;      //(0x40021000 + 0x00)
	//uint32_t *ppinE =(uint32_t *)0x40021010;          //(0x40021000 + 0x10)

	*pclock |= 0x00000001;
    //*pclock |= 0x00000010; ravi

	   // Mode A pin 6,7 output mode enable

	   // *pmodeE &= ~(0x3<< 2); // Mode E pin 4 input mode enable

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	// a. clear
	/*  	*pmodeA &= 0xffffcfff;
	  	// b. set
	  	*pmodeA |= 0xffffc;
*/
  *pmodeA |= (0x55 << 3);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER COD E BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
 // *pmodeE &= 0xfffffcff;
  *pmodeE &= ~(3<<2);
  //*pmodeE |= 0xfffff3ff;
  /* USER CODE END 2 */
  *ppinE |= (1<<4);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t pressed = AdvancedButton(pmodeE, *ppinE, 1, 30, 3000);

		if (pressed == 1) {
		//	printf("%s\r\n", "Short Press");
			*ppinA &= 0xffbf;
		}else if (pressed == 2) {
		//	printf("%s\r\n", "Long Press");
			*ppinA &= 0xff7f;
		}
  }

}

uint8_t AdvancedButton(GPIO_TypeDef *port, uint16_t pin, uint8_t active_state, uint16_t time_ms_short, uint16_t time_ms_long)
{

	uint8_t result = 0;
	uint16_t time;
	uint32_t last_val;

	time = time_ms_long - time_ms_short;
	last_val = millis();

	if (TM_GPIO_GetInputPinValue(GPIOA, GPIO_PIN_6) == active_state)
	{
		result = 1;
		delay_ms(time_ms_short);	// Debounce Time

		while (TM_GPIO_GetInputPinValue(port, pin) == active_state)
		{
			if (millis() - last_val >= time)
			{
				result = 2;
				//
				// Make a beep or something here
				//
				while (TM_GPIO_GetInputPinValue(port, pin) == active_state){}
				break;
			}
		}
	}
	delay_ms(100);	// Short delay to counter debounce on release
	return result;
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_D2_Pin|LED_D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D2_Pin LED_D3_Pin */
  GPIO_InitStruct.Pin = LED_D2_Pin|LED_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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


