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
#include <stdio.h>      // add for printf
#include <string.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t uart_buf[1];
uint8_t uart_Morse_Token_index;
uint8_t ticks;  // timer2 ticks
uint8_t MorseMode = 0;
uint8_t SM_State; // State for the State Machine
uint8_t User_B_Pressed = 0;  // flag for button
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* also redefine _write for printf*/
int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}


/* Callback function for UART RX Complete: process incoming character*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(uart_buf[0] == '?'){
    printf("Sw Version %d.%d\r\n",SW_VERSION/10, SW_VERSION%10);
  }
  else{
    switch(uart_Morse_Token_index){
      case 0:
        if(uart_buf[0] == 'M'){
          uart_Morse_Token_index++;
          HAL_UART_Receive_IT(&huart1, uart_buf, 1); // reactivate interrupt for next char
          return;
        }
        break;
      case 1:
        if(uart_buf[0] == 'o'){
          uart_Morse_Token_index++;
          HAL_UART_Receive_IT(&huart1, uart_buf, 1); // reactivate interrupt for next char
          return;
        }
        break;
      case 2:
        if(uart_buf[0] == 'r'){
          uart_Morse_Token_index++;
          HAL_UART_Receive_IT(&huart1, uart_buf, 1); // reactivate interrupt for next char
          return;
        }
        break;
      case 3:
        if(uart_buf[0] == 's'){
          uart_Morse_Token_index++;
          HAL_UART_Receive_IT(&huart1, uart_buf, 1); // reactivate interrupt for next char
          return;
        }
        break;
      case 4:
        if(uart_buf[0] == 'e'){
          uart_Morse_Token_index = 0;
          MorseMode = 1;
          HAL_UART_Receive_IT(&huart1, uart_buf, 1); // reactivate interrupt for next char
          return;
        }
        break;
    }

    uart_Morse_Token_index = 0;
    uart_buf[0]++;  // increment received char to transmit next one
    HAL_UART_Transmit(&huart1, uart_buf, 1, 10);
  }
  
  HAL_UART_Receive_IT(&huart1, uart_buf, 1); // reactivate interrupt for next char
}

void Morse_Code_State_Switcher(){
  // Team code 17 = 3k+2  =>  mesaj = "hello 17"
  // t = 150ms
  const char morse_message[] = ".... . ._.. ._.. ___   .____ __...";
  uint8_t previous_SM_State = SM_State;
  for(int i = 0 ; i < 35 ; i++){
    switch(morse_message[i]){
      case '.':
        SM_State = SM_BLUE;
        HAL_Delay(75);
        SM_State = SM_OFF;
        HAL_Delay(75);
        break;
      case '_':
        SM_State = SM_YELLOW;
        HAL_Delay(375);
        SM_State = SM_OFF;
        HAL_Delay(75);
        break;
      case ' ':
        SM_State = SM_OFF;
        HAL_Delay(150);
        break;
    }
  }
  SM_State = previous_SM_State;
  MorseMode = 0;
}

/* Timer2 Interrupt Service Routine: Blinks LEDs */
// Team code 17 = 3k+2  =>  colors: BLUE, YELLOW, MAGENTA
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2)
{
  if(ticks++ == 10)    // 1 timer2 tick = 1 ms
    ticks=0;   // reset 
  switch (SM_State) {
    case SM_BLUE: 
      HAL_GPIO_WritePin(RED_LED_BANK, RED_LED_PIN, 0);
      HAL_GPIO_WritePin(GREEN_LED_BANK, GREEN_LED_PIN, 0);
      HAL_GPIO_WritePin(BLUE_LED_BANK, BLUE_LED_PIN, 1);
      break; 
    case SM_YELLOW:
      HAL_GPIO_WritePin(RED_LED_BANK, RED_LED_PIN, ticks < 8);
      HAL_GPIO_WritePin(GREEN_LED_BANK, GREEN_LED_PIN, ticks > 4);
      HAL_GPIO_WritePin(BLUE_LED_BANK, BLUE_LED_PIN, 0);
      break; 
    case SM_MAGENTA:
      HAL_GPIO_WritePin(RED_LED_BANK, RED_LED_PIN, ticks < 5);
      HAL_GPIO_WritePin(GREEN_LED_BANK, GREEN_LED_PIN, 0);
      HAL_GPIO_WritePin(BLUE_LED_BANK, BLUE_LED_PIN, ticks > 4);
      break; 
    case SM_OFF:
      HAL_GPIO_WritePin(RED_LED_BANK, RED_LED_PIN, 0);
      HAL_GPIO_WritePin(GREEN_LED_BANK, GREEN_LED_PIN, 0);
      HAL_GPIO_WritePin(BLUE_LED_BANK, BLUE_LED_PIN, 0);
      break;
  }  // case   
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /****************************************************************************/
  /********************************      MAIN        **************************/
  /****************************************************************************/
  
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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, uart_buf, 1); // enable UART Rx IT=interrupt
  HAL_TIM_Base_Start_IT(&htim2);  //enable Timer 2 interrupt

  uart_Morse_Token_index = 0;
  MorseMode = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

  /*********************************** main loop *************************************************/
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET ) {     // PORTC.13 ==0 means button pressed
        HAL_Delay(25);                                                 // wait 25 milliseconds
        if( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET ) { // then read again for debouncing
          while( HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET); // wait for button release
          User_B_Pressed = 1;
          printf("USER\n\r");
        }  
    }
    
    if(MorseMode) Morse_Code_State_Switcher();        // Doing this here so that I don't have to bother with interrupt priorities.
                                                      // It also blocks out the following switch statement, which is ok because I
                                                      // want the USER button to not do anything while it's blinking morse code.

    switch(SM_State) {     // main State Machine
      case SM_START:
        printf("Code version %d.%d starting...\n\r", SW_VERSION/10, SW_VERSION%10);
        SM_State=SM_BLUE;
        break;

      case SM_BLUE:
        if(User_B_Pressed) {
          User_B_Pressed = 0; // reset flag because it was processed
          SM_State = SM_YELLOW;
        }
        break;

      case SM_YELLOW:
        if(User_B_Pressed) {
          User_B_Pressed = 0;
          SM_State = SM_MAGENTA;
        }
        break;
      
      case SM_MAGENTA:
        if(User_B_Pressed) {
          User_B_Pressed = 0;
          SM_State = SM_OFF;
        }
        break;

      case SM_OFF:
        if(User_B_Pressed) {
          User_B_Pressed = 0;
          SM_State = SM_BLUE;
        }
        break;

      default:  // this shouldn't happen
        SM_State = SM_START;        
    } // switch

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
