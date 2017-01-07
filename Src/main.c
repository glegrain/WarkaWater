/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @version V1.7.0
  * @date    31-May-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dht.h"
#include "dewpoint.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void SystemClock_Config32MHz(void);
static void Error_Handler(void);
static void LED_Init(void);
static void PB_Init(ButtonMode_TypeDef ButtonMode);

/* Private functions ---------------------------------------------------------*/
// void putc(int p, FILE *)
void _ttywrch(int ch) {
  // HAL_UART_Transmit_IT(&UartHandle, (uint8_t*) &ch, 1);
  HAL_UART_Transmit(&UartHandle, (uint8_t *) &ch, 1, 200);
  // return ch;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32L0xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to have a frequency of 2 MHz (Up to 32MHZ possible) */
  SystemClock_Config();

  /* Configure LED2 */
  LED_Init();

  /* Configure Button B1 */
  PB_Init(BUTTON_MODE_EXTI);

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART2 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USART2;
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure DHT11 Humidity & Temperature sensor */
  DHT_Init();

  DHT_ValuesTypeDef sensorValues;
  double dewpoint;

  /* Infinite loop */
  while (1)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    HAL_Delay(1000);
    DHT_StatusTypeDef dht_status = DHT_ReadSensor(&sensorValues);
    if (dht_status != DHT_OK) {
      printf("DHT Error: %d\n", dht_status);
    } else {
      printf("Sensor values: %d.%d%%, %d.%dºC\n",
          sensorValues.RelativeHumidityIntegral,
          sensorValues.RelativeHumidityFractional,
          sensorValues.TemperatureIntegral,
          sensorValues.TemperatureFractional);
      dewpoint = dewPoint((double) sensorValues.TemperatureIntegral,
                          (double) sensorValues.RelativeHumidityIntegral);
      printf("dewpoint: %dºC\n", (int) dewpoint);
    }
    // fputc("c", 1);
    // char toto[10] = "chocola\n";
    // HAL_UART_Transmit(&UartHandle, (uint8_t *) toto, 9, 200);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4194304
  *            HCLK(Hz)                       = 4194304
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Flash Latency(WS)              = 0
  *            Main regulator output voltage  = Scale3 mode
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6; /* MSI = 4.194 MHz */
  RCC_OscInitStruct.MSICalibrationValue = 0x00;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }


  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType   = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_MUL                        = 4
  *            PLL_DIV                        = 2
  *            Flash Latency(WS)              = 1
  *            Main regulator output voltage  = Scale1 mode
  * @param  None
  * @retval None
  */
static void SystemClock_Config32MHz(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);  
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  Configures LED2 GPIO.
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the GPIO_LED Clock */
  LED2_GPIO_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin   = LED2_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void PB_Init(ButtonMode_TypeDef ButtonMode)
{
   GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = USER_BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStruct);
  }

  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = USER_BUTTON_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_SetPriority((IRQn_Type)(USER_BUTTON_EXTI_IRQn), 0x03);
    HAL_NVIC_EnableIRQ((IRQn_Type)((USER_BUTTON_EXTI_IRQn)));
  }
}


void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/

  /* Enable GPIO TX/RX clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Enable USART2 clock */
  __HAL_RCC_USART2_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/

  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

