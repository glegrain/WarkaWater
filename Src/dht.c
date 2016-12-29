/**
  ******************************************************************************
  * @file    Src/dht.c
  * @author  Guillaume Legrain
  * @version V0.1.0
  * @date    26-December-2016
  * @brief   This file includes the DHT11 sensor driver.
  ******************************************************************************
  */
 
/* Includes ------------------------------------------------------------------*/
#include "dht.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DHT_WAKEUP_DELAY  18
#define DHT_LEADING_ZEROS 1

#define GPIO_MODE             ((uint32_t)0x00000003U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef      TimHandle;
TIM_HandleTypeDef      htim;

/* Timer Input Capture Configuration Structure declaration */
TIM_IC_InitTypeDef     sICConfig;

/* Capture index */
uint16_t               uhCaptureIndex = 0;

uint16_t buffer[42];
uint16_t g_i;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void GPIO_ConfigureMode(GPIO_TypeDef  *GPIOx, uint32_t pin, uint32_t mode)
{
  uint32_t temp = 0x00U;
  uint32_t position = log2(pin);
  /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
  temp = GPIOx->MODER;
  temp &= ~(GPIO_MODER_MODE0 << (position * 2U));
  temp |= ((mode & GPIO_MODE) << (position * 2U));
  GPIOx->MODER = temp;
}

// DHT11 Humidity and Temperature communication
// Serial Inteface (Single-Wire Two-Way) up to 20 meters
// Pin 1 --> VDD (3-5.5V DC)
// Pin 2 --> 5k pull-up --> MCU (choose pull-up value according to distance)
// Pin 3 --> GND
// One 100nF decoupling cap can be added between VDD and GND

void DHT_GPIO_Init(void)
{
  // GND
  // D13 -- PA5
  // D12 -- PA6
  // D11 -- PA7
  /* Enable GPIO DHT_VDD and DHT_DATA clock */
  DHT_CLK_ENABLE();

  /* Configure DATA pin */
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin   = DHT_DATA_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP; // ??
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM2; // TODO/checkme
  HAL_GPIO_Init(DHT_DATA_GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_PIN_SET);

  // /* Timer init */
  // __HAL_RCC_TIM2_CLK_ENABLE();
  // htim.Instance           = TIM2;
  // htim.Init.Prescaler     = (SystemCoreClock / 1000000) - 1;
  // htim.Init.CounterMode   = TIM_COUNTERMODE_UP;
  // htim.Init.Period        = 0xFFFF;
  // htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  // htim.Channel            = TIM_CHANNEL_1;
  // if (HAL_TIM_Base_Init(&htim) != HAL_OK) {
  //   while (1);
  // }
  // // if (HAL_TIM_Base_Start(&htim) != HAL_OK) {
  // //   while (1);
  // // }
  // // while(1) {
  // //   uint32_t start = __HAL_TIM_GET_COUNTER(&htim);
  // //   HAL_Delay(10); 
  // //   uint32_t stop = __HAL_TIM_GET_COUNTER(&htim);
  // //   printf("time diff: %u\n", stop - start);
  // // }
  
  /*
   * Input Capture Timer init
   */
  /*##-1- Configure the TIM peripheral #######################################*/
  /* Set TIMx instance */
  TimHandle.Instance = TIM2;
   
  /* Initialize TIMx peripheral as follow:
       + Period = 0xFFFF
       + Prescaler = 0
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period        = 0xFFFF;
  TimHandle.Init.Prescaler     = (SystemCoreClock / 1000000) - 1;
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;  
  if(HAL_TIM_IC_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }  
  /*##-2- Configure the Input Capture channel ################################*/ 
  /* Configure the Input Capture of channel 2 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE /* TIM_ICPOLARITY_RISING */;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 0;   
  if(HAL_TIM_IC_ConfigChannel(&TimHandle, &sICConfig, TIM_CHANNEL_2) != HAL_OK)
 {
    /* Configuration Error */
    while(1);
  }

}

/**
  * @brief  DeInitialize the RTC peripheral.
  * @param  hrtc: RTC handle
  * @note   This function doesn't reset the RTC Backup Data registers.
  * @retval HAL status
  */
DHT_StatusTypeDef DHT_ReadSensor(DHT_ValuesTypeDef *values)
{
  // One communication process is about 4ms
  // Data format 40-bit:
  // [39:32] 8-bit integer RH data
  // [31:24] 8-bit fractional RH data
  // [23:16] 8-bit integer T data
  // [15: 8] 8-bit fractional T data
  // [ 7: 0] 8-bit Checksum
  // 
  // Read:
  // Step 1: Power-on -  Wait 1 s ofter DHT11 power-on
  // Step 2: Request - MCU Output low (> 18ms),
  //         then switch back GPIO to Input mode.
  //         Data is pulled-up (> 40 us) and MCU waits for reponse.
  // Step 3: DHT sends a low signal (~54us to 80us)
  //         DHT sends a high signal (80us)
  // Step 4: Receive - 
  //         0b0: ~54us low followed by ~24us high
  //         0b1: ~54us low followed by ~70us high
  // Step 5: End Of Frame: ~54us low, then bus is pulled back to high and DHT
  //         goes to sleep.

  /* Request sample - send a start signal */
  // NOTE: pin nb hardcoded to pin 6
  // 0xFFFFCFFFF: MODE6 Input 
  // 0xFFFFDFFFF: MODE6 GPOutput
  // uint16_t buffer[41];
  uint16_t counter;
  g_i = 41;
  /* Configure pin to output */
  // DHT_DATA_GPIO_PORT->MODER &= 0xFFFFDFFF;
  GPIO_ConfigureMode(DHT_DATA_GPIO_PORT, GPIO_PIN_1, GPIO_MODE_OUTPUT_PP);
  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(DHT_WAKEUP_DELAY);
  // HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, GPIO_PIN_1, GPIO_PIN_SET);
  /* Configure pin to input */
  // DHT_DATA_GPIO_PORT->MODER &= 0xFFFFCFFF; // function call is too slow with -O0
  // GPIO_ConfigureMode(DHT_DATA_GPIO_PORT, GPIO_PIN_1, GPIO_MODE_AF_PP);
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    while(1);
  }

  // /* Sample DATA using TIMER input capture to measure each pulse width */
  // TODO
  // wait for buffer to be filled
  while (g_i < 42 );
  
  // TIMER test code
  // if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK) {
  //   while (1);
  // }
  // while(g_i >= 0) {
  //   // __HAL_TIM_SET_COUNTER(&TimHandle, 0x00000U);
  //   // uint32_t start = __HAL_TIM_GET_COUNTER(&TimHandle);
  //   HAL_Delay(500); 
  //   // uint32_t stop = __HAL_TIM_GET_COUNTER(&TimHandle);
  //   printf("running: g_i  = %u\n", g_i);
  // }

  // /*
  //  * Sample data with TIM measurments
  //  */
  // /* Start TIM */
  // if (HAL_TIM_Base_Start(&htim) != HAL_OK) {
  //   while (1);
  // }
  // for (int i = 40; i >=0; i--) {
  //   /* wait for DATA to go high */
  //   while ((DHT_DATA_GPIO_PORT->IDR & DHT_DATA_PIN) == (uint32_t)GPIO_PIN_RESET);
  //   /* measure high time */
  //   __HAL_TIM_SET_COUNTER(&htim, 0x00000U);
  //   while ((DHT_DATA_GPIO_PORT->IDR & DHT_DATA_PIN) != (uint32_t)GPIO_PIN_RESET) {
  //     // if (__HAL_TIM_GET_COUNTER(&htim) > 1000) return DHT_TIMEOUT_ERROR;
  //   }
  //   // store timer value in buffer for debug
  //   buffer[i] = __HAL_TIM_GET_COUNTER(&htim);
  // }




  /* Sample DATA from sensor */
  // NOTE: raw register read is faster than HAL call
  // TODO: find minimum sampling rate
  // TODO: use a timer instead of a counter
  // 1. wait for data to go high
  // 2. mesure high time.
  // if high time > 24us --> 0b1
  // else if high time < 70us --> 0b0
  // for (int i = 40; i >=0; i--) {
  //   /* wait for DATA to go high */
  //   while ((DHT_DATA_GPIO_PORT->IDR & DHT_DATA_PIN) == (uint32_t)GPIO_PIN_RESET);
  //   /* measure high time */
  //   counter = 0;
  //   while ((DHT_DATA_GPIO_PORT->IDR & DHT_DATA_PIN) != (uint32_t)GPIO_PIN_RESET) {
  //     counter++;
  //     if (counter > 50) return DHT_TIMEOUT_ERROR;
  //   }
  //   // store counter value in buffer for debug
  //   buffer[i] = counter;
  // }
  
  /* Read sampled data */
  /* Array is converted into 5 8-bit variables */
  uint8_t RH_HByte = 0;
  uint8_t RH_LByte = 0;
  uint8_t T_HByte  = 0;
  uint8_t T_LByte  = 0;
  uint8_t checksum = 0;

  #define TIME_LIMIT 48 /* midpoint between 72us and 24us */
  // #define TIME_LIMIT 4 /* Found with trial and error */
  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+32] > TIME_LIMIT) bit = 1;
    RH_HByte += bit << i; // Shift bit is equivalent to power of 2
  }

  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+24] > TIME_LIMIT) bit = 1;
    RH_LByte += bit << i;
  }

  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+16] > TIME_LIMIT) bit = 1;
    T_HByte += bit << i;
  }

    for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+8] > TIME_LIMIT) bit = 1;
    T_LByte += bit << i;
  }

  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i] > TIME_LIMIT) bit = 1;
    checksum += bit << i;
  }

  /* Store sensor values to destination structure */
  values->RelativeHumidityIntegral = RH_HByte;
  values->RelativeHumidityFractional = RH_LByte;
  values->TemperatureIntegral = T_HByte;
  values->TemperatureFractional = T_LByte;

  /* Verify received data */
  if (checksum != ((RH_HByte + RH_LByte + T_HByte + T_LByte) & 0xFF)) {
    return DHT_CHECKSUM_ERROR;
  }

  return DHT_OK;

}


/**
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim : TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    if(uhCaptureIndex == 1)
     {
       // /* Get the 1st Input Capture value */
       // uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
       /* Reset timer counter on 1st Input Capture */
       __HAL_TIM_SET_COUNTER(&TimHandle, 0x00000U);
       uhCaptureIndex = 0;
    }
    else
    {
        /* Store measured time on 2nd Input Capture */
        if (g_i >= 0 && g_i < 42)
            buffer[g_i] = __HAL_TIM_GET_COUNTER(&TimHandle);
        g_i--;
        uhCaptureIndex = 1;
        // printf("CAPTURE: %u\n", g_buffer[g_i - 1]);
    }
    // while(1);
  //   else if(uhCaptureIndex == 1)
  //   {
  //     /* Get the 2nd Input Capture value */
  //     uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); 
      
  //     /* Capture computation */
  //     if (uwIC2Value2 > uwIC2Value1)
  //     {
  //       uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
  //     }
  //     else if (uwIC2Value2 < uwIC2Value1)
  //     {
  //       uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2) + 1;
  //     }
  //     else
  //     {
  //       uwDiffCapture = 0;
  //     }
  //     /* uwFrequency computation
  //     TIM2 counter clock = RCC_Clocks.HCLK_Frequency */      
  //     uwFrequency = HAL_RCC_GetHCLKFreq()/ (uwDiffCapture + 1);
  //     uhCaptureIndex = 0;
  //   }
    
  }
}