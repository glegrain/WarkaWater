/**
  ******************************************************************************
  * @file    Src/dht.c
  * @author  Guillaume Legrain
  * @version V0.1.2
  * @date    26-December-2016
  * @brief   This file includes the DHT11 sensor driver.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dht.h"
#include "stm32l0xx_ll_gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DHT_WAKEUP_DELAY  18
#define DHT_LEADING_ZEROS 1
#define TIME_LIMIT        48 /* midpoint between 72us and 24us */
#define BUFF_SIZE         42
#define DHT_TIMEOUT       0x1FFF /* SysClock and Code optimization dependent */

#define GPIO_MODE             ((uint32_t)0x00000003U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef      TimHandle;

/* Timer Input Capture Configuration Structure declaration */
TIM_IC_InitTypeDef     sICConfig;

/* Capture index */
uint16_t               captureEdge = 0;

uint16_t buffer[BUFF_SIZE];
uint16_t bufferIndex;

/* Private function prototypes -----------------------------------------------*/
static DHT_StatusTypeDef decodeBuffer(DHT_ValuesTypeDef *values,
                                      uint16_t *buffer, uint8_t size);

/* Private functions ---------------------------------------------------------*/
void GPIO_ConfigureMode(GPIO_TypeDef  *GPIOx, uint32_t Pin, uint32_t Mode)
{
  MODIFY_REG(GPIOx->MODER, ((Pin * Pin) * GPIO_MODER_MODE0), ((Pin * Pin) * Mode));
}

// DHT11 Humidity and Temperature communication
// Serial Interface (Single-Wire Two-Way) up to 20 meters
// Pin 1 --> VDD (3-5.5V DC)
// Pin 2 --> 5k pull-up --> MCU (choose pull-up value according to distance)
// Pin 3 --> GND
// One 100nF decoupling cap can be added between VDD and GND

void DHT_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIM2 Peripheral clock enable */
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* Enable GPIO channels Clock */
  DHT_GPIO_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* Configure (TIM2_Channel2) in Alternate function, push-pull and High speed */
  GPIO_InitStruct.Pin       = DHT_DATA_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = DHT_DATA_AF;
  HAL_GPIO_Init(DHT_DATA_GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_PIN_SET);

  /*##-3- Configure the TIM peripheral #######################################*/
  /* Set TIMx instance */
  TimHandle.Instance = TIM2;

  /* Initialize TIMx peripheral to count every 1us */
  TimHandle.Init.Period        = 0xFFFF;
  TimHandle.Init.Prescaler     = (SystemCoreClock / 1000000) - 1;
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /*##-4- Configure the Input Capture channel ################################*/
  /* Configure the Input Capture of channel 2 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&TimHandle, &sICConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    while(1);
  }

}

void DHT_Init_IT(void)
{
  DHT_Init();
  /*##- Configure the NVIC for TIMx ##########################################*/
  /* Set the TIM2 global Interrupt */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);

  /* Enable the TIM2 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void DHT_Init_DMA(void)
{
  static DMA_HandleTypeDef  hdma_tim;

  /* Enable DMA1 clock */
  __HAL_RCC_DMA1_CLK_ENABLE();

  DHT_Init();

  /*##- Configure the DMA stream #############################################*/
  /* Set the parameters to be configured */
  /*  - Memory address is automatically incremented by 16-bit
   *  - Circular mode: after the last transfer, the DMA_CNDTRx register is
   *     automatically reloaded with the initially programmed value.
   */
  hdma_tim.Init.Request             = DMA_REQUEST_8;
  hdma_tim.Instance                 = DMA1_Channel3;
  hdma_tim.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_tim.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tim.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tim.Init.MemDataAlignment    = DMA_PDATAALIGN_HALFWORD;
  hdma_tim.Init.Mode                = DMA_CIRCULAR;
  hdma_tim.Init.Priority            = DMA_PRIORITY_HIGH;

  // NOTE: Channel 2 or 3?
  /* Link hdma_tim to hdma[TIM_DMA_ID_CC2] (channel2) */
  __HAL_LINKDMA(&TimHandle, hdma[TIM_DMA_ID_CC2], hdma_tim);

  // HAL_DMA_Init(htim->hdma[TIM_DMA_ID_CC2]);

  /*##- Configure the NVIC for DMA ###########################################*/
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/**
  * @brief  Acquire measurements from DHT sensor.
  * @param  values: Sensor values destination
  * @retval DHT status
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
  // Step 1: Power-on -  Wait 1s after DHT11 power-on
  // Step 2: Request - MCU Output low (> 18ms),
  //         then switch back GPIO to Input mode.
  //         Data is pulled-up (> 40 us) and MCU waits for response.
  // Step 3: DHT sends a low signal (~54us to 80us)
  //         DHT sends a high signal (80us)
  // Step 4: Receive -
  //         0b0: ~54us low followed by ~24us high
  //         0b1: ~54us low followed by ~70us high
  // Step 5: End Of Frame: ~54us low, then bus is pulled back to high and DHT
  //         goes to sleep.

  uint16_t timeoutCounter = 0;
  bufferIndex = BUFF_SIZE - 1;

  /* Configure pin to output */
  LL_GPIO_SetPinMode(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_MODE_OUTPUT_PP);

  /* Start the Input Capture in polling mode */
  HAL_TIM_IC_Start(&TimHandle, TIM_CHANNEL_2);

  /* Send request signal */
  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_PIN_RESET);
  HAL_Delay(DHT_WAKEUP_DELAY);
  LL_GPIO_SetOutputPin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN);

  /* Configure pin for Input Capture */
  // NOTE: Requires code optimization to use LL Drivers or a SysClock > 4.2MHz
  // LL_GPIO_SetPinMode(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, LL_GPIO_MODE_ALTERNATE);
  // LL_GPIO_SetAFPin_0_7(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, LL_GPIO_AF_2);
  DHT_DATA_GPIO_PORT->MODER = 0xEBFFD4AB; // Pin1 to Alternate Function Mode
  DHT_DATA_GPIO_PORT->AFR[0] = 0x00004420; // Pin1 to AF2

  /* Sample DATA using TIMER input capture to measure each pulse width */
  while (bufferIndex < BUFF_SIZE) {
    // TODO: Add timeout
    if (TIM2->SR & TIM_SR_CC2IF) {
      if (captureEdge == 0) {
        buffer[bufferIndex] = TIM2->CNT;
        TIM2->SR = 0U; // Reset TIM2 FLAGS
        // __HAL_TIM_CLEAR_FLAG(&TimHandle, TIM_FLAG_CC2);
        bufferIndex--;
        captureEdge = 1;
      } else {
        TIM2->SR = 0U;  // Reset TIM2 FLAGS
        // __HAL_TIM_CLEAR_FLAG(&TimHandle, TIM_FLAG_CC2);
        __HAL_TIM_SET_COUNTER(&TimHandle, 0U);
        captureEdge = 0;
      }
    }
    timeoutCounter++;
    if (timeoutCounter > DHT_TIMEOUT)
      return DHT_TIMEOUT_ERROR;
  }
  HAL_TIM_IC_Stop(&TimHandle, TIM_CHANNEL_2);

  return decodeBuffer(values, buffer, BUFF_SIZE);

}

/**
  * @brief  Acquire measurements from DHT sensor in interrupt mode.
  * @note   Code execution speed is more demanding than in polling mode.
  * @param  values: Sensor values destination
  * @retval DHT status
  */
DHT_StatusTypeDef DHT_ReadSensor_IT(DHT_ValuesTypeDef *values)
{
  bufferIndex = BUFF_SIZE - 1;
  captureEdge = 1;

  /* Configure pin to output */
  LL_GPIO_SetPinMode(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, LL_GPIO_MODE_OUTPUT);

  /* Start the Input Capture in interrupt mode */
  HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_2);

  /* Send request signal */
  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_PIN_RESET);
  HAL_Delay(DHT_WAKEUP_DELAY);
  LL_GPIO_SetOutputPin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN);

  /* Configure pin for Input Capture */
  // NOTE: Requires code optimization to use LL Drivers or a SysClock > 4.2MHz
  // LL_GPIO_SetPinMode(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, LL_GPIO_MODE_ALTERNATE);
  // LL_GPIO_SetAFPin_0_7(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, LL_GPIO_AF_2);
  DHT_DATA_GPIO_PORT->MODER = 0xEBFFD4AB; // Pin1 to Alternate Function Mode
  DHT_DATA_GPIO_PORT->AFR[0] = 0x00004420; // Pin1 to AF2

  /* Wait for buffer to be filled by interrupt */
  while (bufferIndex < 42);

  /* Stop Input Capture interrupts*/
  HAL_TIM_IC_Stop_IT(&TimHandle, TIM_CHANNEL_2);

  return decodeBuffer(values, buffer, BUFF_SIZE);
}

DHT_StatusTypeDef DHT_ReadSensor_DMA(DHT_ValuesTypeDef *values)
{
  // TODO
  // /*##-3- Start the Input Capture in DMA mode ################################*/
  // if(HAL_TIM_IC_Start_DMA(&TimHandle, TIM_CHANNEL_2, buffer, 42) != HAL_OK)
  // {
  //   /* Starting Error */
  //   while(1);
  // }

  return DHT_TIMEOUT_ERROR;
}


DHT_StatusTypeDef decodeBuffer(DHT_ValuesTypeDef *values, uint16_t *buffer,
                               uint8_t size)
{
  /* Read sampled data */
  // if high time > 24us --> 0b1
  // else if high time < 70us --> 0b0

  /* Array is converted into 5 8-bit variables */
  uint8_t RH_HByte = 0;
  uint8_t RH_LByte = 0;
  uint8_t T_HByte  = 0;
  uint8_t T_LByte  = 0;
  uint8_t checksum = 0;

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
  if (checksum != ((RH_HByte + RH_LByte + T_HByte + T_LByte) & 0xFF))
  {
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
    if (captureEdge == 1)
     {
       // /* Get the 1st Input Capture value */
       // uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
       /* Reset timer counter on 1st Input Capture */
       __HAL_TIM_SET_COUNTER(&TimHandle, 0x00000U);
       captureEdge = 0;
    }
    else
    {
      /* Store measured time on 2nd Input Capture */
      if (bufferIndex >= 0 && bufferIndex < BUFF_SIZE)
      {
        buffer[bufferIndex] = __HAL_TIM_GET_COUNTER(&TimHandle);
      }
      bufferIndex--;
      captureEdge = 1;
    }
  }
}

