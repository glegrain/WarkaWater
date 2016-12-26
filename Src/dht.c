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
  HAL_GPIO_Init(DHT_DATA_GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_PIN_SET);

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
  uint16_t buffer[41];
  uint16_t counter;
  /* Configure pin to output */
  // DHT_DATA_GPIO_PORT->MODER &= 0xFFFFDFFF;
  GPIO_ConfigureMode(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_MODE_OUTPUT_PP);
  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_PIN_RESET);
  HAL_Delay(DHT_WAKEUP_DELAY);
  HAL_GPIO_WritePin(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_PIN_SET);
  /* Configure pin to input */
  DHT_DATA_GPIO_PORT->MODER &= 0xFFFFCFFF; // function call is too slow with -O0
  // GPIO_ConfigureMode(DHT_DATA_GPIO_PORT, DHT_DATA_PIN, GPIO_MODE_INPUT);

  /* Sample DATA from sensor */
  // NOTE: raw register read is faster than HAL call
  // TODO: find minimum sampling rate
  // TODO: use a timer instead of a counter
  // 1. wait for data to go high
  // 2. mesure high time.
  // if high time > 24us --> 0b1
  // else if high time < 70us --> 0b0
  for (int i = 40; i >=0; i--) {
    /* wait for DATA to go high */
    while ((DHT_DATA_GPIO_PORT->IDR & DHT_DATA_PIN) == (uint32_t)GPIO_PIN_RESET);
    /* measure high time */
    counter = 0;
    while ((DHT_DATA_GPIO_PORT->IDR & DHT_DATA_PIN) != (uint32_t)GPIO_PIN_RESET) {
      counter++;
      if (counter > 50) return DHT_TIMEOUT_ERROR;
    }
    // store counter value in buffer for debug
    buffer[i] = counter;
  }

  /* Read sampled data */
  /* Array is converted into 5 8-bit variables */
  uint8_t RH_HByte = 0;
  uint8_t RH_LByte = 0;
  uint8_t T_HByte  = 0;
  uint8_t T_LByte  = 0;
  uint8_t checksum = 0;

  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+32] > 4) bit = 1;
    RH_HByte += bit << i; // Shift bit is equivalent to power of 2
  }

  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+24] > 4) bit = 1;
    RH_LByte += bit << i;
  }

  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+16] > 4) bit = 1;
    T_HByte += bit << i;
  }

    for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i+8] > 4) bit = 1;
    T_LByte += bit << i;
  }

  for (int i = 0; i < 8; i++) {
    int bit = 0;
    if (buffer[i] > 4) bit = 1;
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