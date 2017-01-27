/**
  ******************************************************************************
  * @file    Src/ms5540c.c
  * @author  Guillaume Legrain
  * @version V0.1.1
  * @date    12-January-2017
  * @brief   This file includes the MS5540C pressure sensor driver.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ms5540c.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi;
/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef transmitReceive(uint16_t *pTxData, uint16_t *pRxData,
                                  uint32_t Timeout);
void resetSensor(void);
/* Private functions ---------------------------------------------------------*/
HAL_StatusTypeDef MS5540C_Init(void)
{
  HAL_StatusTypeDef status;

  /*##- Configure the SPI peripheral #########################################*/
  hspi.Instance               = SPIx;
  hspi.Init.Mode              = SPI_MODE_MASTER;
  hspi.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi.Init.DataSize          = SPI_DATASIZE_16BIT;
  hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi.Init.CLKPhase          = SPI_PHASE_1EDGE; // ??
  hspi.Init.NSS               = SPI_NSS_SOFT;
  hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi.State                  = HAL_SPI_STATE_RESET;

  status = HAL_SPI_Init(&hspi);
  if (status != HAL_OK) return status;

  /* Output LSE (32.768kHz) to MCO pin. To be used by MS5540C MSCLK */
  /* Configure GPIO pin : PA9 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin       = GPIO_PIN_9;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO1SOURCE_LSE, RCC_MCODIV_1);

  return HAL_OK;
}

HAL_StatusTypeDef MS5540C_Acquire(void)
{
  uint16_t txBuffer;
  uint16_t rxBuffer;

  /* The MS5540C device does not need a 'chip select' signal. Instead there is
   * a START sequence (3-bit high) before each SETUP sequence and a STOP
   * sequence (3-bit low) after each SETUP sequence.
   */

  resetSensor();

  /*##-2- Read calibration data (factory calibrated) from PROM of MS5540C ####*/
  uint16_t word1, word2, word3, word4;
  /* Get Calibration word 1 */
  txBuffer = 0x1D50;
  rxBuffer = 0;
  HAL_SPI_Transmit(&hspi, (uint8_t *) &txBuffer, 1, 5000);
  txBuffer = 0;
  HAL_SPI_TransmitReceive(&hspi, (uint8_t *) &txBuffer, (uint8_t *) &rxBuffer, 1, 5000);
  word1 = rxBuffer;

  resetSensor();

  /* Get Calibration word 2 */
  txBuffer = 0x1D60;
  rxBuffer = 0;
  HAL_SPI_Transmit(&hspi, (uint8_t *) &txBuffer, 1, 5000);
  txBuffer = 0;
  HAL_SPI_TransmitReceive(&hspi, (uint8_t *) &txBuffer, (uint8_t *) &rxBuffer, 1, 5000);
  word2 = rxBuffer;

  resetSensor();


  /* Get Calibration word 3 */
  txBuffer = 0x1D90;
  rxBuffer = 0;
  HAL_SPI_Transmit(&hspi, (uint8_t *) &txBuffer, 1, 5000);
  txBuffer = 0;
  HAL_SPI_TransmitReceive(&hspi, (uint8_t *) &txBuffer, (uint8_t *) &rxBuffer, 1, 5000);
  word3 = rxBuffer;

  resetSensor();

  /* Get Calibration word 4 */
  txBuffer = 0x1DA0;
  rxBuffer = 0;
  HAL_SPI_Transmit(&hspi, (uint8_t *) &txBuffer, 1, 5000);
  txBuffer = 0;
  HAL_SPI_TransmitReceive(&hspi, (uint8_t *) &txBuffer, (uint8_t *) &rxBuffer, 1, 5000);
  word4 = rxBuffer;

  /*##-3- Convert calibration data into coefficients #########################*/
  uint16_t c1 = (word1 >> 1) & 0x7FFF; // Pressure sensitivity
  uint16_t c2 = ((word3 & 0x003F) << 6) | (word4 & 0x003F); // Pressure offset
  uint16_t c3 = (word4 >> 6) & 0x03FF;
  uint16_t c4 = (word3 >> 6) & 0x03FF;
  uint16_t c5 = ((word1 & 0x0001) << 10) | ((word2 >> 6) & 0x03FF);
  uint16_t c6 = word2 & 0x003F;

  resetSensor();
  hspi.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
  HAL_SPI_Init(&hspi);

  /*##-4- Read digital pressure value from MS5540C ###########################*/
  // TODO: Send reset (it is recommended to send a reset before each conversion)
  txBuffer = 0x0F40;
  rxBuffer = 0;
  HAL_SPI_Transmit(&hspi, (uint8_t *) &txBuffer, 1, 5000);
  HAL_Delay(36); // wait for end of conversion (can be an interrupt on MISO)
  txBuffer = 0;
  HAL_SPI_TransmitReceive(&hspi, (uint8_t *) &txBuffer, (uint8_t *) &rxBuffer, 1, 5000);
  // uint16_t d1 = rxBuffer;
  uint16_t d1 = (rxBuffer << 1) & 0xFFFF;

  resetSensor();
  hspi.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
  HAL_SPI_Init(&hspi);


  /*##-5- Read digital temperature value from MS5540C ########################*/
  txBuffer = 0x0F20;
  rxBuffer = 0;
  HAL_SPI_Transmit(&hspi, (uint8_t *) &txBuffer, 1, 5000);
  HAL_Delay(36); // wait for end of conversion (can be an interrupt on MISO)
  txBuffer = 0;
  HAL_SPI_TransmitReceive(&hspi, (uint8_t *) &txBuffer, (uint8_t *) &rxBuffer, 1, 5000);
  // uint16_t d2 = rxBuffer;
  uint16_t d2 = (rxBuffer << 1) & 0xFFFF;

  /* Calculate calibration temperature */
  int32_t ut1 = 8 * c5 + 20224;

  /* Calculate actual temperature */
  int32_t dT = d2 - ut1;
  double temperature = (200 + ((dT * (c6 + 50)) >> 10)) / 10.0;

  /* Calculate pressure offset at actual temperature */
  int32_t offset = (c2 * 4) + (((c4 - 512) * dT) >> 12);
  /* Calculate pressure sensitivity at actual temperature */
  int32_t sensitivity = c1 + ((c3 * dT) >> 10) + 24576;
  int32_t x = (sensitivity * (d1 - 7168) >> 14) - offset;
  /* Calibration temperature compensated pressure */
  double pressure = (((x * 10) >> 5) + 2500) / 10.0;

  printf("words: %u, %u, %u, %u\n", word1, word2, word3, word4);
  printf("coefs: %u, %u, %u, %u, %u, %u\n", c1, c2, c3, c4, c5, c6);
  printf("d1 = %u, d2 = %u\n", d1, d2);
  printf("ut1 = %ld, dT = %ld, \n", ut1, dT);
  printf("temperature = %.1fºC, pressure = %.1f mbar\n", temperature, pressure);

  return HAL_OK;
}

HAL_StatusTypeDef transmitReceive(uint16_t *pTxData, uint16_t *pRxData,
                                  uint32_t Timeout)
{
  return HAL_OK;
}

void resetSensor(void)
{
  /* Set Motorola mode */
  // hspi.Instance->CR2 &= ~SPI_CR2_FRF_Msk;
  hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
  HAL_SPI_Init(&hspi);

  /* Reset MS5540C Sensor */
  uint32_t resetBuffer[2] = {0x0015, 0x5540};
  HAL_SPI_Transmit(&hspi, (uint8_t *) resetBuffer, 2, 5000);

  /* Set TI mode */
  // hspi.Instance->CR2 |= SPI_CR2_FRF_Msk;
  hspi.Init.TIMode            = SPI_TIMODE_ENABLE;
  HAL_SPI_Init(&hspi);
}
