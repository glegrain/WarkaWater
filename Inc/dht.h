/**
  ******************************************************************************
  * @file    Inc/dht.h
  * @author  Guillaume Legrain
  * @version V0.1.1
  * @date    26-December-2016
  * @brief   This file includes the DHT11 sensor driver.
  ******************************************************************************
  */

#ifndef __DHT_H
#define __DHT_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
// #include "stm32l0xx_hal_def.h"
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  DHT Status structures definition  
  */  
typedef enum 
{
  DHT_OK             = 0x00U,
  DHT_CHECKSUM_ERROR = 0x01U,
  DHT_TIMEOUT_ERROR  = 0x02U
} DHT_StatusTypeDef;

// NOTE: The DHT11 only fills the decimal part and keeps the fractional part zero.
//       The DHT22 and 21 do provide a fractonal part.
//       The DHT11 does also not support negative values, where the 22 and 21 do.
typedef struct
{
  uint8_t RelativeHumidityIntegral;
  uint8_t RelativeHumidityFractional;
  uint8_t TemperatureIntegral;
  uint8_t TemperatureFractional;
} DHT_ValuesTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DHT_GPIO_Init(void);
DHT_StatusTypeDef DHT_ReadSensor(DHT_ValuesTypeDef *values);
DHT_StatusTypeDef DHT_ReadSensor_IT(DHT_ValuesTypeDef *values);
DHT_StatusTypeDef DHT_ReadSensor_DMA(DHT_ValuesTypeDef *values);

#endif /* __DHT_H */

