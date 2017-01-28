/**
  ******************************************************************************
  * @file    Inc/ms5540c.h
  * @author  Guillaume Legrain
  * @version V0.1.1
  * @date    12-January-2017
  * @brief   This file includes the MS5540C pressure sensor driver.
  ******************************************************************************
  */

#ifndef __MS5540C_H
#define __MS5540C_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l0xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
HAL_StatusTypeDef MS5540C_Init(void);
HAL_StatusTypeDef MS5540C_Acquire(void);

#endif /* __MS5540C_H */

