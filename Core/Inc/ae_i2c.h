/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ae_i2c.h
  * @brief          : Header for i2c_gpio.c file.
  *                   This file contains the common defines of the application.
  * @version		: 0.0.1
  * @creaor			: JKS
  * @update			: 21.08.12
  ******************************************************************************
  * @attention			: Design for BlueBerry Board
  *
  ******************************************************************************
  */
/* USER CODE END Header */


#ifndef __I2C_GPIO_H
#define __I2C_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void idelay(uint32_t tmout);
void init_i2c(void);
uint32_t i2c_write_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t* dat, uint16_t size, uint16_t tmout);
uint32_t i2c_read_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t* dat, uint16_t size, uint16_t tmout);


#ifdef __cplusplus
}
#endif

#endif /* __I2C_GPIO_H */

