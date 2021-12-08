/**
  ******************************************************************************
  * @file       : ae_i2c.c
  * @version	: 210812A
  * @creaor		: Jung Kwang Sung
  * @update		: 21.08.12
  ******************************************************************************
  * @attention	: Design for CW EVB
  * 2021.08.25 	: JKS Release
 ******************************************************************************
*/

#include "stm32f4xx_hal.h"
#include "ae_i2c.h"

/*user free define code*/
GPIO_TypeDef *I2C_BASS;
uint16_t SCL_PIN;
uint16_t SDA_PIN;

/*can not redefine!!*/
#define SDA(in)	I2C_BASS->BSRR = (in) ? SDA_PIN : SDA_PIN << 16
#define SCL(in)	I2C_BASS->BSRR = (in) ? SCL_PIN : SCL_PIN << 16

/*ACK:"0", NoACK:"1"*/
#define I2C_NACK	(I2C_BASS->IDR & SDA_PIN) ? 1 : 0
#define I2C_DAT		(I2C_BASS->IDR & SDA_PIN) ? 1 : 0


void init_i2c(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_BASS = GPIOB;

	SCL_PIN  = GPIO_PIN_8;
	SDA_PIN  = GPIO_PIN_9;

	/*Configure I2Cx Port */
	GPIO_InitStruct.Pin = SCL_PIN|SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(I2C_BASS, &GPIO_InitStruct);
	HAL_GPIO_WritePin(I2C_BASS, SDA_PIN|SCL_PIN, GPIO_PIN_SET);
}


void idelay(uint32_t tmout)
{
	if(tmout == 1) {
		#if 1	//--------------------------------- 3MHz
		__asm volatile("NOP");
		#endif

		#if 0
		__asm volatile("NOP");
		__asm volatile("NOP");
		#endif
	}
	else {
		for(volatile int i=0; i<tmout; i++);
	}
}



uint32_t i2c_write_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t* dat, uint16_t size, uint16_t tmout)
{
	uint32_t i, tmp, ack;

  /*START*/
	SDA(0);
  idelay(tmout);	// 100ns ?
  ack = 0;
	tmp = id;
	SCL(0);

  /*Start ID*/
  for(i=0; i<8; i++) {
    SDA((tmp & 0x80) ? 1 : 0);
    idelay(tmout);
    SCL(1);
    idelay(tmout);
    SCL(0);
		tmp <<= 1;
  }

  /*ACK*/
	idelay(tmout); /*hold margin*/
  SDA(1);
  idelay(tmout);
  SCL(1);
  idelay(tmout);
	ack = I2C_NACK;
  SCL(0);

	if(type == 2) {	// 16bit address
		tmp = addr>>8;
		for(i=0; i<8; i++) {
			SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			tmp <<= 1;
		}
		/*ACK*/
		idelay(tmout);	/*hold margin*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		tmp = addr;
		SCL(0);

		for(i=0; i<8; i++) {
			SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			tmp <<= 1;
		}
		/*ACK*/
		idelay(tmout);	/*hold margin*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
	}

	else {  // 8bit address
		tmp = addr;
		for(i=0; i<8; i++) {
			SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
			SCL(1);
			idelay(tmout);
			SCL(0);
			tmp <<= 1;
		}
		/*ACK*/
		idelay(tmout);	/*hold margin*/
		SDA(1);
		idelay(tmout);
		SCL(1);
		idelay(tmout);
		SCL(0);
	}

  while(size) {
    tmp = (*dat++);
    for(i=0; i<8; i++) {
      SDA((tmp & 0x80) ? 1 : 0);
      idelay(tmout);
      SCL(1);
      idelay(tmout);
      SCL(0);
      tmp <<= 1;
    }
    size--;

    /*ACK*/
		idelay(tmout);	/*hold margin*/
    SDA(1);
    idelay(tmout);
    SCL(1);
    idelay(tmout);
    SCL(0);
  }

  /*STOP*/
  SDA(0);
  idelay(tmout);
  SCL(1);
  idelay(tmout);
  SDA(1);

  return ack;
}


uint32_t i2c_read_task(uint8_t id, uint16_t addr, uint16_t type, uint8_t* dat, uint16_t size, uint16_t tmout)
{
    uint32_t i, ack, tmp;

		ack = i2c_write_task(id, addr, type, dat, 0, tmout);

		/*START*/
		idelay(tmout);
    SDA(0);
    idelay(tmout);
    SCL(0);
		idelay(tmout);
		tmp = id + 1;

		/*RID*/
    for(i=0; i<8; i++)
    {
      SDA((tmp & 0x80) ? 1 : 0);
			idelay(tmout);
      SCL(1);
      idelay(tmout);
			tmp <<= 1;
      SCL(0);
    }
    /*ACK*/
		idelay(tmout); /*hold margin*/
    SDA(1);
    idelay(tmout);
    SCL(1);
    idelay(tmout);
		tmp = 0;
    SCL(0);

		/*R-DATA*/
    while(size > 0)
    { /*read sda pin*/
      for(i=0; i<8; i++) {
				idelay(tmout);
        SCL(1);
        idelay(tmout);
				tmp |= I2C_DAT;
        SCL(0);
				if(i < 7) tmp <<= 1;
      }
      (*dat++) = tmp;
			tmp = 0;
			size--;

      if(size) {
        SDA(0);/*Force ACK*/
        idelay(tmout);
        SCL(1);
        idelay(tmout);
        SCL(0);
        __asm volatile("NOP");
        SDA(1);
				__asm volatile("NOP");
      }
      else { /*Noack*/
        idelay(tmout);
        SCL(1);
        idelay(tmout);
        SCL(0);
      }
    }

    /*STOP*/
    SDA(0);
    idelay(tmout);
    SCL(1);
    idelay(tmout);
    SDA(1);

    return ack;
}


