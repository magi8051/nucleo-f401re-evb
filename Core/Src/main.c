/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ae_i2c.h"
/*
 * Make		: 2021.08.02
 * Creator	: JKS
 *
 *
 */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned char u8;
typedef signed char s8;
typedef unsigned short u16;
typedef signed short s16;
typedef unsigned int u32;
typedef signed int s32;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_BUAD 		921600
#define RX_SIZE 		1024
#define BBB_HD_SIZE 	5
#define STREAM_HD 		7

#define P0(in) 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, in)
#define P1(in) 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, in)
#define P2(in) 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, in)
#define P3(in) 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, in)

#define P0_T 			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6)
#define P1_T 			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7)
#define P2_T 			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8)
#define P3_T 			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9)

#define S_CTRL(in)		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5, in) //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10, in)
#define S_EN(in)		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, in) //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11, in)
#define T_CTRL			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5)
#define T_EN			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6)

#define T_LED 			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5)

#define ENP(in)			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5, in)
#define ENN(in)			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, in)

#define CWB_RESET 		SCB->AIRCR = 0x05FA0000 | 0x04

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Delay_us(int delay);
void DeviceSet(void);
int S_Wire_Func(int type);
int S_Wire_Test(int type);
void VCP_Transmit_Packet(int plus);
void VCP_Receive_Packet(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);

u16 _8u16(u8 a, u8 b) { return (u16)(((u16)a << 8) | (u16)b); }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

struct BBB_PACKET {
	u8 Buf[RX_SIZE];
	u8 Get;
	u32 RxSize;
	u32 TxSize;
};

struct FTUSB_SR {
	u32 Rx_Done : 1,
		Tx_Done : 1,
		reserve : 30;
};

/*Base Struct for BBB*/
struct FTUSB_SR 	USBsr;
struct BBB_PACKET	USBpk;



u16 g_update;
u16 g_firmver;
u16 g_hardver;

u32 g_swire_pulse_time;
u32 g_swire_init_time;
u32 g_swire_store_time;

u32 g_swire_pulse_time1;
u32 g_swire_init_time1;
u32 g_swire_store_time1;

u32 g_swire_pulse_time2;
u32 g_swire_init_time2;
u32 g_swire_store_time2;

u32 g_swire_pulse_time3;
u32 g_swire_init_time3;
u32 g_swire_store_time3;
u32 g_swire_test_flag;

u32 g_dw8786_pin_idle;
u32 g_dw8786_pin_mode;

u32 g_dw8790_mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*Before use, stdio.h must be declared first.*/
#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
    if( HAL_UART_Transmit(&huart2, ptr, len, len) == HAL_OK ) return len;
    else return 0;
}



/**************************************************************************
Function	: LED display
Date		: 2021.07.14
Version		: CLK base is 170Mhz
***************************************************************************/
void LED_LOOK()
{
	static int t;

	if(t > 500000) {
		t=0;
		T_LED;
	}
	else t++;
}



/**************************************************************************
Function	: soft delay
Date		: 2021.07.14
Version		: CLK base is 170Mhz
***************************************************************************/
void Delay_us(int delay)
{
	volatile int i, k;

	for(i=0; i < delay; i++) {
		for(k=0; k < 1; k++) {
			__asm volatile("NOP");
		}
	}

	for(i=0; i < delay; i++) {
		for(k=0; k < 2; k++) {
			__asm volatile("NOP");
			__asm volatile("NOP");
			__asm volatile("NOP");
			__asm volatile("NOP");
			__asm volatile("NOP");
			__asm volatile("NOP");
			__asm volatile("NOP");
			//__asm volatile("NOP");
		}
	}
}




/*
CN7 Header Pin
PC10 : CTRL
PC11 : EN
*/

int S_Wire_Func(int type)
{
	u32 cnt, p0, p1, p2, p3;

	p0 = (uint32_t)USBpk.Buf[6]<<8 | USBpk.Buf[7];
	p1 = (uint32_t)USBpk.Buf[8]<<8 | USBpk.Buf[9];
	p2 = (uint32_t)USBpk.Buf[10]<<8 | USBpk.Buf[11];
	p3 = (uint32_t)USBpk.Buf[12]<<8 | USBpk.Buf[13];

	if(type == 0) S_CTRL(1);
	else S_EN(1);

	//step 1: init time setting
	Delay_us(g_swire_init_time);

	//step 2: setup time setting
	for(cnt=0; cnt < p0; cnt++) {
		Delay_us(g_swire_pulse_time);	// setup time
		if(type == 0) T_CTRL;
		else T_EN;
	}

	Delay_us(g_swire_store_time);	// store time
	S_CTRL(0);

	if(g_swire_init_time1 > 0 && p1 > 0)
	{
		if(type == 0) S_CTRL(1);
		else S_EN(1);
		Delay_us(g_swire_init_time1);	// store time

		for(cnt=0; cnt < p1; cnt++) {
			Delay_us(g_swire_pulse_time1);	// setup time
			if(type == 0) T_CTRL;
			else T_EN;
		}
		Delay_us(g_swire_store_time1);	// store time
	}

	if(g_swire_init_time2 > 0 && p2 > 0)
	{
		if(type == 0) S_CTRL(1);
		else S_EN(1);
		Delay_us(g_swire_init_time2);	// store time

		for(cnt=0; cnt < p2; cnt++) {
			Delay_us(g_swire_pulse_time2);	// setup time
			if(type == 0) T_CTRL;
			else T_EN;
		}
		Delay_us(g_swire_store_time2);	// store time
	}

	if(g_swire_init_time3 > 0 && p3 > 0)
	{
		if(type == 0) S_CTRL(1);
		else S_EN(1);
		Delay_us(g_swire_init_time3);	// store time

		for(cnt=0; cnt < p3; cnt++) {
			Delay_us(g_swire_pulse_time3);	// setup time
			if(type == 0) T_CTRL;
			else T_EN;
		}
		Delay_us(g_swire_store_time3);	// store time
	}
	return 0;
}



int S_Wire_Test(int type)
{
	volatile uint32_t cnt, pulse, pulse1;

	if(g_swire_test_flag == 1) {

		pulse = (uint32_t)USBpk.Buf[7]<<8 | USBpk.Buf[8];
		pulse1 = (uint32_t)USBpk.Buf[9]<<8 | USBpk.Buf[10];

		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, 1);
		Delay_us(g_swire_init_time);	// store time 1

		for(cnt=0; cnt < pulse; cnt++) {
			Delay_us(g_swire_pulse_time);	// setup time 1
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);	// p3 wire
		}
		Delay_us(g_swire_store_time);	// store time 1

		g_swire_test_flag = 2;
	}

	if(g_swire_init_time1 > 0 && pulse1 > 0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12, 1);
		Delay_us(g_swire_init_time1);	// store time 2

		for(cnt=0; cnt < pulse1; cnt++) {
			Delay_us(g_swire_pulse_time1);	// setup time 1
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);	// p3 wire
		}
		Delay_us(g_swire_store_time1);	// store time 2
	}

	return 0;
}

/**************************************************************************
Function	: Boot Message Function
Date		: 2021.07.13 JKS
Version		: 1.0
***************************************************************************/
void Boot_Message(void)
{
	g_update = 0x0729;
	g_firmver = 0x0005;
	g_hardver = 0x8786;
	g_swire_pulse_time = 10;
	g_swire_init_time = 1000;
	g_swire_store_time = 80;

	USBpk.Buf[5] = 0;
	USBpk.Buf[6] = 0;
	USBpk.Buf[7] = 1;
	USBpk.Buf[8] = g_hardver >> 8;
	USBpk.Buf[9] = (uint8_t)g_hardver;
	USBpk.Buf[10] = g_firmver >> 8;
	USBpk.Buf[11] = (uint8_t)g_firmver;
	USBpk.Buf[12] = g_update >> 8;
	USBpk.Buf[13] = (uint8_t)g_update;
	VCP_Transmit_Packet(10);
}



/**************************************************************************
Function	: Device Set Call Function
Date			: 2017.02.22
Version	: 1.0
***************************************************************************/
void DeviceSet(void)
{
	switch(USBpk.Buf[6]) {

	case 1:     // who are you
		if(USBpk.Buf[7] == 1) Boot_Message();
		break;
	case 2: // Reset
		SCB->AIRCR = 0x05FA0000 | 0x04;
		break;
	case 20:
		ENP(USBpk.Buf[7]);
		break;
	case 21:
		ENN(USBpk.Buf[7]);
		break;
	case 22:
		S_EN(USBpk.Buf[7]);
		break;
	case 23:
		S_CTRL(USBpk.Buf[7]);
		break;
	case 26:	// ctrl + en pin idle state set
		if(USBpk.Buf[7] == 1) g_dw8786_pin_idle = 1;
		else g_dw8786_pin_idle = 0;
		break;
	}
}



/**************************************************************************
Function	: VCP Transmit Packet (Maximum Transmit size 4Kbyte)
Date		: 2021.7.13 JKS
Version		: 1.0
Descript 	: size is Actual data size without header(5)
***************************************************************************/
void VCP_Transmit_Packet(int plus)
{
	USBpk.TxSize = plus;
	USBpk.Buf[0] = 0x28;
	USBpk.Buf[1] = 0xAE;
	USBpk.Buf[2] = USBpk.TxSize >> 8;
	USBpk.Buf[3] = USBpk.TxSize;
	USBpk.Buf[4] = USBpk.Buf[0] + USBpk.Buf[1] + USBpk.Buf[2] + USBpk.Buf[3];
	HAL_UART_Transmit_IT(&huart2,(uint8_t*)USBpk.Buf, USBpk.TxSize + BBB_HD_SIZE);

	/*Wait for the transfer to complete.*/
	//while(USBsr.Tx_Done == 0);
	//USBsr.Tx_Done = 0;
}



void i2c_task(void)
{
	int ret;

	u16 size;

	size = _8u16(USBpk.Buf[7], USBpk.Buf[8]);

	if(USBpk.Buf[6] == 4) { //multi read
		ret = i2c_read_task(USBpk.Buf[9], USBpk.Buf[10], 1, USBpk.Buf+12, size, 10);
		size = 7 + size;
	}

	else if(USBpk.Buf[6] == 5) { //multi write
		ret = i2c_write_task(USBpk.Buf[9], USBpk.Buf[10], 1, USBpk.Buf+11, size, 10);
		size = 7;
		VCP_Transmit_Packet(size);
	}

	if(ret == 1) { // noack
		size = 3;
		USBpk.Buf[5] = 0;
		USBpk.Buf[6] = 1;
		USBpk.Buf[7] = 1;
		VCP_Transmit_Packet(size);
	}


}

/**************************************************************************
Function	: VCP Receive Packet (Maximum Receive size 4Kbyte)
Date		: 2021.07.13 JKS
Version		: 1.0
***************************************************************************/
void VCP_Receive_Packet(void)
{
	switch(USBpk.Buf[5]) {
	case 0:
		DeviceSet();
		break;

	case 1: // I2C Command
		i2c_task();
		break;

	case 5: //	s-wire set
		g_swire_init_time = (uint32_t)USBpk.Buf[6]<<8 | USBpk.Buf[7];
		g_swire_pulse_time = (uint32_t)USBpk.Buf[8]<<8 | USBpk.Buf[9];
		g_swire_store_time = (uint32_t)USBpk.Buf[10]<<8 | USBpk.Buf[11];
		g_swire_init_time1 = (uint32_t)USBpk.Buf[12]<<8 | USBpk.Buf[13];
		g_swire_pulse_time1 = (uint32_t)USBpk.Buf[14]<<8 | USBpk.Buf[15];
		g_swire_store_time1 = (uint32_t)USBpk.Buf[16]<<8 | USBpk.Buf[17];
		g_swire_init_time2 = (uint32_t)USBpk.Buf[18]<<8 | USBpk.Buf[19];
		g_swire_pulse_time2 = (uint32_t)USBpk.Buf[20]<<8 | USBpk.Buf[21];
		g_swire_store_time2 = (uint32_t)USBpk.Buf[22]<<8 | USBpk.Buf[23];
		g_swire_init_time3 = (uint32_t)USBpk.Buf[24]<<8 | USBpk.Buf[25];
		g_swire_pulse_time3 = (uint32_t)USBpk.Buf[26]<<8 | USBpk.Buf[27];
		g_swire_store_time3 = (uint32_t)USBpk.Buf[28]<<8 | USBpk.Buf[29];
		VCP_Transmit_Packet(5);
		break;

	case 6: // s-wire play
		g_dw8786_pin_mode = USBpk.Buf[14];
		S_Wire_Func(g_dw8786_pin_mode);
		break;

	case 7: // s-wire test
		g_swire_test_flag = USBpk.Buf[6];
		break;
	}

	USBsr.Rx_Done = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Boot_Message();
  HAL_UART_Receive_IT(&huart2, &USBpk.Get, 1);
  init_i2c();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LED_LOOK();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  huart2.Init.BaudRate = UART_BUAD;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


/**************************************************************************
Function	: EXTI line detection callbacks
Date		: 2021.06.13 JKS
Version		: 1.0
***************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint8_t dat;

	if(GPIO_Pin == GPIO_PIN_13) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//printf("test\r\n");
		Boot_Message();
		dat++;
		i2c_write_task(0xe4, 0x01, 1, &dat, 1, 1);
		ENP(dat&0x01);
		ENN(dat&0x01);
	}
}



/**************************************************************************
Function	: Rx Transfer completed callback
Date		: 2019.10.28 JKS
Version		: 1.0
***************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    u8 crc;
	static u32 RxCnt, g_RxSize;

	USBpk.Buf[RxCnt++] = USBpk.Get;

    if(RxCnt == 4) {
        g_RxSize = (((uint32_t)USBpk.Buf[2] << 8) | USBpk.Buf[3]) + 5;
    }

    if (RxCnt == g_RxSize) {
        crc = USBpk.Buf[0] + USBpk.Buf[1] + USBpk.Buf[2] + USBpk.Buf[3];

        if(USBpk.Buf[4] == crc) {
         HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
         VCP_Receive_Packet();
        }
        RxCnt = 0;
    }

	HAL_UART_Receive_IT(&huart2, &USBpk.Get, 1);
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
	USBsr.Tx_Done = 1;
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
