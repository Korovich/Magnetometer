/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HAL_LCD_MT_12864A.h"
#include "LibSym.h"
#include "math.h"
#include "usbd_cdc_if.h"
#include "ADS1220.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
void LcdByte(uint32_t byte)
{
	GPIOA->ODR |= byte>>11;
	byte &=(uint32_t)0x7FFF;
	GPIOA->ODR |=(byte>>10)<<1;
	byte &=(uint32_t)0x3FFF;
	GPIOB->ODR |=(byte>>9)<<11;
	byte &=(uint32_t)0x1FFF;
	GPIOB->ODR |=(byte>>8)<<10;
	byte &=(uint32_t)0x0FF;
	GPIOB->ODR |=(byte>>7)<<2;
	byte &=(uint32_t)0x07F;
	GPIOB->ODR |=(byte>>6)<<1;
	byte &=(uint32_t)0x03F;
	GPIOB->ODR |=(byte>>5);
	byte &=(uint32_t)0x01F;
	GPIOA->ODR |=byte<<3;	 
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);    // Выставляем Е
	for (int i=0;i!=100;i++){}
	//HAL_Delay(1);			   	// Пауза 1 мкс
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);	// Сбрасываем Е		   	// Пауза 1 мкс
	
	byte = 0x0000;
	GPIOA->ODR &= byte>>11;
	byte &=(uint32_t)0x7FFF;
	GPIOA->ODR &=(byte>>10)<<1;
	byte &=(uint32_t)0x3FFF;
	GPIOB->ODR &=(byte>>9)<<11;
	byte &=(uint32_t)0x1FFF;
	GPIOB->ODR &=(byte>>8)<<10;
	byte &=(uint32_t)0x0FF;
	GPIOB->ODR &=(byte>>7)<<2;
	byte &=(uint32_t)0x07F;
	GPIOB->ODR &=(byte>>6)<<1;
	byte &=(uint32_t)0x03F;
	GPIOB->ODR &=(byte>>5);
	byte &=(uint32_t)0x01F;
	GPIOA->ODR &=byte<<3;
}

void LcdByteWO(uint32_t byte)
{
	GPIOA->ODR |= byte>>11;
	byte &=(uint32_t)0x7FFF;
	GPIOA->ODR |=(byte>>10)<<1;
	byte &=(uint32_t)0x3FFF;
	GPIOB->ODR |=(byte>>9)<<11;
	byte &=(uint32_t)0x1FFF;
	GPIOB->ODR |=(byte>>8)<<10;
	byte &=(uint32_t)0x0FF;
	GPIOB->ODR |=(byte>>7)<<2;
	byte &=(uint32_t)0x07F;
	GPIOB->ODR |=(byte>>6)<<1;
	byte &=(uint32_t)0x03F;
	GPIOB->ODR |=(byte>>5);
	byte &=(uint32_t)0x01F;
	GPIOA->ODR |=byte<<3;	 
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);    // Выставляем Е
	//HAL_Delay(1);			   	// Пауза 1 мкс
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);	// Сбрасываем Е
	
	byte = 0x0000;
	GPIOA->ODR &= byte>>11;
	byte &=(uint32_t)0x7FFF;
	GPIOA->ODR &=(byte>>10)<<1;
	byte &=(uint32_t)0x3FFF;
	GPIOB->ODR &=(byte>>9)<<11;
	byte &=(uint32_t)0x1FFF;
	GPIOB->ODR &=(byte>>8)<<10;
	byte &=(uint32_t)0x0FF;
	GPIOB->ODR &=(byte>>7)<<2;
	byte &=(uint32_t)0x07F;
	GPIOB->ODR &=(byte>>6)<<1;
	byte &=(uint32_t)0x03F;
	GPIOB->ODR &=(byte>>5);
	byte &=(uint32_t)0x01F;
	GPIOA->ODR &=byte<<3;
}

void SetPage(uint32_t code,uint8_t crystal)
{
	if (crystal ==1)
	{
		code |= 0x1B8;    // Побитное ИЛИ кода с командой в левый кристалл
		LcdByte(code);
	}
	if (crystal ==2)
	{
		code |= 0x2B8;    // Побитное ИЛИ кода с командой в левый кристалл
		LcdByte(code);
	}
}

void SetAdress(uint32_t code,uint8_t crystal)
{
	if (crystal ==1)
	{
		code |= 0x140;    // Побитное ИЛИ кода с командой в левый кристалл
		LcdByte(code);
	}
	if (crystal ==2)
	{
		code |= 0x240;    // Побитное ИЛИ кода с командой в левый кристалл
		LcdByte(code);
	}
}

void WData(uint32_t code,uint8_t crystal)
{
	if (crystal ==1)
	{
		code |= 0x900;    // Побитное ИЛИ кода с командой в левый кристалл
		LcdByte(code);
	}
	if (crystal ==2)
	{
		code |= 0xA00;    // Побитное ИЛИ кода с командой в правый кристалл
		LcdByte(code);
	}
}

void RData (uint8_t crystal)
{
	if (crystal ==1)
	{
		LcdByte(0xB00);
	}
	if (crystal ==2)
	{
		LcdByte(0xE00);
	}
}

void WCommand(uint32_t code,uint8_t crystal)
{
	if (crystal ==1)
	{
		code |= 0x100;    // Побитное ИЛИ кода с командой в левый кристалл
		LcdByte(code);
	}
	if (crystal ==2)
	{
		code |= 0x200;    // Побитное ИЛИ кода с командой в правый кристалл
		LcdByte(code);
	}
}

void Status(uint8_t crystal)
{ 
	if (crystal == 1)
	{
		LcdByteWO((uint32_t)0x500);
	}
	if (crystal == 2)
	{
		LcdByteWO(0x0600);
	}
}

void DisplaySw(uint8_t crystal,uint8_t state)
{ 
	if (state == 1)
	{
		if (crystal == 1)
		{
			LcdByte((uint32_t)0x13F);
		}
		if (crystal == 2)
		{
			LcdByte((uint32_t)0x23F);
		}
	}
	if (state == 0)
	{
		if (crystal == 1)
		{
			LcdByte((uint32_t)0x13E);
		}
		if (crystal == 2)
		{
			LcdByte((uint32_t)0x23E);
		}
	}
}

void ClearLCD(uint8_t crystal)
{
	if (crystal == 0)
	{
		for (uint8_t j=0; j<8;j++)
		{
			SetPage((uint32_t)j,1);
			SetPage((uint32_t)j,2);
			SetAdress((uint32_t)0,1);
			SetAdress((uint32_t)0,2);
			for (uint8_t i=0;i<64;i++)
			{
				WData(0x00,1);
				WData(0x00,2);
			}
		}
	}
	if (crystal == 1)
	{
		for (uint8_t j=0; j<8;j++)
		{
			SetPage((uint32_t)j,1);
			SetAdress((uint32_t)0,1);
			for (uint8_t i=0;i<64;i++)
			{
				WData(0x00,1);
			}
		}
	}
	if (crystal == 2)
	{
		for (uint8_t j=0; j<8;j++)
		{
			SetPage((uint32_t)j,2);
			SetAdress((uint32_t)0,2);
			for (uint8_t i=0;i<64;i++)
			{
				WData(0x00,2);
			}
		}
	}
}

void WByte (uint8_t x,uint8_t y, uint8_t code)
{
	if (x<=64)
	{
		SetPage(y,1);
		SetAdress(x,1);
		WData(code,1);
	}
	if (x>64)
	{
		SetPage(y,2);
		SetAdress(x-64,2);
		WData(code,2);
	}
}

void LcdInit(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);    // Выставляем res
	HAL_Delay(1);			   	// Пауза 1 мкс
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);	// Сбрасываем res
	HAL_Delay(10);			   	// Пауза 1 мкс
	DisplaySw(1,1);
	DisplaySw(2,1);
	ClearLCD(0);
}

void WSym (uint8_t x, uint8_t y, uint8_t symbol[6])
{
	if (x<=57)
	{
			SetPage(y,1);
			SetAdress(x,1);
			for (uint8_t j=0;j<6;j++)
			{
				WData(symbol[j],1);
			}
	}
	if(x>=64)
	{
			SetPage(y,2);
			SetAdress(x,2);
			for (uint8_t j=0;j<=x;j++)
			{
				WData(symbol[j],2);
			}
	}
}

uint8_t SymC2Int (char ch)
{
	uint8_t n=0;
	for (uint8_t j=0;j<strlen(codeSymR);j++)
	{
		if (ch==codeSymR[j])
		{
			n=j;
			break;
		}			
	}
	return(n);
}

void WStr(uint8_t x,uint8_t y,char str[])
{
	if (x<=57)
	{
		SetPage(y,1);
		SetAdress(x,1);
		for (uint8_t i=0;i<strlen(str);i++)
		{
			for (uint8_t j=0;j<sizeof(SmFR[SymC2Int(str[i])+96]);j++)
			{
				WData(SmFR[SymC2Int(str[i])+96][j],1);
			}
		}
	}
	if(x>=64)
	{
		SetPage(y,2);
		SetAdress(x,2);
		for (uint8_t j=0;j<6;j++)
		{
			for (uint8_t i=0;i<strlen(str);i++)
			{
				WData(SmFR[SymC2Int(str[i])+97][j],2);
			}
		}
	}
}

void WNum (uint8_t x,uint8_t y, int32_t num)
{
	uint8_t m=0;
	uint8_t minus = 0;
	if (num<0) 
		{
			minus = 1;
			num = abs(num);
		}
	int32_t n=num;
	while(n!=0)
	{
		n = n/10;
		m++;
	}
	if (x<=57)
	{
		SetPage(y,1);
		SetAdress(x,1);
		
		if (minus == 1)
		{
			for (uint8_t j=0;j<6;j++)
			{
				WData(SmFR[29][j],1);
			}
		}			
		
		for (uint8_t i=m;i>0;i--)
		{
			n=num/(pow(10,i-1));
			for (uint8_t j=0;j<6;j++)
			{
				WData(SmFR[n][j],1);
			}
			num=num%(uint32_t)(pow(10,i-1));
		}
		
	}
	/*
	if(x>=64)
	{
		SetPage(y,2);
		SetAdress(x,2);
		for (uint8_t j=0;j<6;j++)
		{
			for (uint8_t i=0;i<m;i++)
			{
				WData(SmFR[n/(10*m)][j],2);
			}
		}
	}
	*/
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	LcdInit();
	WStr(0,0,"Привет мир");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		CDC_Transmit_FS((uint8_t*)"0",1);
		HAL_Delay(100);
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))
		{
			for (int i=0;i<10000;i++)
			{
				WNum(0,1,i);
			}
		}
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))
		{
			WSym(0,0,(uint8_t*)SmFR[161]);
			HAL_Delay(10);
		}
		
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7))
		{
			CDC_Transmit_FS((uint8_t*)"2",1);
			SetPage((uint32_t)0,1);
			SetPage((uint32_t)0,2);
			SetAdress((uint32_t)0,1);
			SetAdress((uint32_t)0,2);
		}
		
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3))
		{
			CDC_Transmit_FS((uint8_t*)"3",1);
			DisplaySw(1,0);
			HAL_Delay(100);
		}
		
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6))
		{
			ADS1220_Init();
			ADS1220_SingleShot_Mode();
			ADS1220_Set_Data_Rate(DR_1000SPS);
			ADS1220_PGA_ON();
			//ADS1220_Set_PGA_Gain(PGA_GAIN_1);
			ADS1220_Volt_Ref(VOLT_REF_ANALOG);
			while(1)
			{
				
				ADS1220_Sel_MUX_ch(MUX_AIN3_AVSS);
				WNum(0, 1, ADS1220_Read_Single_WaitForData());
				ADS1220_Sel_MUX_ch(MUX_AIN2_AVSS);
				WNum(0, 2, ADS1220_Read_Single_WaitForData());
				ADS1220_Sel_MUX_ch(MUX_AIN1_AVSS);
				WNum(0, 3, ADS1220_Read_Single_WaitForData());
				
				//HAL_Delay(10);
			}
			/*
			uint8_t buf=0;
			uint32_t bufarr=0;
			uint8_t resp1=0;
			uint8_t resp2=0;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi2,(uint8_t*)0x06,1,10);//reset
			HAL_Delay(100);
			HAL_SPI_Transmit(&hspi2,(uint8_t*)0x43,1,10);//com wreg
			HAL_SPI_Transmit(&hspi2,(uint8_t*)0xb8,1,10);//0reg b0
			HAL_SPI_Transmit(&hspi2,(uint8_t*)0x04,1,10);//1reg 04
			HAL_SPI_Transmit(&hspi2,(uint8_t*)0x10,1,10);//2reg 10
			HAL_SPI_Transmit(&hspi2,(uint8_t*)0x00,1,10);//3reg 00
			HAL_Delay(10);
			HAL_SPI_Transmit(&hspi2,(uint8_t*)0x08,1,10);//start
			HAL_Delay(10);
			
			NVIC_EnableIRQ (EXTI15_10_IRQn);
			
			while(1)
			{
				bufarr=0;
				if(!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
				{
					HAL_Delay(10);
					HAL_SPI_Receive(&hspi2,&buf,1,10);
					bufarr =buf;
					HAL_SPI_Receive(&hspi2,&buf,1,10);
					bufarr +=buf<<8;
					HAL_SPI_Receive(&hspi2,&buf,1,10);
					bufarr +=buf<<16;
					HAL_Delay(5);
					WNum(0,1,bufarr);
					HAL_SPI_Transmit(&hspi2,(uint8_t*)0x10,1,10);
					CDC_Transmit_FS((uint8_t*)&bufarr,3);
					HAL_Delay(100);
					WStr(0,1,"             ");
					CDC_Transmit_FS((uint8_t*)"321",1);
				}
				__nop();
			}
			*/
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
