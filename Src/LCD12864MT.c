#include "LCD12864MT.h"

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

void LCD_SetPage(uint32_t code,uint8_t crystal)
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

void LCD_SetAdress(uint32_t code,uint8_t crystal)
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

void LCD_Clear(uint8_t crystal)
{
	if (crystal == 0)
	{
		for (uint8_t j=0; j<8;j++)
		{
			LCD_SetPage((uint32_t)j,1);
			LCD_SetPage((uint32_t)j,2);
			LCD_SetAdress((uint32_t)0,1);
			LCD_SetAdress((uint32_t)0,2);
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
			LCD_SetPage((uint32_t)j,1);
			LCD_SetAdress((uint32_t)0,1);
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
			LCD_SetPage((uint32_t)j,2);
			LCD_SetAdress((uint32_t)0,2);
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
		LCD_SetPage(y,1);
		LCD_SetAdress(x,1);
		WData(code,1);
	}
	if (x>64)
	{
		LCD_SetPage(y,2);
		LCD_SetAdress(x-64,2);
		WData(code,2);
	}
}

void LCD_Init(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);    // Выставляем res
	HAL_Delay(1);			   	// Пауза 1 мкс
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);	// Сбрасываем res
	HAL_Delay(10);			   	// Пауза 1 мкс
	DisplaySw(1,1);
	DisplaySw(2,1);
	LCD_Clear(0);
}

void LCD_WSym (uint8_t x, uint8_t y, uint8_t i)
{
	if (x<=57)
	{
			LCD_SetPage(y,1);
			LCD_SetAdress(x,1);
			for (uint8_t j=0;j<6;j++)
			{
				WData(SmFR(i,j),1);
			}
	}
	if(x>=64)
	{
			LCD_SetPage(y,2);
			LCD_SetAdress(x,2);
			for (uint8_t j=0;j<=x;j++)
			{
				WData(SmFR(i,j),2);
			}
	}
}

uint8_t SymC2Int (char ch)
{
	uint8_t n=0;
	for (uint8_t j=0;j<CodeSymR_Length();j++)
	{
		if (ch==CodeSymR(j))
		{
			n=j;
			break;
		}			
	}
	return(n);
}

void LCD_WStr(uint8_t x,uint8_t y,char str[])
{
	if (x<=57)
	{
		LCD_SetPage(y,1);
		LCD_SetAdress(x,1);
		for (uint8_t i=0;i<strlen(str);i++)
		{
			for (uint8_t j=0;j<6;j++)
			{
				WData(SmFR(SymC2Int(str[i])+96, j),1);
			}
		}
	}
	if(x>=64)
	{
		LCD_SetPage(y,2);
		LCD_SetAdress(x,2);
		for (uint8_t j=0;j<6;j++)
		{
			for (uint8_t i=0;i<strlen(str);i++)
			{
				WData(SmFR(SymC2Int(str[i])+97, j),2);
			}
		}
	}
}

void LCD_WNum (uint8_t x,uint8_t y, int32_t num)
{
	
	uint8_t m=0;
	uint8_t minus = 0;
	if (num<0) 
		{
			minus = 1;
			num = num * (-1);
		}
	int32_t n=num;
	while(n!=0)
	{
		n = n/10;
		m++;
	}
	if (x<=57)
	{
		LCD_SetPage(y,1);
		LCD_SetAdress(x,1);
		
		if (minus == 1)
		{
			for (uint8_t j=0;j<6;j++)
			{
				WData(SmFR(29, j),1);
			}
		}			
		
		for (uint8_t i=m;i>0;i--)
		{
			n=num/(pow(10,i-1));
			for (uint8_t j=0;j<6;j++)
			{
				WData(SmFR(n, j),1);
			}
			num=num%(uint32_t)(pow(10,i-1));
		}
		
	}
}
