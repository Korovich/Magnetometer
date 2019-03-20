void LcdByte(uint32_t byte)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,byte>>12);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,byte>>11);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,byte>>10);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,byte>>9);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,byte>>8);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,byte>>7);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,byte>>6);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,byte>>5);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,byte>>4);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,byte>>3);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,byte>>2);
	byte=(byte<<1)>>1;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,byte>>1);
	HAL_Delay(1);		   	// Пауза 1 мкс
	GPIOA->BSRR = GPIO_BSRR_BS2;    // Выставляем Е
	HAL_Delay(1);			   	// Пауза 1 мкс
	GPIOA->BSRR = GPIO_BSRR_BR2;	// Сбрасываем Е
	HAL_Delay(1);			   	// Пауза 1 мкс
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

void DisplaySw(uint32_t byte)
{ 
		byte |= 0x03E;
		LcdByte(byte);
}

void LcdInit(void)
{
	GPIOC->BSRR = GPIO_BSRR_BR15;	// Сбрасываем RES
	HAL_Delay(1);			// Пауза 2 МКС
	GPIOC->BSRR = GPIO_BSRR_BS15;    // Выставляем RES
	HAL_Delay(1);			// Пауза 10 МКС
}
