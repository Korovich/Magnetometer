
// Пауза
#include "stm32f1xx_hal.h"
void Delay(volatile uint32_t nCount) {
    for (; nCount != 0; nCount--);
}

// Запись байта в ЖК

void LcdByte(uint32_t byte)
{
	HAL_GPIO
	//GPIOC->ODR = byte;	   	// Вывод в порту
	HAL_Delay(1);		   	// Пауза 1 мкс
	GPIOA->BSRR = GPIO_BSRR_BS2;    // Выставляем Е
	HAL_Delay(1);			   	// Пауза 1 мкс
	GPIOA->BSRR = GPIO_BSRR_BR2;	// Сбрасываем Е
	HAL_Delay(1);			   	// Пауза 1 мкс
}

void CodeL(uint32_t code)
{
	code |= 0x0900;    // Побитное ИЛИ кода с командой в левый кристалл
	LcdByte(code);
}

void CodeR(uint32_t code)
{
	code |= 0x1100;    // Побитное ИЛИ кода с командой в правый кристалл
	LcdByte(code);
}

void DataL(uint32_t code)
{
	code |= 0x2900;    // Побитное ИЛИ данных с командой в левый кристалл
 	LcdByte(code);
}

void DataR(uint32_t code)
{
	code |= 0x3100;    // Побитное ИЛИ данных с командой в правый кристалл
	LcdByte(code);
}

void LcdInit(void)
{
	GPIOC->BSRR = GPIO_BSRR_BR15;	// Сбрасываем RES
	HAL_Delay(1);			// Пауза 2 МКС
	GPIOC->BSRR = GPIO_BSRR_BS15;    // Выставляем RES
	HAL_Delay(1);			// Пауза 10 МКС
}
