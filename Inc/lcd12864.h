
// �����
#include "stm32f1xx_hal.h"
void Delay(volatile uint32_t nCount) {
    for (; nCount != 0; nCount--);
}

// ������ ����� � ��

void LcdByte(uint32_t byte)
{
	HAL_GPIO
	//GPIOC->ODR = byte;	   	// ����� � �����
	HAL_Delay(1);		   	// ����� 1 ���
	GPIOA->BSRR = GPIO_BSRR_BS2;    // ���������� �
	HAL_Delay(1);			   	// ����� 1 ���
	GPIOA->BSRR = GPIO_BSRR_BR2;	// ���������� �
	HAL_Delay(1);			   	// ����� 1 ���
}

void CodeL(uint32_t code)
{
	code |= 0x0900;    // �������� ��� ���� � �������� � ����� ��������
	LcdByte(code);
}

void CodeR(uint32_t code)
{
	code |= 0x1100;    // �������� ��� ���� � �������� � ������ ��������
	LcdByte(code);
}

void DataL(uint32_t code)
{
	code |= 0x2900;    // �������� ��� ������ � �������� � ����� ��������
 	LcdByte(code);
}

void DataR(uint32_t code)
{
	code |= 0x3100;    // �������� ��� ������ � �������� � ������ ��������
	LcdByte(code);
}

void LcdInit(void)
{
	GPIOC->BSRR = GPIO_BSRR_BR15;	// ���������� RES
	HAL_Delay(1);			// ����� 2 ���
	GPIOC->BSRR = GPIO_BSRR_BS15;    // ���������� RES
	HAL_Delay(1);			// ����� 10 ���
}
