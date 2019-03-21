#include "LibSym.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "math.h"

#ifndef __LCD12864MT_H__
#define __LCD12864MT_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
void LCD_SetPage(uint32_t code,uint8_t crystal);
void LCD_SetAdress(uint32_t code,uint8_t crystal);
void LCD_Clear(uint8_t crystal);
void LCD_Init(void);
void LCD_WSym (uint8_t x, uint8_t y, uint8_t i);
void LCD_WStr(uint8_t x,uint8_t y,char str[]);
void LCD_WNum (uint8_t x,uint8_t y, int32_t num);
	 
#ifdef __cplusplus
}
#endif

#endif /* __LCD12864MT_H__*/
