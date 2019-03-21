
#ifndef __LIBSYM_H
#define __LIBSYM_H

#ifdef __cplusplus
extern "C" {
#endif
	
	#include "stm32f1xx_hal.h"
	#include "string.h"

	uint32_t CodeSymR_Length (void);
	char CodeSymR (uint32_t i);
	uint8_t SmFR (uint32_t i, uint32_t j);
	
	#ifdef __cplusplus
}
#endif

#endif /* __LIBSYM_H__*/
