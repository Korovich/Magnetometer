#include "usbd_cdc_if.h"
#include "usb_device.h"

#ifndef __ADS1220_H__
#define __ADS1220_H__

#ifdef __cplusplus
 extern "C" {
#endif

#define GPIO_RES_TYPE GPIOB
#define GPIO_RES_PIN GPIO_PIN_12
#define GPIO_DRDY_TYPE GPIOA
#define GPIO_DRDY_PIN GPIO_PIN_15
	 
#define Timeout_Talk (uint8_t)10

#define SPI_MASTER_DUMMY    0xFF
#define RDATA 							0x10
#define RESET               0x06    //Send the RESET command (06h) to make sure the ADS1220 is properly reset after power-up
#define START               0x08    //Send the START/SYNC command (08h) to start converting in continuous conversion mode
#define WREG  0x40
#define RREG  0x20

//Config registers
#define CONFIG_REG0_ADDRESS 0x00
#define CONFIG_REG1_ADDRESS 0x01
#define CONFIG_REG2_ADDRESS 0x02
#define CONFIG_REG3_ADDRESS 0x03

#define REG_CONFIG2_VOLT_MASK     0x3F
#define REG_CONFIG1_DR_MASK       0xE0
#define REG_CONFIG0_PGA_GAIN_MASK 0x0E
#define REG_CONFIG0_MUX_MASK      0xF0

#define DR_20SPS    0x00
#define DR_45SPS    0x20
#define DR_90SPS    0x40
#define DR_175SPS   0x60
#define DR_330SPS   0x80
#define DR_600SPS   0xA0
#define DR_1000SPS  0xC0

#define PGA_GAIN_1   0x00
#define PGA_GAIN_2   0x02
#define PGA_GAIN_4   0x04
#define PGA_GAIN_8   0x06
#define PGA_GAIN_16  0x08
#define PGA_GAIN_32  0x0A
#define PGA_GAIN_64  0x0C
#define PGA_GAIN_128 0x0E

#define VOLT_REF_INT     0x00
#define VOLT_REF_EXT_REF 0x40
#define VOLT_REF_EXT_AIN 0x80
#define VOLT_REF_ANALOG  0xC0

#define MUX_AIN0_AIN1   0x00
#define MUX_AIN0_AIN2   0x10
#define MUX_AIN0_AIN3   0x20
#define MUX_AIN1_AIN2   0x30
#define MUX_AIN1_AIN3   0x40
#define MUX_AIN2_AIN3   0x50
#define MUX_AIN1_AIN0   0x60
#define MUX_AIN3_AIN2   0x70
#define MUX_AIN0_AVSS   0x80
#define MUX_AIN1_AVSS   0x90
#define MUX_AIN2_AVSS   0xA0
#define MUX_AIN3_AVSS   0xB0

#define MUX_SE_CH0      0x80
#define MUX_SE_CH1      0x90
#define MUX_SE_CH2      0xA0
#define MUX_SE_CH3      0xB0

#define _BV(bit) (1<<(bit))

extern SPI_HandleTypeDef hspi2;

void ADS1220_Init(void);
void ADS1220_Start(void);
void ADS1220_Reset(void);

//void ADS1220_Command(unsigned char data_in);
//void ADS1220_WriteRegister(uint8_t address, uint8_t *value);
//uint8_t ADS1220_ReadRegister(uint8_t address);
//uint8_t * Read_Data(void);
int32_t ADS1220_Read_WaitForData(void); //
int32_t ADS1220_Read_Single_WaitForData(void); //

//uint8_t * ADS1220_Get_Config_REG(void);
void ADS1220_Volt_Ref (uint8_t Ref);
void ADS1220_PGA_OFF(void);
void ADS1220_PGA_ON(void);
void ADS1220_Contnuous_Mode(void);
void ADS1220_SingleShot_Mode(void);
void ADS1220_Set_Data_Rate(int datarate);
void ADS1220_Set_PGA_Gain(int pgagain);
void ADS1220_Sel_MUX_ch(int channels_conf);
void set_conv_mode_single_shot(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADS1220_H__*/
