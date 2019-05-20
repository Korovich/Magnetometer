#include "ADS1220.h"

uint8_t m_config_reg0=0;
uint8_t m_config_reg1=0;
uint8_t m_config_reg2=0;
uint8_t m_config_reg3=0;

uint8_t Config_Reg0;
uint8_t Config_Reg1;
uint8_t Config_Reg2;
uint8_t Config_Reg3;
 
int ADS1220WaitForDataReady(int Timeout)
{
   if (Timeout > 0)
   {
      /* wait for /DRDY = 1 */
      while (!(HAL_GPIO_ReadPin(GPIO_DRDY_TYPE,GPIO_DRDY_PIN) == GPIO_PIN_RESET) && (Timeout-- >= 0));
      /* wait for /DRDY = 0 */
      while ( (HAL_GPIO_ReadPin(GPIO_DRDY_TYPE,GPIO_DRDY_PIN) == GPIO_PIN_RESET) && (Timeout-- >= 0))         ;
      if (Timeout < 0)
         return 0; /* ADS1220_TIMEOUT_WARNING; */
   }
   else
   {
      /* wait for /DRDY = 1 */
      while (!(HAL_GPIO_ReadPin(GPIO_DRDY_TYPE,GPIO_DRDY_PIN) == GPIO_PIN_RESET));
      /* wait for /DRDY = 0 */
      while ( (HAL_GPIO_ReadPin(GPIO_DRDY_TYPE,GPIO_DRDY_PIN) == GPIO_PIN_RESET));
   }
   return ADS1220_NO_ERROR;
}
 
void ADS1220_WriteRegister (uint8_t address, uint8_t *value)
{
	uint8_t buf = WREG | (address << 2);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi2,&buf,1,Timeout_Talk);
	HAL_SPI_Transmit(&hspi2,value,1,Timeout_Talk);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_SET);
}

uint8_t ADS1220_ReadRegister (uint8_t address)
{
	uint8_t buf = RREG | (address << 2);
	uint8_t data;
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi2,&buf,1,Timeout_Talk);
	HAL_Delay(5);
	HAL_SPI_Receive(&hspi2,&data,1,Timeout_Talk);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_SET);
	return data;
}

void ADS1220_Command( unsigned char data)
{
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_SET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_SPI_Transmit(&hspi2,&data,1,Timeout_Talk);
	HAL_Delay(2);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_SET);
}

void ADS1220_Init ()
{
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_RESET);
	HAL_Delay(100);
	ADS1220_Reset();
	HAL_Delay(100);
	m_config_reg0 = 0x00;   //Default settings: AINP=AIN0, AINN=AIN1, Gain 1, PGA enabled
  m_config_reg1 = 0x04;   //Default settings: DR=20 SPS, Mode=Normal, Conv mode=continuous, Temp Sensor disabled, Current Source off
  m_config_reg2 = 0x10;   //Default settings: Vref internal, 50/60Hz rejection, power open, IDAC off
  m_config_reg3 = 0x00;   //Default settings: IDAC1 disabled, IDAC2 disabled, DRDY pin only
	
	ADS1220_WriteRegister( CONFIG_REG0_ADDRESS , &m_config_reg0);
  ADS1220_WriteRegister( CONFIG_REG1_ADDRESS , &m_config_reg1);
  ADS1220_WriteRegister( CONFIG_REG2_ADDRESS , &m_config_reg2);
  ADS1220_WriteRegister( CONFIG_REG3_ADDRESS , &m_config_reg3);
	
	HAL_Delay(100);
	
	Config_Reg0 = ADS1220_ReadRegister(CONFIG_REG0_ADDRESS);
  Config_Reg1 = ADS1220_ReadRegister(CONFIG_REG1_ADDRESS);
  Config_Reg2 = ADS1220_ReadRegister(CONFIG_REG2_ADDRESS);
  Config_Reg3 = ADS1220_ReadRegister(CONFIG_REG3_ADDRESS);
	
	CDC_Transmit_FS((uint8_t*)"Config : \n",11);
	CDC_Transmit_FS(&Config_Reg0,1);
	CDC_Transmit_FS((uint8_t*)"\n",2);
	CDC_Transmit_FS(&Config_Reg1,1);
	CDC_Transmit_FS((uint8_t*)"\n",2);
	CDC_Transmit_FS(&Config_Reg2,1);
	CDC_Transmit_FS((uint8_t*)"\n",2);
	CDC_Transmit_FS(&Config_Reg3,1);
	CDC_Transmit_FS((uint8_t*)"\n",2);
	
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_SET);
	
	HAL_Delay(100);
}

void ADS1220_Reset ()
{
	ADS1220_Command(RESET);
}

void ADS1220_Start ()
{
	ADS1220_Command(START);
}
void ADS1220_Volt_Ref (uint8_t Ref)
{
	m_config_reg2 &= ~REG_CONFIG2_VOLT_MASK;
  m_config_reg2 |= Ref;
  ADS1220_WriteRegister(CONFIG_REG2_ADDRESS,&m_config_reg2);
}

void ADS1220_PGA_ON ()
{
	m_config_reg0 &= ~_BV(0);
	ADS1220_WriteRegister(CONFIG_REG0_ADDRESS,&m_config_reg0);
}

void ADS1220_PGA_OFF ()
{
	m_config_reg0 |= _BV(0);
	ADS1220_WriteRegister(CONFIG_REG0_ADDRESS,&m_config_reg0);
}

void ADS1220_Contnuous_Mode ()
{
	m_config_reg1 |= _BV(2);
	ADS1220_WriteRegister(CONFIG_REG1_ADDRESS,&m_config_reg1);
}

void ADS1220_SingleShot_Mode ()
{
	m_config_reg1 &= ~_BV(2);
	ADS1220_WriteRegister(CONFIG_REG1_ADDRESS,&m_config_reg1);
}

void ADS1220_Set_Data_Rate ( int datarate)
{
	m_config_reg1 &= ~REG_CONFIG1_DR_MASK;
  m_config_reg1 |= datarate;
  ADS1220_WriteRegister(CONFIG_REG1_ADDRESS,&m_config_reg1);
}

void ADS1220_Sel_MUX_ch ( int channels_conf)
{
	m_config_reg0 &= ~REG_CONFIG0_MUX_MASK;
  m_config_reg0 |= channels_conf;
  ADS1220_WriteRegister(CONFIG_REG0_ADDRESS,&m_config_reg0);
}

void ADS1220_Set_PGA_Gain(int pgagain)
{
    m_config_reg0 &= ~REG_CONFIG0_PGA_GAIN_MASK;
    m_config_reg0 |= pgagain ;
    ADS1220_WriteRegister(CONFIG_REG0_ADDRESS,&m_config_reg0);
}

uint8_t * ADS1220_Get_Config_REG()
{
    static uint8_t config_Buff[4];

    m_config_reg0 = ADS1220_ReadRegister(CONFIG_REG0_ADDRESS);
    m_config_reg1 = ADS1220_ReadRegister(CONFIG_REG1_ADDRESS);
    m_config_reg2 = ADS1220_ReadRegister(CONFIG_REG2_ADDRESS);
    m_config_reg3 = ADS1220_ReadRegister(CONFIG_REG3_ADDRESS);

    config_Buff[0] = m_config_reg0 ;
    config_Buff[1] = m_config_reg1 ;
    config_Buff[2] = m_config_reg2 ;
    config_Buff[3] = m_config_reg3 ;

    return config_Buff;
}

int32_t ADS1220_Read_WaitForData(void)
{
	static uint8_t SPI_Buf[3];
	int32_t mResult32=0;
	
	while(HAL_GPIO_ReadPin(GPIO_DRDY_TYPE,GPIO_DRDY_PIN) == GPIO_PIN_SET)
	
	ADS1220_Command(RDATA);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_RESET);
	HAL_Delay(10);
	for (uint8_t i=0;i<3;i++)
	{
		HAL_SPI_Receive(&hspi2,&SPI_Buf[i],1,Timeout_Talk);
	}
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIO_RES_TYPE,GPIO_RES_PIN,GPIO_PIN_SET);
	mResult32 |= SPI_Buf[2];
	mResult32 |= SPI_Buf[1]<<8;
	mResult32 |= SPI_Buf[0]<<16;
	return mResult32;
}

int32_t ADS1220_Read_Single_WaitForData(void)
{
	static uint8_t SPI_Buf[3];
	int32_t mResult32=0;
	
	ADS1220_Start();
	
	while(HAL_GPIO_ReadPin(GPIO_DRDY_TYPE,GPIO_DRDY_PIN) == GPIO_PIN_SET){}
	
	HAL_Delay(RDATA_TIMEOUT);
	ADS1220_Command(RDATA);
		
	//HAL_SPI_Transmit(&hspi2,0x00,1,Timeout_Talk);
	for (uint8_t i=0;i<3;i++)
	{
		HAL_SPI_Receive(&hspi2,&SPI_Buf[i],1,Timeout_Talk);
	}
			
	mResult32 = SPI_Buf[0];
	mResult32 = (mResult32 << 8) |SPI_Buf[1];
	mResult32 = (mResult32 << 8) |SPI_Buf[2];
 
	// sign extend data
	if (mResult32 & 0x800000)
			mResult32 |= 0xff000000;
	return mResult32;
}
