#include "bq25895.h"


uint8_t BQ25895Reg[21];
uint8_t BQ25895TempData[1];
uint8_t PowerState;


extern I2C_HandleTypeDef hi2c1;



void BQ25895_Init(void)
{ 
	HAL_GPIO_WritePin(CHG_CE_GPIO_Port,CHG_CE_Pin,GPIO_PIN_RESET);
	BQ25895_Write(0x02,0x7d);//ADC����ת��
	BQ25895_Write(0x03,0x1e);//OTG�رգ���Сϵͳ��ѹ����Ϊ3.5V
	BQ25895_Write(0x05,0x10);//���ضϵ�������Ϊ64mA
	BQ25895_Write(0x07,0x8d);//�رտ��Ź�
}  

uint8_t temp1;
void BQ25895_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
{ 	
	while(hi2c1.State != HAL_I2C_STATE_READY){;}
	HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, ReadAddr,I2C_MEMADD_SIZE_8BIT, pBuffer,1);
}  

void BQ25895_MultiRead(uint8_t* pBuffer)   
{ 
	while(hi2c1.State != HAL_I2C_STATE_READY){;}	
	HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, 0x00,I2C_MEMADD_SIZE_8BIT, pBuffer,0x14);
}  
 
void BQ25895_Write(uint8_t WriteAddr,uint8_t WriteData)
{
	while(hi2c1.State != HAL_I2C_STATE_READY){;}	
	BQ25895TempData[0]=WriteData;
	HAL_I2C_Mem_Write_DMA(&hi2c1, BQ25895Address, WriteAddr,I2C_MEMADD_SIZE_8BIT, BQ25895TempData, 1);
} 

void PowerStateUpdate(void)
{
	uint8_t CHRG_STAT;
          	CHRG_STAT=(BQ25895Reg[0x0b]&0x18)>>3;
			 
	if(CHRG_STAT==1||CHRG_STAT==2)//Pre-charge Fast Charging
	{
		PowerState=1;
	}
	else if(CHRG_STAT==3)//Charge Termination Done
	{
		PowerState=2;
	}
	else if(CHRG_STAT==0)//Not Charging
	{
		PowerState=3;
	}
	//printf("PowerState: %u\n", (unsigned int)PowerState);
}


