#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#define delay_us DWT_Delay_us
#define uint8 uint8_t 
#define SCCB_ID		0x42 //ov7670的地址


//sbit SCCB_SCL=P0^1;
//sbit SCCB_SDA=P0^0;
//sbit SDA_STATE=P0^0;

void SDA_IN(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void SDA_OUT(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);	
}


void SCCB_Start(void)
{		
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);    
    delay_us(100);

    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);	   
    delay_us(100);
 
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
    delay_us(100);

    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);	 
    delay_us(100);
}

 void SCCB_Stop(void)
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
    delay_us(100);
 
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
    delay_us(100);  

    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
    delay_us(100);
}

void SCCB_NA(void)
{
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	delay_us(100);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
	delay_us(100);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
	delay_us(100);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	delay_us(100);

}

uint8 WR_Byte(uint8 dat)
{
	unsigned char j,tem;

	for(j=0;j<8;j++) 
	{
		if((dat<<j)&0x80)
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		}
		delay_us(100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
		delay_us(100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);;
		delay_us(100);

	}
	delay_us(100);
	SDA_IN();
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
	delay_us(100);
	if(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))
	{
		tem=1;
//		Prints("fail  ");     //1失败
	}
	else
	{
		tem=0;
//		Prints("success  ");      //0成功
	}
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
	delay_us(100);	
    SDA_OUT();

	return(tem);  
}


uint8 RD_Byte(void){
 	uint8 temp=0,j;
	SDA_IN();
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
	for(j=8;j>0;j--){
	 	delay_us(50);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);	 //scl拉高才能读取sda
		temp=temp<<1;
		if(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))temp++;	//读到temp中
		delay_us(50);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
	}
	SDA_OUT();
	return temp;
}

uint8 WR_Reg(unsigned char regID, unsigned char regDat){
	SCCB_Start();
	if(1==WR_Byte(0x42)){
		SCCB_Stop();
		printf("fail_01_0x42_%#x_%#x \n",regID,regDat);
		return(1);
	}
	delay_us(100);
  if(1==WR_Byte(regID)){
		SCCB_Stop();
		printf("fail_02_0x42_%#x_%#x \n",regID,regDat);
		return(1);
	}
	delay_us(100);
  if(1==WR_Byte(regDat)){
		SCCB_Stop();
		printf("fail_03_0x42_%#x_%#x \n",regID,regDat);
		return(1);
	}
  SCCB_Stop();	
  return(0);
}


uint8 RD_Reg(uint8 reg){
	uint8 val=0;
	SCCB_Start();
	if(0==WR_Byte(SCCB_ID))//Prints("WriteIDSuccess");else Prints("WriteIDFail");
	delay_us(100);
	if(0==WR_Byte(reg))//Prints("WriteRegSuccess");else Prints("WriteRegFail");
	delay_us(100);
	SCCB_Stop();
	delay_us(100);
	SCCB_Start();
	if(0==WR_Byte(SCCB_ID|0x01))//Prints("WriteID|01Success");else Prints("WriteID|01Fail");
	delay_us(100);
	val=RD_Byte();
	SCCB_NA();
	SCCB_Stop();
	return val;
}

