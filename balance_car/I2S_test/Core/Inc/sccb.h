#ifndef __SCCB_H__
#define __SCCB_H__

#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#define uint8 uint8_t 

void delay_us(unsigned int);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_NA(void);
uint8 WR_Byte(uint8 dat);
uint8 RD_Byte(void);
uint8 WR_Reg(uint8 reg,uint8 a);
uint8 RD_Reg(uint8 reg);

#endif
