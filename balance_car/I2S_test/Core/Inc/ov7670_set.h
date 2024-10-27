#ifndef __Ov7670_Set_H__
#define __Ov7670_Set_H__
#include "sccb.h"
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#define uint8 uint8_t
void init_ov7670reg(void);
uint8 ov7670_Init(void);
void OV7670_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);

#endif
