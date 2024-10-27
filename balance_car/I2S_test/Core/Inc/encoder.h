#ifndef _ENCODER_H
#define _ENCODER_H

#include "tim.h"


#define ENCODER_PPR (4*442)
#define ENC_S16Abs(Value)(Value>0?Value:-Value)

typedef struct{
	TIM_HandleTypeDef *hEncTimer;
	int16_t EncCntVal;
	float EncSpeed;
}Encoder_HanderTypeDef;

extern volatile Encoder_HanderTypeDef Encoder_1;
extern volatile Encoder_HanderTypeDef Encoder_2;

void Encoder_Init(TIM_HandleTypeDef *htimer,volatile Encoder_HanderTypeDef *hEncoder);
void Encoder_1msPolling(volatile Encoder_HanderTypeDef *hEncoder);
float Encoder_GetSpeed(volatile Encoder_HanderTypeDef *hEncoder);
int16_t Encoder_GerContIn1ms(volatile Encoder_HanderTypeDef *hEncoder);
int16_t Encoder_GetConts(volatile Encoder_HanderTypeDef *hEncoder);

#endif
