#include "encoder.h"
#include "main.h"

static uint8_t Encoder_ReadCnt=0;
volatile Encoder_HanderTypeDef Encoder_1;
volatile Encoder_HanderTypeDef Encoder_2;


void Encoder_Init(TIM_HandleTypeDef *htimer,volatile Encoder_HanderTypeDef *hEncoder){
	hEncoder->hEncTimer = htimer;
	HAL_TIM_Encoder_Start(hEncoder->hEncTimer,TIM_CHANNEL_ALL);
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         

void Encoder_1msPolling(volatile Encoder_HanderTypeDef *hEncoder){
	if(Encoder_ReadCnt<10){
			Encoder_ReadCnt++;
	}else{
		Encoder_ReadCnt = 0;
		hEncoder->EncCntVal = hEncoder->hEncTimer->Instance->CNT;//__HAL_TIM_GetCounter(hEncoder->hEncTimer);
		hEncoder->hEncTimer->Instance->CNT = 0;
		hEncoder->EncSpeed = hEncoder->EncCntVal*100*60*1.0/ENCODER_PPR;// circle/min		
	}
}

float Encoder_GetSpeed(volatile Encoder_HanderTypeDef *Encoder){
	return Encoder->EncSpeed;
}

int16_t Encoder_GetConts(volatile Encoder_HanderTypeDef *Encoder){
	return Encoder->EncCntVal;
}
