#ifndef __CONTROL_H
#define __CONTROL_H

#include "encoder.h"

#define Foreward_Fast 1
#define Foreward_Slow 2
#define	Backward_Fast 3
#define Backward_Slow 4
#define Fast 5
#define Slow 6

#if 0
#define df_Bkp 150
#define df_Bkd 45
#define df_Vkp -11		//-7
#define df_Vki -0.4
#define df_Tkp 15
#define df_Tkd -5

#else
#define df_Bkp 140
#define df_Bkd 7	
#define df_Vkp -11		//-7
#define df_Vki -0.05
#define df_Tkp 15
#define df_Tkd -5

#endif

extern float balance_kp;
extern float balance_kd;
extern float velocity_kp;
extern float velocity_ki;
extern float turn_kp;
extern float turn_kd;
extern float target_speed;
extern float target_movement;
extern int8_t flag_left;
extern int8_t flag_right;
extern int8_t flag_fore;
extern int8_t flag_back;
extern int8_t flag_speed;
extern int8_t flag_movement;
extern float speed_integral;
extern float Mechanical_balance;
extern float angle_now;

extern int16_t SETTING;
extern int8_t TURN_ON;
extern int8_t PRINT_ON;
extern int8_t PRINT_P;
extern int8_t PRINT_I;
extern int8_t PRINT_D;


void control_motor(TIM_HandleTypeDef *htim,int PWM,uint8_t mode);
void output();

#endif
