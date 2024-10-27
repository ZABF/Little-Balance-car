#include "main.h"
#include "control.h"

int Moto1_pwm=0;
int Moto2_pwm=0;
float balance_kp = df_Bkp;
float balance_kd = df_Bkd;
float velocity_kp = df_Vkp;
float velocity_ki = df_Vki;
float turn_kp = df_Tkp;
float	turn_kd = df_Tkd;
float target_speed = 0;
float target_movement = 0;
int8_t flag_left = 0;
int8_t flag_right = 0;
int8_t flag_fore = 0;
int8_t flag_back = 0;
int8_t flag_speed = 0;
int8_t flag_movement = 0;
float speed_integral = 0; 
float Mechanical_balance = 0;
float angle_now = 0;

float last_target_speed=0;
float last_vkp=df_Vkp;

float actural_target_speed, actural_vkp;


int16_t SETTING = 0;
int8_t TURN_ON = 0;
int8_t PRINT_ON = 1;

void control_motor(TIM_HandleTypeDef *htim,int PWM,uint8_t mode){
	uint16_t IN1,IN2;
	if(PWM>0&&mode==Fast) mode = Foreward_Fast;				//1
	else if(PWM<=0&&mode==Fast) mode = Backward_Fast; //3
	else if(PWM>0&&mode==Slow) mode = Foreward_Slow;  //2
	else if(PWM<=0&&mode==Slow) mode = Backward_Slow; //4
	PWM=PWM>0?PWM:-PWM;
	if(htim==&htim2){
		IN1=TIM_CHANNEL_2;
		IN2=TIM_CHANNEL_1;
	}else{
		IN1=TIM_CHANNEL_3;
		IN2=TIM_CHANNEL_4;
	}
	if(mode==1|| mode==4){
		__HAL_TIM_SET_COMPARE(htim, IN1, PWM);
		if(mode==1){
			__HAL_TIM_SET_COMPARE(htim, IN2, 0);
		}else{
			__HAL_TIM_SET_COMPARE(htim, IN2, 6720);
		}
	}else{
		__HAL_TIM_SET_COMPARE(htim, IN2, PWM);
		if(mode==3){
			__HAL_TIM_SET_COMPARE(htim, IN1, 0);
		}else{
			__HAL_TIM_SET_COMPARE(htim, IN1, 6720);
		}
	}
}


int balance_pid(float angle, float gyro){
	static int Balance_pwm;
	Balance_pwm = balance_kp*angle+balance_kd*gyro;  //kp
	if(Balance_pwm<-6720/2) Balance_pwm = -6720/2;
	if(Balance_pwm>6720/2) Balance_pwm = 6720/2;
	return Balance_pwm;
}				


int velocity_pid(float speed_left, float speed_right, float angle){
	int Velocity_pwm;
	static int16_t speed_now, speed_filtered;//, speed_integral;
	
	if(last_target_speed - target_speed > 3){
		actural_target_speed = last_target_speed-3;
	}else if(last_target_speed - target_speed < -3){ 
		actural_target_speed = last_target_speed+3;
	}else actural_target_speed = target_speed;
	
	if(last_vkp - velocity_kp > 0.2){
		actural_vkp = last_vkp-0.2;
	}else if(last_vkp - velocity_kp < -0.2){
		actural_vkp = last_vkp+0.2;
	}else actural_vkp = velocity_kp;//平滑
	
	last_target_speed = actural_target_speed;
	last_vkp = actural_vkp;
	
	speed_now = speed_left + speed_right - actural_target_speed;
	speed_filtered = speed_filtered*0.8 + speed_now*0.2;
	speed_integral += speed_filtered;
	speed_integral = speed_integral-target_movement;
		
	
	if(speed_integral>10000) speed_integral=10000;
	if(speed_integral<-10000) speed_integral=-10000;	//积分限辐，0-6720
	
	
	Velocity_pwm = speed_filtered*actural_vkp + speed_integral*velocity_ki*(1 - flag_speed);
	if(angle>55||angle<-55||TURN_ON==0) speed_integral = 0;
	return Velocity_pwm;
}	

int turn_pid(float speed_left, float speed_right, float  gyro){
	int Turn_pwm = 0;
	float Turn_Amplitude=44;
	float kd = 0;
	static int Turn_count = 0,Encoder_temp = 0;
	static float Turn_Target = 0;
	static float Turn_Convert = 0.9;
		
	if(1==flag_left||1==flag_left)                      
		{
			if(++Turn_count==1)
			Encoder_temp = speed_left+speed_right;
			Encoder_temp = Encoder_temp>0? Encoder_temp :-Encoder_temp;
			Turn_Convert=200/Encoder_temp;
			if(Turn_Convert<0.6)Turn_Convert=0.6;
			if(Turn_Convert>3)Turn_Convert=3;
		}	
	  else
		{
			Turn_Convert=0.9;
			Turn_count=0;
			Encoder_temp=0;
		}

		if(1==flag_left)	           Turn_Target-=Turn_Convert;
		else if(1==flag_right)	     Turn_Target+=Turn_Convert; 
		else Turn_Target=0;
    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向	速度限幅
	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
		if(flag_fore==1||flag_back==1)  kd=turn_kd;        
		else kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
	
	Turn_pwm=-Turn_Target*turn_kp-gyro*kd;                 //===结合Z轴陀螺仪进行PD控制
	return Turn_pwm;
}	


void output(){
	static uint16_t Freq_print = 0;
	long data[3];
	int8_t accuracy;
	unsigned long timestamp; 
	float Pitch, Roll, Yaw, angle, gyroy, gyroz, gyrox, speed_left, speed_right;
	int Balance_pwm = 0;
	int Velocity_pwm = 0;
	int Turn_pwm = 0;
	
	inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t *)&timestamp);					
  gyrox = data[0] * 1.0 / (1 << 16);		
	gyroy = data[1] * 1.0 / (1 << 16);
	gyroz = data[2] * 1.0 / (1 << 16);
	
	inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp);					 
  Pitch = data[0] * 1.0 / (1 << 16);
	Pitch = Pitch>0?(180-Pitch):(-180-Pitch);
	angle = -(Pitch-3.3)-Mechanical_balance;
	
	speed_right = 0 - Encoder_GetSpeed(&Encoder_1);
	speed_left = Encoder_GetSpeed(&Encoder_2);
	
	Balance_pwm = balance_pid(angle, gyrox);
	Velocity_pwm = velocity_pid(speed_left,speed_right,angle);
	Turn_pwm = turn_pid(speed_left,speed_right,gyroz);
	angle_now = angle;

			if(angle>40||angle<-40||gyroy>3||gyroy<-3){
				control_motor(&htim2,0,Fast);
				control_motor(&htim5,0,Fast);
				Balance_pwm = 0;
				TURN_ON =0;
			}else{	
				Moto1_pwm = Balance_pwm-Velocity_pwm+Turn_pwm;                
				Moto2_pwm = Balance_pwm-Velocity_pwm-Turn_pwm;    
				if(Moto1_pwm<-6720/2 ) Moto1_pwm=-6720/2 ;
				if(Moto1_pwm>6720/2 )  Moto1_pwm=6720/2 ;
				if(Moto2_pwm<-6720/2 ) Moto2_pwm=-6720/2 ;
				if(Moto2_pwm>6720/2 )  Moto2_pwm=6720/2 ;
				Moto1_pwm = Moto1_pwm>0?Moto1_pwm+6720/2:Moto1_pwm-6720/2;
				Moto2_pwm = Moto1_pwm>0?Moto2_pwm+6720/2:Moto2_pwm-6720/2;
						
				if(TURN_ON){			
					control_motor(&htim2,Moto1_pwm,Fast);	
					control_motor(&htim5,Moto2_pwm,Fast);
					if(PRINT_ON>0) {
		//				printf("Balance_pwm = %d\t\tMoto1_PWM = %d\t\tMotol2_PWM = %d\n",Balance_pwm,Moto1_pwm,Moto2_pwm);
					}
				}	else {
						control_motor(&htim2,0,Fast);
						control_motor(&htim5,0,Fast);
				}
			}
				if(	FLAG_IMAGE_TRANSMITING == 0) {
//					printf("\nBkp = %f\nBkd = %f\nVkp = %f\nVki = %f\n",balance_kp,balance_kd,velocity_kp,velocity_ki);
//					printf("\nangle:%f\ngyro:%f\n",angle,gyro);
					Freq_print++;
					if(Freq_print>10){
					printf("Bkp = %.3f;Bkd = %.3f;Vkp = %.3f;Vki = %.3f;Tkp = %.3f;Tkd = %.3f;Angle:%.2f;bal_A:%.2f;tar_V:%.1f;tar_X:%.1f;int_V:%.1f;"
									,balance_kp,balance_kd
									,actural_vkp,velocity_ki
									,turn_kp,turn_kd
									,angle,Mechanical_balance
									,actural_target_speed,target_movement
									,speed_integral
					);
						
					printf("%.2f;%.2f;%d;%d;%d;%d;%d;[gyrox:%.3f][gyroy:%.3f][gyroz:%.3f] end"
									,speed_left,speed_right
									,Balance_pwm,Velocity_pwm,Turn_pwm,Moto1_pwm,Moto2_pwm
									,gyrox,gyroy,gyroz
					);
						
					Freq_print = 0;
					}
					
				}
}				
