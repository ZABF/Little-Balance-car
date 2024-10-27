/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* mpu6050 DMP�?  end */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint32_t ID;
	uint32_t Size;
	uint32_t Type;
}RIFF_chunk;

typedef struct{
	uint32_t ID;
	uint32_t Size;
	uint16_t AudioFormat;
	uint16_t NumChannels;
	uint32_t SampleRate;
	uint32_t ByteRate;
	uint16_t BlockAlign;
	uint16_t BitsPerSample;
}Format_chunk;

typedef struct{
	uint32_t ID;
	uint32_t Size;
	uint8_t *Data;
}Data_chunk;

typedef struct{
	RIFF_chunk		riff;
	Format_chunk	format;
	Data_chunk		data;
}File_WAV;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
	
#define R_SIZE 55   //6*9+1=55
	
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t dma[4];
uint16_t val16;
uint32_t val24;
int val32;
volatile uint8_t temp0[1];//1024
volatile uint8_t temp1[1];//1024
volatile uint8_t nexttemp=0;
int num=0;

volatile uint32_t hal_timestamp = 0;

uint8_t uart_rx[R_SIZE];
int Uart1_Rx_Cnt = 0;

volatile uint8_t UART_OVER_FLAG;
volatile uint8_t DCMI_OVER_FLAG;
volatile uint8_t	FLAG_MAIN_INITIAL = 0;
volatile uint8_t	FLAG_IMAGE_TRANSMITING = 0;
volatile uint8_t  FLAG_IMAGE_RGB = 0;
volatile uint8_t	FLAG_IMAGE_BIN = 1;

uint8_t rec[pic_width*pic_length*2];
uint8_t rec_b_buffer[pic_width*pic_length*2/8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

double word_to_double(uint8_t* string, uint8_t length){
	double rxDouble=0;
	uint8_t negtimes = 0;
	uint8_t postimes = 0;
	uint8_t position=0;
	uint8_t pot_position=length;
	for(position=0;position<length;position++){
		if(string[position]=='-') negtimes++;
		else if(string[position]=='+') postimes++;
		else if(string[position]=='.') pot_position = position+1;
		else rxDouble = rxDouble*10 +(string[position]-48);
	}
	rxDouble=rxDouble*1.0*pow(0.1,(length-pot_position));
	if(negtimes%2) rxDouble = -rxDouble;
	return rxDouble;	
	
}

void usart_send_char(uint8_t c){
    HAL_UART_Transmit(&huart3, &c, 1, 0xff);
    // while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET)
    // USART_SendData(DEBUG_USARTx, c);
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){
	if(hi2s==&hi2s3){
/*	static unsigned cb_cnt=0;
		cb_cnt++;
		val24=(dma[0]<<8)+(dma[1]>>8);
		if(val24 & 0x800000){//negative
			val32=0xff000000 | val24;
		}else{//positive
			val32=val24;
		}
		if(cb_cnt%10==0)
			printf("%d\r\n",val32);	
*/
		if(nexttemp==0){
			temp0[num*2]=dma[0];
			temp0[num*2+1]=dma[0]>>8;
			if(num==100){
				num=0;
				HAL_UART_Transmit_DMA(&huart3,(uint8_t*)temp0,200);
				nexttemp=1;
			}else num++;
		}else{
			temp1[num*2]=dma[0];
			temp1[num*2+1]=dma[0]>>8;
			if(num==100){
				num=0;
				HAL_UART_Transmit_DMA(&huart3,(uint8_t*)temp1,200);
				nexttemp=0;
			}else num++;
		}
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	static float velocity_kp_temp = df_Vkp;
	UNUSED(huart);	//uart_rx[]='C'+"OONN/OOFF"='S'+9+9+9+9
	
	if(uart_rx[0] == 'C'){
		if(*(uint32_t *)&uart_rx[1] == 'NNOO')//OONN
			TURN_ON = 1;
			
		if(*(uint32_t *)&uart_rx[1] == 'FFOO'){//OOFF
			TURN_ON = 0;
			control_motor(&htim2,0,Fast);
			control_motor(&htim5,0,Fast);
		}	
		if(*(uint32_t *)&uart_rx[1] == 'POTS'){//STOP
			target_speed = 0;
			target_movement = 0;
			if(flag_movement==1){}else speed_integral = 0;//速度控制、转向结束积分清零
			flag_left = 0;
			flag_right = 0;
			flag_fore = 0;
			flag_back = 0;
			flag_speed = 0;
			flag_movement = 0;
//			speed_integral = 0;
			velocity_kp = velocity_kp_temp;
		}
		if(*(uint32_t *)&uart_rx[1] == 'WROF'){//FORW
			target_speed = word_to_double(&uart_rx[6],uart_rx[5]-48);
			if(target_speed != 0) flag_speed = 1;
			else flag_speed = 0;
			
			target_movement = word_to_double(&uart_rx[15],uart_rx[14]-48);
			
			if(target_movement != 0){
				velocity_kp = -6;
				flag_movement = 1;
			}else{
				velocity_kp = velocity_kp_temp;
				flag_movement = 0;
			}
			flag_fore = 1;
		}
		if(*(uint32_t *)&uart_rx[1] == 'KCAB'){//BACK
			target_speed = -word_to_double(&uart_rx[6],uart_rx[5]-48);
			if(target_speed != 0) flag_speed = 1;
			else flag_speed = 0;
			
			target_movement = -word_to_double(&uart_rx[15],uart_rx[14]-48);	
			if(target_movement != 0){
				velocity_kp = -6;
				flag_movement = 1;
			}else{
				velocity_kp = velocity_kp_temp;	//模糊pid
				flag_movement = 0;
			}
			flag_back = 1;
		}
		if(*(uint32_t *)&uart_rx[1] == 'RNUT')//TUNR
			flag_right = 1;
		if(*(uint32_t *)&uart_rx[1] == 'LNUT')//TUNL
			flag_left = 1;
		if(*(uint32_t *)&uart_rx[1] == 'ALAB')//BALA
			Mechanical_balance = Mechanical_balance + angle_now;
		if(*(uint32_t *)&uart_rx[1] == '1GMI')//IMG1	
			FLAG_IMAGE_TRANSMITING = 1;
		if(*(uint32_t *)&uart_rx[1] == '0GMI')//IMG0
			FLAG_IMAGE_TRANSMITING = 0;
		if(*(uint32_t *)&uart_rx[1] == 'IBGR'){//RGBI
			FLAG_IMAGE_RGB = 1;
			FLAG_IMAGE_BIN = 0;
		}
		if(*(uint32_t *)&uart_rx[1] == 'INIB'){//BINI
			FLAG_IMAGE_BIN = 1;
			FLAG_IMAGE_RGB = 0;
		}
	}
	
	if(uart_rx[0] == 'S'){
		balance_kp=word_to_double(&uart_rx[2],uart_rx[1]-48);
		balance_kd=word_to_double(&uart_rx[11],uart_rx[10]-48);
		velocity_kp=word_to_double(&uart_rx[20],uart_rx[19]-48);
		velocity_ki=word_to_double(&uart_rx[29],uart_rx[28]-48);
		turn_kp=word_to_double(&uart_rx[38],uart_rx[37]-48);
		turn_kd=word_to_double(&uart_rx[47],uart_rx[46]-48);
		velocity_kp_temp = velocity_kp;
	}
	
	memset(uart_rx,0x00,sizeof(uart_rx));
	HAL_UART_Receive_IT(&huart3,uart_rx,R_SIZE);
}




extern struct inv_sensor_cal_t sensors;
static void send_to_pc(void){
	  long msg, data[9];
    int8_t accuracy;
    float float_data[3] = {0};
        unsigned long timestamp, step_count, walk_time;

        /*获取euler angle*/
        if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp))
        {
            float Pitch, Roll, Yaw;
            Pitch = data[0] * 1.0 / (1 << 16);
            Roll = data[1] * 1.0 / (1 << 16);
            Yaw = data[2] * 1.0 / (1 << 16);

            /*向匿名上位机发姿态�?�加速度、陀螺仪信息*/
            Data_Send_Status(Pitch, Roll, Yaw,(int16_t *)&sensors.gyro.raw,(int16_t *)&sensors.accel.raw);
            /*向匿名上位机发原始数据?*/
//            Send_Data((int16_t *)&sensors.gyro.raw, (int16_t *)&sensors.accel.raw);
        }

        /*获取步数*/
        get_tick_count(&timestamp);
        if (timestamp > hal.next_pedo_ms)
        {

            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);
        }
}

static void read_from_mpl(void){
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};
		static int read_time = 0;
		read_time++;
		if(read_time<1){
			;
		}else {
			read_time=0;

    //    MPU_DEBUG_FUNC();
    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t *)&timestamp))
    {
        /* Sends a quaternion packet to the PC. Since this is used by the Python
         * test app to visually represent a 3D quaternion, it's sent each time
         * the MPL has new data.
         */
 //       eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
        if (hal.report & PRINT_QUAT){
						float X,Y,Z,W;
            W = data[0] * 1.0 / (1 << 30);
            X = data[1] * 1.0 / (1 << 30);
            Y = data[2] * 1.0 / (1 << 30);
						Z = data[3] * 1.0 / (1 << 30);
						printf("W X Y Z:\n%7.3f\t%7.3f\t%7.3f\t%7.3f\t\r\n\n",W,X,Y,Z);
      //      eMPL_send_data(PACKET_DATA_QUAT, data);
				}
    }

    if (hal.report & PRINT_ACCEL)
    {
        if (inv_get_sensor_type_accel(data, &accuracy, (inv_time_t *)&timestamp)){
					printf("Acceleration (g's) in body frame:\n%7.3f\t%7.3f\t%7.3f\t\n\n",data[0] * 1.0 / (1 << 16),
									data[1] * 1.0 / (1 << 16),data[2] * 1.0 / (1 << 16));
				}
           // eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    if (hal.report & PRINT_GYRO)
    {
        if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t *)&timestamp))
            printf("Angular velocity (deg/s) in body frame:\n%7.3f\t%7.3f\t%7.3f\t\n\n",data[0] * 1.0 / (1 << 16),
									data[1] * 1.0 / (1 << 16),data[2] * 1.0 / (1 << 16));
				//eMPL_send_data(PACKET_DATA_GYRO, data);
    }
#ifdef COMPASS_ENABLED
    if (hal.report & PRINT_COMPASS)
    {
        if (inv_get_sensor_type_compass(data, &accuracy, (inv_time_t *)&timestamp))
            eMPL_send_data(PACKET_DATA_COMPASS, data);
    }
#endif
    if (hal.report & PRINT_EULER)
    {
        if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp)){
					  float Pitch, Roll, Yaw;
            Pitch = data[0] * 1.0 / (1 << 16);
            Roll = data[1] * 1.0 / (1 << 16);
            Yaw = data[2] * 1.0 / (1 << 16);
						printf("Pitch Roll Yaw:\n%7.3f\t%7.3f\t%7.3f\t\r\n\n",Pitch,Roll,Yaw);
				}
					
       
    }
    if (hal.report & PRINT_ROT_MAT)
    {
        if (inv_get_sensor_type_rot_mat(data, &accuracy, (inv_time_t *)&timestamp))
            printf("Body-to-world frame rotation matrix:\n%7.3f\t%7.3f\t%7.3f\t\n%7.3f\t%7.3f\t%7.3f\t\n%7.3f\t%7.3f\t%7.3f\t\n\n"
										,data[0] * 1.0 / (1 << 30),data[1] * 1.0 / (1 << 30),data[2] * 1.0 / (1 << 30)
										,data[3] * 1.0 / (1 << 30),data[4] * 1.0 / (1 << 30),data[5] * 1.0 / (1 << 30)
										,data[6] * 1.0 / (1 << 30),data[7] * 1.0 / (1 << 30),data[8] * 1.0 / (1 << 30));					
    //      eMPL_send_data(PACKET_DATA_ROT, data);
    }
    if (hal.report & PRINT_HEADING)
    {
        if (inv_get_sensor_type_heading(data, &accuracy, (inv_time_t *)&timestamp))
					printf("Quaternion-derived heading:\n%7.3f\t\n\n", data[0] * 1.0 / (1 << 16));
            //eMPL_send_data(PACKET_DATA_HEADING, data);
    }
    if (hal.report & PRINT_LINEAR_ACCEL)
    {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t *)&timestamp))
        {
            printf("Linear Accel:\n%7.3f\t%7.3f\t%7.3f\t\r\n\n", float_data[0], float_data[1], float_data[2]);
        }
    }
    if (hal.report & PRINT_GRAVITY_VECTOR)
    {
        if (inv_get_sensor_type_gravity(float_data, &accuracy, (inv_time_t *)&timestamp))
            printf("Gravity Vector:\n%7.3f\t%7.3f\t%7.3f\t\r\n\n", float_data[0], float_data[1], float_data[2]);
    }
    if (hal.report & PRINT_PEDO)
    {
        unsigned long timestamp;
        get_tick_count(&timestamp);
        if (timestamp > hal.next_pedo_ms)
        {
            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
            unsigned long step_count, walk_time;
            dmp_get_pedometer_step_count(&step_count);
            dmp_get_pedometer_walk_time(&walk_time);
            printf("Walked %ld steps over %ld milliseconds..\n\n", step_count, walk_time);
        }
    }

    /* Whenever the MPL detects a change in motion state, the application can
     * be notified. For this example, we use an LED to represent the current
     * motion state.
     */
    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
    if (msg)
    {
        if (msg & INV_MSG_MOTION_EVENT)
        {
            printf("Motion!\n");
        }
        else if (msg & INV_MSG_NO_MOTION_EVENT)
        {
            printf("No motion!\n");
        }
    }
	}//read_time else
}



/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)

	{
  /* USER CODE BEGIN 1 */
	uint16_t pwm_1=0;
	uint16_t pwm_2=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2S3_Init();
//  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//	HAL_I2S_Receive_DMA(&hi2s3,(uint16_t*)dma,4);//i2s
//	WAVfile_Init(&file1,0xffffffff-36,0);	//wav文件

		unsigned char new_temp = 0;
    unsigned long timestamp;
    MPU6050_mpu_init();
    MPU6050_mpl_init();
    MPU6050_config();
		hal.report = PRINT_GYRO|PRINT_EULER;
		/*
				PRINT_ACCEL     (0x01) 
				PRINT_GYRO      (0x02) 
				PRINT_QUAT      (0x04) 
				PRINT_COMPASS 	(0x08) 
				PRINT_EULER     (0x10) 
				PRINT_ROT_MAT   (0x20) 
				PRINT_HEADING   (0x40)
				PRINT_PEDO      (0x80)
				PRINT_LINEAR_ACCEL (0x100)
				PRINT_GRAVITY_VECTOR (0x200)
		*/
		
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);  //�?启PWM2通道1
  	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);  //�?启PWM2通道2
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);  //�?启PWM2通道3
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);  //�?启PWM2通道4

		Encoder_Init(&htim3,&Encoder_1);
		Encoder_Init(&htim4,&Encoder_2);

		HAL_UART_Receive_IT(&huart3,uart_rx,R_SIZE);//(1+8)*4

		ov7670_Init();
		UART_OVER_FLAG=0;
//////////////////////////////////////////////////////////////////1帧图片:1.设置图片长宽，2设置摄像头输出，3开辟数组使pPic指向数组，4启动dcmi//	uint8_t rec[Pic.length*Pic.width*2];//使用动态开辟空间，使用柔性数组，扩大heap内存
	Picture Pic={pic_length,pic_width,rec};
	OV7670_Window_Set(160,10,Pic.length,Pic.width);
	
	HAL_DCMI_Start_DMA(&hdcmi,DCMI_MODE_SNAPSHOT,(uint32_t)rec,Pic.length*Pic.width*2/4);//(320*240*2)/4/4);//uint8 [320*240*2]=[640*24*10]
	
	
	FLAG_MAIN_INITIAL = 1;
	printf("[ININTIAL_DONE]");	
    while (1)
    {	
			while(FLAG_IMAGE_TRANSMITING == 0);
			while(DCMI_OVER_FLAG==0);
			DCMI_OVER_FLAG=0;
			/* 图像处理 开始 */

			if(FLAG_IMAGE_BIN == 1){
			Img_Binarize_bit(&Pic,rec_b_buffer);		
			
			/* 图像处理 结束 */
//			HAL_UART_Transmit_DMA(&huart3,rec,Pic.width*Pic.length*2/8);
//			while(UART_OVER_FLAG==0);
//			printf("ThisIsTheFrameHeader");
			for(int i = 0; i<Pic.width*Pic.length/8/200;i++){
				HAL_UART_Transmit(&huart3,&rec_b_buffer[i*200],200,1000);
				HAL_Delay(100);  
		}
			
		}else if(FLAG_IMAGE_RGB == 1){

			for(int i = 0; i<Pic.width*Pic.length*2/200;i++){
				HAL_UART_Transmit(&huart3,&rec[i*200],200,1000);
				
			DWT_Delay_ms(50);
			}
			
		}
		
			UART_OVER_FLAG=0;
			HAL_DCMI_Start_DMA(&hdcmi,DCMI_MODE_SNAPSHOT,(uint32_t)rec,Pic.width*Pic.length*2/4);
		
		}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
void get_tick_count (unsigned long *count){
	*count = HAL_GetTick();
} 

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart3, &ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

