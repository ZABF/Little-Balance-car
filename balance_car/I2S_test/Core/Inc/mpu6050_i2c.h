#ifndef __MPU6050_I2C_H__
#define __MPU6050_I2C_H__
#include "main.h"
/*
硬件I2C模式
需要：
1.I2C
    I2C
    (默认设置)
    标准模式
    时钟频率100kHz
    地址长度7bit
    不用填写设备地址
取消下方注释
*/

extern I2C_HandleTypeDef hi2c1;
#define MPU6050_I2C_Handle hi2c1
//#define MPU6050_Hardware_I2C
#define MPU6050_Software_I2C
//MPU6050 当AD0低电平址为0x68 << 1 = 0xD0，当AD0高电平地址为0x69 << 1 = 0xD2，此模块下拉默认为低电平，地址为0xD0
#define MPU_ADDR 0x68
/*
软件I2C模式
需要：
1.GPIO 2个
    均为开漏输出（上不上拉取决于外部电路）
    最高等级
取消下方注释,按照自己的管脚更改即可


#define MPU6050_Software_I2C
*/
#ifdef MPU6050_Software_I2C
#define I2C_Group_SCL GPIOB // I2C的时钟GPIO组号
#define I2C_SCL GPIO_PIN_8  // I2C时钟的GPIO端口号

#define I2C_Group_SDA GPIOB // I2C的数据GPIO组号
#define I2C_SDA GPIO_PIN_9  // I2C数据的GPIO端口号

#define I2C_Write_SCL(x) HAL_GPIO_WritePin(I2C_Group_SCL, I2C_SCL, x)
#define I2C_Write_SDA(x) HAL_GPIO_WritePin(I2C_Group_SDA, I2C_SDA, x)

#define I2C_Read_SCL() HAL_GPIO_ReadPin(I2C_Group_SCL, I2C_SCL)
#define I2C_Read_SDA() HAL_GPIO_ReadPin(I2C_Group_SDA, I2C_SDA)
#endif


uint8_t MPU_Write_Len(uint8_t add,uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t add,uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data);
uint8_t MPU_Read_Byte(uint8_t reg);


#endif
