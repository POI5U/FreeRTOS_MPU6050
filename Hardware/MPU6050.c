#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "stdlib.h"

#define MPU6050_ADDRESS		0xD0


static void MPU6050_delayms(uint32_t xms) {
	
    volatile uint32_t ticks = xms * 72000;  // 72 MHz / 1000 (ms)
	
    for (uint32_t i = 0; i < xms * 72000; i++) {
        __NOP();
    }
}

static void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}

static void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, Data);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2, ENABLE);
}

static uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C2);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	
	return Data;
}

static void MUP6050_getG(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ){
	*GyroX = (MPU6050_ReadReg(MPU6050_GYRO_XOUT_H) << 8) | MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroY = (MPU6050_ReadReg(MPU6050_GYRO_YOUT_H) << 8) | MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroZ = (MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H) << 8) | MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
}
static int16_t errorX = 0, errorY = 0, errorZ = 0;


void MPU6050_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 50000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C2, &I2C_InitStructure);
	
	I2C_Cmd(I2C2, ENABLE);
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);		// max: 2000du/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);		// max: 2g
	
	// error -> 0
	int16_t GX, GY, GZ;
	MPU6050_delayms(30);
	for(int i = 0; i < 10; i++){
		MUP6050_getG(&GX, &GY, &GZ);
		errorX += GX;
		errorY += GY;
		errorZ += GZ;
	}
	errorX /= 10;
	errorY /= 10;
	errorZ /= 10;
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = ((DataH << 8) | DataL) - errorX;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = ((DataH << 8) | DataL) - errorY;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = ((DataH << 8) | DataL) - errorZ;
}



//		main.c FreeRTOS  Ê¾Àý´úÂë (ÈÎÎñÐÎÊ½)
/*


float pitch = 0, roll = 0, yaw = 0;
float dt = 0;

void MPU6050(void *pvParameters){
	
	int16_t AX, AY, AZ, GX, GY, GZ;
	float pitch_angle, roll_angle, yaw_angle;
	float pitch_gyro, roll_gyro;
	uint32_t T_last = 0, T_now = 0;

	Kalman_t* pitch_kalman = Kalman_Init();
	Kalman_t* roll_kalman = Kalman_Init();

	while(1){
		
		//Ô­Ê¼Êý¾Ý¶ÁÈ¡ÓëÔ¤´¦Àí£¨²»±£»¤£©
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		pitch_angle = atan2(AX, AZ) * 57.29578;
		roll_angle = atan2(AY, AZ) * 57.29578;
		pitch_gyro = (float)GY * 2000 / 32767;
		roll_gyro  = (float)GX * 2000 / 32767;
		
		taskENTER_CRITICAL();		//±£»¤¿ªÆô  ±£»¤Ê±¼äÇ¿Ïà¹ØÈÎÎñ
		
		// »ñÈ¡²¢¸üÐÂ dt
		T_now = xTaskGetTickCount();
		dt = (float)(T_now - T_last) * 0.001;
		T_last = T_now;


		//½øÐÐ½Ç¶ÈÂË²¨¼ÆËã
		pitch = Kalman_Cal(pitch_kalman, pitch_angle, pitch_gyro, dt);
		roll = Kalman_Cal(roll_kalman, roll_angle, roll_gyro, dt);

		yaw_angle += (float)GZ * 2000 / 32767 * dt;
		yaw = yaw_angle;
		

		taskEXIT_CRITICAL();		//±£»¤½áÊø	 ¼ÆËã½áÊø
		
		
		Delay_ms(5);
	}
}



*/

