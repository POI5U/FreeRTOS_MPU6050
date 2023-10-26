#include "math.h"
#include "stm32f10x.h"                  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"



float pitch = 0, roll = 0, yaw = 0;

float dt = 0;

// 一套下来约耗时 9 ms
void MPU6050(void *pvParameters){
	
	int16_t AX, AY, AZ, GX, GY, GZ;
	float pitch_angle, roll_angle, yaw_angle;
	float pitch_gyro, roll_gyro;
	uint32_t T_last = 0, T_now = 0;

	Kalman_t* pitch_kalman = Kalman_Init();
	Kalman_t* roll_kalman = Kalman_Init();

	while(1){
		
		//原始数据读取与预处理（不保护）
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		pitch_angle = atan2(AX, AZ) * 57.29578;
		roll_angle = atan2(AY, AZ) * 57.29578;
		pitch_gyro = (float)GY * 2000 / 32767;
		roll_gyro  = (float)GX * 2000 / 32767;
		
		taskENTER_CRITICAL();		//保护开启  保护时间强相关任务
		
		// 获取并更新 dt
		T_now = xTaskGetTickCount();
		dt = (float)(T_now - T_last) * 0.001;
		T_last = T_now;


		//进行角度滤波计算
		pitch = Kalman_Cal(pitch_kalman, pitch_angle, pitch_gyro, dt);
		roll = Kalman_Cal(roll_kalman, roll_angle, roll_gyro, dt);

		yaw_angle += (float)GZ * 2000 / 32767 * dt;
		yaw = yaw_angle;
		

		taskEXIT_CRITICAL();		//保护结束	 计算结束
		
		
		Delay_ms(5);
	}
}

void OLED(void *pvParameters){
	OLED_ShowString(1, 1, "dt:");
	OLED_ShowString(2, 1, "pitch:");
	OLED_ShowString(3, 1, "roll:");
	OLED_ShowString(4, 1, "yaw:");

	while(1){
		OLED_ShowFloat(1, 8, dt, 3);
		OLED_ShowFloat(2, 8, pitch, 3);
		OLED_ShowFloat(3, 8, roll, 3);
		OLED_ShowFloat(4, 8, yaw, 3);
		
		
		Delay_ms(0);
		//portYIELD();
	}
}



int main(void){
	OLED_Init();
	MPU6050_Init();
	CLK_Delay_s(1);
	
	xTaskCreate(MPU6050, "MPU6050", 2048, NULL, 2, NULL);
	xTaskCreate(OLED, "OLED", 256, NULL, 1, NULL);
	
	vTaskStartScheduler();
}
