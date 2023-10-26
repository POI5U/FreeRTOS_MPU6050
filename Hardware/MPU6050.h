#ifndef __MPU6050_H
#define __MPU6050_H


struct Kalman_t{
	float Q_angle;		//acc   角度数据置信度，角度噪声的协方差
	float Q_gyro;		//gyro  角速度数据置信度，角速度噪声的协方差
	float R_angle;			//加速度计测量噪声的协方差
	float Q_bias;				//陀螺仪gyro的偏差
	float K_0, K_1;
	float PP[2][2];
	float Angle_kalman;

};
// 包含单个角度卡尔曼滤波基本参数
typedef struct Kalman_t Kalman_t;




void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

Kalman_t* Kalman_Init(void);
float Kalman_Cal(Kalman_t* p, float acc, float gyro, float dt);


#endif
