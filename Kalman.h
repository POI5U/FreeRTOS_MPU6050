#ifndef __KALMAN_H
#define __KALMAN_H


// 一个角度创建一个结构体
struct Kalman_t{
	float Q_angle;		//acc   
	float Q_gyro;		//gyro  
	float R_angle;			
	float Q_bias;				
	float K_0, K_1;
	float PP[2][2];
	float Angle_kalman;

};

typedef struct Kalman_t Kalman_t;

Kalman_t* Kalman_Init(float Q_angle, float Q_gyro);
float Kalman_Cal(Kalman_t* p, float acc, float gyro, float dt);


#endif

