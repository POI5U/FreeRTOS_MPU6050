#include "Kalman.h"
#include "stdlib.h"


/**
  * @brief         	获取并初始化一个滤波结构体
  * @param     		Q_angle: 角度数据置信度		建议取 0.001
  * @param      	Q_gyro : 角速度数据置信度	建议取 0.003
  * @retval         卡尔曼滤波结构体
  */
Kalman_t* Kalman_Init(float Q_angle, float Q_gyro){
	Kalman_t* port = (Kalman_t*)malloc(sizeof(Kalman_t));

	port->Q_angle = 0.001;	//acc   角度数据置信度，角度噪声的协方差
	port->Q_gyro = 0.003;	//gyro  角速度数据置信度，角速度噪声的协方差
	port->R_angle = 0.5;
	
	port->Q_bias = 0;
	
	port->K_0 = 0;
	port->K_1 = 0;
	
	port->PP[0][0] = 1;
	port->PP[1][0] = 0;
	port->PP[0][1] = 0;
	port->PP[1][1] = 1;

	port->Angle_kalman = 0;

	return port;
}




/**
  * @brief         	进行一次滤波操作
  * @param     		p: 滤波结构体
  * @param      	acc : 获取的实际角度 ( 单位: 度 )
  * @param      	gyro : 获取的角速度 ( 单位 : 度每秒 )
  * @param      	dt : 运行本函数的周期 ( 也为距离上次进入函数的时间 )
  * @retval         卡尔曼滤波结构体
  */
float Kalman_Cal(Kalman_t* p, float acc, float gyro, float dt){

	// 预测角度
	p->Angle_kalman += (gyro - p->Q_bias) * dt;		
	
	// 预测协方差矩阵
	p->PP[0][0] = p->PP[0][0] + p->Q_angle - (p->PP[0][1] + p->PP[1][0])*dt;	
	p->PP[0][1] = p->PP[0][1] - p->PP[1][1]*dt;
	p->PP[1][0] = p->PP[1][0] - p->PP[1][1]*dt;
	p->PP[1][1] = p->PP[1][1] + p->Q_gyro;
	
	// 计算增益
	p->K_0 = p->PP[0][0] / (p->PP[0][0] + p->R_angle);
	p->K_1 = p->PP[1][0] / (p->PP[0][0] + p->R_angle);
	
	// 计算当前估计值
	p->Angle_kalman = p->Angle_kalman + p->K_0 * (acc - p->Angle_kalman);
	p->Q_bias = p->Q_bias + p->K_1 * (acc - p->Angle_kalman);
	
	// 更新协方差矩阵
	p->PP[0][0] = p->PP[0][0] - p->K_0 * p->PP[0][0];
	p->PP[0][1] = p->PP[0][1] - p->K_0 * p->PP[0][1];
	p->PP[1][0] = p->PP[1][0] - p->K_1 * p->PP[0][0];
	p->PP[1][1] = p->PP[1][1] - p->K_1 * p->PP[0][1];
	
	return p->Angle_kalman;
}



