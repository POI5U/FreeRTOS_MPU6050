#include "Kalman.h"
#include "stdlib.h"


/**
  * @brief         	��ȡ����ʼ��һ���˲��ṹ��
  * @param     		Q_angle: �Ƕ��������Ŷ�		����ȡ 0.001
  * @param      	Q_gyro : ���ٶ��������Ŷ�	����ȡ 0.003
  * @retval         �������˲��ṹ��
  */
Kalman_t* Kalman_Init(float Q_angle, float Q_gyro){
	Kalman_t* port = (Kalman_t*)malloc(sizeof(Kalman_t));

	port->Q_angle = 0.001;	//acc   �Ƕ��������Ŷȣ��Ƕ�������Э����
	port->Q_gyro = 0.003;	//gyro  ���ٶ��������Ŷȣ����ٶ�������Э����
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
  * @brief         	����һ���˲�����
  * @param     		p: �˲��ṹ��
  * @param      	acc : ��ȡ��ʵ�ʽǶ� ( ��λ: �� )
  * @param      	gyro : ��ȡ�Ľ��ٶ� ( ��λ : ��ÿ�� )
  * @param      	dt : ���б����������� ( ҲΪ�����ϴν��뺯����ʱ�� )
  * @retval         �������˲��ṹ��
  */
float Kalman_Cal(Kalman_t* p, float acc, float gyro, float dt){

	// Ԥ��Ƕ�
	p->Angle_kalman += (gyro - p->Q_bias) * dt;		
	
	// Ԥ��Э�������
	p->PP[0][0] = p->PP[0][0] + p->Q_angle - (p->PP[0][1] + p->PP[1][0])*dt;	
	p->PP[0][1] = p->PP[0][1] - p->PP[1][1]*dt;
	p->PP[1][0] = p->PP[1][0] - p->PP[1][1]*dt;
	p->PP[1][1] = p->PP[1][1] + p->Q_gyro;
	
	// ��������
	p->K_0 = p->PP[0][0] / (p->PP[0][0] + p->R_angle);
	p->K_1 = p->PP[1][0] / (p->PP[0][0] + p->R_angle);
	
	// ���㵱ǰ����ֵ
	p->Angle_kalman = p->Angle_kalman + p->K_0 * (acc - p->Angle_kalman);
	p->Q_bias = p->Q_bias + p->K_1 * (acc - p->Angle_kalman);
	
	// ����Э�������
	p->PP[0][0] = p->PP[0][0] - p->K_0 * p->PP[0][0];
	p->PP[0][1] = p->PP[0][1] - p->K_0 * p->PP[0][1];
	p->PP[1][0] = p->PP[1][0] - p->K_1 * p->PP[0][0];
	p->PP[1][1] = p->PP[1][1] - p->K_1 * p->PP[0][1];
	
	return p->Angle_kalman;
}



