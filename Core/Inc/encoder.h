#ifndef __encoder_H
#define __encoder_H
#include "main.h"

#define MOTOR_SPEED_RERATIO 30    	//������ٱ�
#define PULSE_PRE_ROUND 13 			//һȦ���ٸ�����
#define RADIUS_OF_TYRE 34 			//��̥�뾶����λ����
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14				 //��̥�ܳ�
#define Get_ARR_Left __HAL_TIM_GetAutoreload(&htim3)   		 //��ȡ�Զ�װ��ֵ,������Ϊ20000
#define Get_ARR_Right __HAL_TIM_GetAutoreload(&htim4)
#define Get_Encoder_Left __HAL_TIM_GetCounter(&htim3)        //��ȡ��������ʱ���еļ���ֵ
#define Get_Encoder_Right __HAL_TIM_GetCounter(&htim4)


typedef struct 
{
    int32_t lastCount;   //��һ�μ���ֵ
    int32_t totalCount;  //�ܼ���ֵ
    int16_t overflowNum; //�������
    float speed;         //���ת��
    uint8_t direct;      //��ת����
}Motor_Left;


typedef struct 
{
    int32_t lastCount;   //��һ�μ���ֵ
    int32_t totalCount;  //�ܼ���ֵ
    int16_t overflowNum; //�������
    float speed;         //���ת��
    uint8_t direct;      //��ת����
}Motor_Right;


void Encoder_Init(void);

#endif


