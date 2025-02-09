#include "encoder.h"
#include "tim.h"
#include "Motor_Control.h"
#include "MPU6050.h"


Motor_Left motorleft;				//����encoder.h�ж���Ľṹ��
Motor_Right motorright;				//����encoder.h�ж���Ľṹ��





float speed_left;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3 , TIM_CHANNEL_ALL);      //������������ʱ��3�����֣�
		HAL_TIM_Encoder_Start(&htim4 , TIM_CHANNEL_ALL);			//������������ʱ��4�����֣�
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);           //������������ʱ��3�����ж�,���������
	  __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);           //������������ʱ��4�����ж�,���������

    HAL_TIM_Base_Start_IT(&htim2);                       //����20ms��ʱ���ж�
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);            //����PWM1�����֣�
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);            //����PWM4�����֣�
	
    __HAL_TIM_SET_COUNTER(&htim3, 10000);                //������3��ʱ����ʼֵ�趨Ϊ10000
	  __HAL_TIM_SET_COUNTER(&htim4, 10000);                //������4��ʱ����ʼֵ�趨Ϊ10000
	
    motorleft.lastCount = 0;                           //��һ�μ���ֵ
    motorleft.totalCount = 0;														//�ܼ���ֵ
    motorleft.overflowNum = 0;                          //�������      
    motorleft.speed = 0;    														//���ת��
    motorleft.direct = 0;															//��ת����
	
	  motorright.lastCount = 0;                          
    motorright.totalCount = 0;
    motorright.overflowNum = 0;                                  
    motorright.speed = 0;
    motorright.direct = 0;
}





/******************�˲�����***********************/
#define SPEED_RECORD_NUM 20 // �����ԣ�50Hz������ֵ�����˲���Ч���ȽϺ�

float speed_left_Record[SPEED_RECORD_NUM]={0};
float speed_right_Record[SPEED_RECORD_NUM]={0};

/*
 * �����ٶȵ�ƽ���˲�
 * �����²��������ٶȣ�����ٶȵ����飬
 * �����˲�����ٶ�
 */
float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    float test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//���������ݺ���һλ
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//��һλ���µ�����
    sum += new_Spe;
    test_Speed = sum/SPEED_RECORD_NUM;
    return test_Speed;//���ؾ�ֵ
}





/*************************�жϴ�������ٶ�*****************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//��ʱ���ص����������ڼ����ٶ�
{	
    if(htim->Instance==htim2.Instance)//�����ʱ���жϣ���ʱ������ٶ���
    {
			
				/******************************���������ٶ�********************************/
        motorleft.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);//�ж����ַ���������ϼ�������ת��������ֵΪ0�����򷵻�ֵΪ1
        motorleft.totalCount = Get_Encoder_Left + motorleft.overflowNum * Get_ARR_Left;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        
        if(motorleft.lastCount - motorleft.totalCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motorleft.overflowNum++;
            motorleft.totalCount = Get_Encoder_Left + motorleft.overflowNum * Get_ARR_Left;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        else if(motorleft.totalCount - motorleft.lastCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motorleft.overflowNum--;
            motorleft.totalCount = Get_Encoder_Left + motorleft.overflowNum * Get_ARR_Left;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        motorleft.speed = (float)(motorleft.totalCount - motorleft.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 50;//���ÿ�����ת,����4����Ϊ4��Ƶ
				/***************����������˲�����*******************/
			  motorleft.speed = Speed_Low_Filter(motorleft.speed,speed_left_Record);
        motorleft.lastCount = motorleft.totalCount; //��¼��һ�εļ���ֵ
				
				
		
				
				/******************************���������ٶ�********************************/
				motorright.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);//������ϼ�������ת��������ֵΪ0�����򷵻�ֵΪ1
        motorright.totalCount = Get_Encoder_Right + motorright.overflowNum * Get_ARR_Right;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        
        if(motorright.lastCount - motorright.totalCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motorright.overflowNum++;
            motorright.totalCount = Get_Encoder_Right + motorright.overflowNum * Get_ARR_Right;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        else if(motorright.totalCount - motorright.lastCount > 19000) // �ڼ���ֵ���ʱ���з��������
        {
            motorright.overflowNum--;
            motorright.totalCount = Get_Encoder_Right + motorright.overflowNum * Get_ARR_Right;//һ�������ڵ��ܼ���ֵ����Ŀǰ����ֵ��������ļ���ֵ
        }
        motorright.speed = (float)(motorright.totalCount - motorright.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 50;//���ÿ�����ת,����4����Ϊ4��Ƶ
				/***************����������˲�����*******************/
			  motorright.speed = Speed_Low_Filter(motorright.speed,speed_right_Record);
        motorright.lastCount = motorright.totalCount; //��¼��һ�εļ���ֵ

 

				
		}
}

