#include "PID.h"
#include "Motor_Control.h"
#include "encoder.h"
#include "MPU6050.h"

PID pid;



//��������ֵ
extern Motor_Left motorleft;				//����encoder.h�ж���Ľṹ��
extern Motor_Right motorright;				//����encoder.h�ж���Ľṹ��
float pitch,roll,yaw;
short AX, AY, AZ, GX, GY, GZ;

//�ջ������м����
float Vertical_out,Velocity_out,Turn_out,Target_speed=0;

void PID_Init(void)
{
	pid.Vertical_Kp = 60;
	pid.Vertical_Kd = -0.18;
	
	pid.Velocity_Kp = 170;
	pid.Velocity_Ki = 0.85;	
	
	pid.Turn_Kp = 0;
	pid.Turn_Kd = 0;
	pid.outputmax =1000;
	Target_speed = 0;
	pid.output = 0;
	pid.SumError = 0;
}

float stop;
int pwm_left,pwm_right;





/********************************************************************************
*����С��
********************************************************************************/
uint8_t Turn_Off(float angle)
{
	uint8_t temp;
	if(angle<-40||angle>40)//��Ǵ���40�ȹرյ��
	{	                                            
		temp=0;                                         
		SetPWMleft(0);  //�ر�����
		SetPWMright(0); //�ر��ҵ��
	}
	else
		temp=1;
	return temp;			
}


/********************************************************************************
*ֱ����
********************************************************************************/

float PID_Balance_Calc(PID *pid, float Angle,short Gyro_x)  
{  
   float Angle_bias,Gyro_bias;
	 Angle_bias=Angle-Middle_angle;                    				//���ƽ��ĽǶ���ֵ �ͻ�е���
	 Gyro_bias=0-Gyro_x; 
	 pid->output= -pid->Vertical_Kp*Angle_bias-Gyro_bias*pid->Vertical_Kd; //����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	
	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
	
	return pid->output;
}

/**************************************************************************************************************
*�ٶȻ�
**************************************************************************************************************/

//Ϊ�˷�ֹ����������ۻ��������������޷���һ�ֳ�����������
//���ƻ�����ķ�ֵ���Է�ֹ������������ӣ��Ӷ�������ϵͳ���ۻ����������Ա���ϵͳ������Ӧ���߲��ȶ���
float abs_limit(float value, float ABS_MAX)   //�����޷����������ֵ��
{
	if(value > ABS_MAX)
		value = ABS_MAX;

	if(value< -ABS_MAX)
		value = -ABS_MAX;
	return value;
}

float PID_Speed_Calc(PID *pid, float  speed_left, float speed_right,float Target_speed)  
{  
    static float Encoder_bias;
	pid->Error = Target_speed-(speed_left+speed_right);    //��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩             
    Encoder_bias *=0.8;    //һ�׵�ͨ�˲���     
    Encoder_bias += pid->Error*0.2;  //һ�׵�ͨ�˲���  
    pid->SumError +=Encoder_bias;
    
	pid->output  = -pid->Velocity_Kp* Encoder_bias 
	               -pid->Velocity_Ki* abs_limit( pid->SumError, 10000);

	if(Turn_Off(roll)==1)   pid->SumError=0;      //����رպ��������	
	
	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;						
	return pid->output ;   //���Ϊpwmֵ
}


/**************************************************************************************************************
*ת��
**************************************************************************************************************/

float Turn(float gyro_z,float Target_turn)
{
	float temp;
	temp = pid.Turn_Kp*Target_turn +pid.Turn_Kd*gyro_z;
	return temp ;
	
}

/**************************************************************************************************************
*���Ŀ��ƺ���
**************************************************************************************************************/

int PWM_Limit(int IN,int max,int min)//�޷�����������pwm���
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}

void Control(void)	//Ҫ��ÿ��10ms����һ��
{
	//��ȡ�������ٶȺ������ǵ�ֵ
	MPU6050_DMP_Get_Data( &pitch, &roll, &yaw, &GX, &GY, &GZ, &AX, &AY, &AZ);
	//�����ݴ���PID�������������������
	Velocity_out = PID_Speed_Calc(&pid,motorleft.speed,motorright.speed,Target_speed);
	Vertical_out = PID_Balance_Calc(&pid,roll,GX);
	pwm_left =Velocity_out+Vertical_out;
	pwm_right =Velocity_out+Vertical_out;
	pwm_left = PWM_Limit(pwm_left,1000,-1000);
	pwm_right = PWM_Limit(pwm_right,1000,-1000);
	SetPWMleft(pwm_left);
	SetPWMright(pwm_right);
	
}


