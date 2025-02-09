#include "PID.h"
#include "Motor_Control.h"
#include "encoder.h"
#include "MPU6050.h"

PID pid;



//传感器的值
extern Motor_Left motorleft;				//引用encoder.h中定义的结构体
extern Motor_Right motorright;				//引用encoder.h中定义的结构体
float pitch,roll,yaw;
short AX, AY, AZ, GX, GY, GZ;

//闭环控制中间变量
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
*保护小车
********************************************************************************/
uint8_t Turn_Off(float angle)
{
	uint8_t temp;
	if(angle<-40||angle>40)//倾角大于40度关闭电机
	{	                                            
		temp=0;                                         
		SetPWMleft(0);  //关闭左电机
		SetPWMright(0); //关闭右电机
	}
	else
		temp=1;
	return temp;			
}


/********************************************************************************
*直立环
********************************************************************************/

float PID_Balance_Calc(PID *pid, float Angle,short Gyro_x)  
{  
   float Angle_bias,Gyro_bias;
	 Angle_bias=Angle-Middle_angle;                    				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro_x; 
	 pid->output= -pid->Vertical_Kp*Angle_bias-Gyro_bias*pid->Vertical_Kd; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	
	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
	
	return pid->output;
}

/**************************************************************************************************************
*速度环
**************************************************************************************************************/

//为了防止积分项过度累积，引入积分项的限幅是一种常见的做法。
//限制积分项的幅值可以防止积分项过度增加，从而限制了系统的累积误差。这样可以避免系统过度响应或者不稳定。
float abs_limit(float value, float ABS_MAX)   //积分限幅，设置最大值。
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
	pid->Error = Target_speed-(speed_left+speed_right);    //获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）             
    Encoder_bias *=0.8;    //一阶低通滤波器     
    Encoder_bias += pid->Error*0.2;  //一阶低通滤波器  
    pid->SumError +=Encoder_bias;
    
	pid->output  = -pid->Velocity_Kp* Encoder_bias 
	               -pid->Velocity_Ki* abs_limit( pid->SumError, 10000);

	if(Turn_Off(roll)==1)   pid->SumError=0;      //电机关闭后清除积分	
	
	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;						
	return pid->output ;   //输出为pwm值
}


/**************************************************************************************************************
*转向环
**************************************************************************************************************/

float Turn(float gyro_z,float Target_turn)
{
	float temp;
	temp = pid.Turn_Kp*Target_turn +pid.Turn_Kd*gyro_z;
	return temp ;
	
}

/**************************************************************************************************************
*核心控制函数
**************************************************************************************************************/

int PWM_Limit(int IN,int max,int min)//限幅函数，限制pwm输出
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}

void Control(void)	//要求：每隔10ms调用一次
{
	//读取编码器速度和陀螺仪的值
	MPU6050_DMP_Get_Data( &pitch, &roll, &yaw, &GX, &GY, &GZ, &AX, &AY, &AZ);
	//将数据传入PID控制器，计算出输出结果
	Velocity_out = PID_Speed_Calc(&pid,motorleft.speed,motorright.speed,Target_speed);
	Vertical_out = PID_Balance_Calc(&pid,roll,GX);
	pwm_left =Velocity_out+Vertical_out;
	pwm_right =Velocity_out+Vertical_out;
	pwm_left = PWM_Limit(pwm_left,1000,-1000);
	pwm_right = PWM_Limit(pwm_right,1000,-1000);
	SetPWMleft(pwm_left);
	SetPWMright(pwm_right);
	
}


