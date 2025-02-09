#ifndef __PID_H
#define __PID_H
#include "main.h"


#define Middle_angle 0  //直立环的机械中值

typedef struct PID {
		float  Vertical_Kp;         //  Proportional Const  P系数
		float  Vertical_Kd;         //  Derivative Const    D系数
	
		float  Velocity_Kp;         //  Proportional Const  P系数
		float  Velocity_Ki;           //  Integral Const      I系数
	
		float  Turn_Kp;         //  Proportional Const  P系数
		float  Turn_Kd;         //  Derivative Const    D系数
		
		float  PrevError ;          //  Error[-2]  
		float  LastError;          //  Error[-1]  
		float  Error;              //  Error[0 ]  
		float  DError;            //pid->Error - pid->LastError	
		float  SumError;           //  Sums of Errors  
		
		float  output;
		
		float  Integralmax;      //积分项的最大值
		float  outputmax;        //输出项的最大值
	  

	
} PID;



void PID_Init(void);
uint8_t Turn_Off(float angle);
float PID_Balance_Calc(PID *pid, float Angle,short Gyro)  ;
float PID_Speed_Calc(PID *pid, float  speed_left, float speed_right,float Target_speed)  ;
float Turn(float gyro_z,float Target_turn);
void Control(void);
float abs_limit(float value, float ABS_MAX)  ;
	
#endif


