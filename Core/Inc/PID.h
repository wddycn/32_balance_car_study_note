#ifndef __PID_H
#define __PID_H
#include "main.h"


#define Middle_angle 0  //ֱ�����Ļ�е��ֵ

typedef struct PID {
		float  Vertical_Kp;         //  Proportional Const  Pϵ��
		float  Vertical_Kd;         //  Derivative Const    Dϵ��
	
		float  Velocity_Kp;         //  Proportional Const  Pϵ��
		float  Velocity_Ki;           //  Integral Const      Iϵ��
	
		float  Turn_Kp;         //  Proportional Const  Pϵ��
		float  Turn_Kd;         //  Derivative Const    Dϵ��
		
		float  PrevError ;          //  Error[-2]  
		float  LastError;          //  Error[-1]  
		float  Error;              //  Error[0 ]  
		float  DError;            //pid->Error - pid->LastError	
		float  SumError;           //  Sums of Errors  
		
		float  output;
		
		float  Integralmax;      //����������ֵ
		float  outputmax;        //���������ֵ
	  

	
} PID;



void PID_Init(void);
uint8_t Turn_Off(float angle);
float PID_Balance_Calc(PID *pid, float Angle,short Gyro)  ;
float PID_Speed_Calc(PID *pid, float  speed_left, float speed_right,float Target_speed)  ;
float Turn(float gyro_z,float Target_turn);
void Control(void);
float abs_limit(float value, float ABS_MAX)  ;
	
#endif


