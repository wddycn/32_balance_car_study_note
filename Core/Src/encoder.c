#include "encoder.h"
#include "tim.h"
#include "Motor_Control.h"
#include "MPU6050.h"


Motor_Left motorleft;				//引用encoder.h中定义的结构体
Motor_Right motorright;				//引用encoder.h中定义的结构体





float speed_left;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3 , TIM_CHANNEL_ALL);      //开启编码器定时器3（左轮）
		HAL_TIM_Encoder_Start(&htim4 , TIM_CHANNEL_ALL);			//开启编码器定时器4（右轮）
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);           //开启编码器定时器3更新中断,防溢出处理
	  __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);           //开启编码器定时器4更新中断,防溢出处理

    HAL_TIM_Base_Start_IT(&htim2);                       //开启20ms定时器中断
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);            //开启PWM1（左轮）
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);            //开启PWM4（右轮）
	
    __HAL_TIM_SET_COUNTER(&htim3, 10000);                //编码器3定时器初始值设定为10000
	  __HAL_TIM_SET_COUNTER(&htim4, 10000);                //编码器4定时器初始值设定为10000
	
    motorleft.lastCount = 0;                           //上一次计数值
    motorleft.totalCount = 0;														//总计数值
    motorleft.overflowNum = 0;                          //溢出次数      
    motorleft.speed = 0;    														//电机转速
    motorleft.direct = 0;															//旋转方向
	
	  motorright.lastCount = 0;                          
    motorright.totalCount = 0;
    motorright.overflowNum = 0;                                  
    motorright.speed = 0;
    motorright.direct = 0;
}





/******************滤波函数***********************/
#define SPEED_RECORD_NUM 20 // 经测试，50Hz个采样值进行滤波的效果比较好

float speed_left_Record[SPEED_RECORD_NUM]={0};
float speed_right_Record[SPEED_RECORD_NUM]={0};

/*
 * 进行速度的平均滤波
 * 输入新采样到的速度，存放速度的数组，
 * 返回滤波后的速度
 */
float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    float test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//第一位是新的数据
    sum += new_Spe;
    test_Speed = sum/SPEED_RECORD_NUM;
    return test_Speed;//返回均值
}





/*************************中断处理计算速度*****************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数，用于计算速度
{	
    if(htim->Instance==htim2.Instance)//间隔定时器中断，是时候计算速度了
    {
			
				/******************************计算左轮速度********************************/
        motorleft.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);//判断左轮方向，如果向上计数（正转），返回值为0，否则返回值为1
        motorleft.totalCount = Get_Encoder_Left + motorleft.overflowNum * Get_ARR_Left;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        
        if(motorleft.lastCount - motorleft.totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motorleft.overflowNum++;
            motorleft.totalCount = Get_Encoder_Left + motorleft.overflowNum * Get_ARR_Left;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motorleft.totalCount - motorleft.lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motorleft.overflowNum--;
            motorleft.totalCount = Get_Encoder_Left + motorleft.overflowNum * Get_ARR_Left;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        motorleft.speed = (float)(motorleft.totalCount - motorleft.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 50;//算得每秒多少转,除以4是因为4倍频
				/***************在这里添加滤波函数*******************/
			  motorleft.speed = Speed_Low_Filter(motorleft.speed,speed_left_Record);
        motorleft.lastCount = motorleft.totalCount; //记录这一次的计数值
				
				
		
				
				/******************************计算右轮速度********************************/
				motorright.direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);//如果向上计数（正转），返回值为0，否则返回值为1
        motorright.totalCount = Get_Encoder_Right + motorright.overflowNum * Get_ARR_Right;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        
        if(motorright.lastCount - motorright.totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motorright.overflowNum++;
            motorright.totalCount = Get_Encoder_Right + motorright.overflowNum * Get_ARR_Right;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motorright.totalCount - motorright.lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motorright.overflowNum--;
            motorright.totalCount = Get_Encoder_Right + motorright.overflowNum * Get_ARR_Right;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        motorright.speed = (float)(motorright.totalCount - motorright.lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 50;//算得每秒多少转,除以4是因为4倍频
				/***************在这里添加滤波函数*******************/
			  motorright.speed = Speed_Low_Filter(motorright.speed,speed_right_Record);
        motorright.lastCount = motorright.totalCount; //记录这一次的计数值

 

				
		}
}

