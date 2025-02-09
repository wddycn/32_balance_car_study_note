#ifndef __encoder_H
#define __encoder_H
#include "main.h"

#define MOTOR_SPEED_RERATIO 30    	//电机减速比
#define PULSE_PRE_ROUND 13 			//一圈多少个脉冲
#define RADIUS_OF_TYRE 34 			//轮胎半径，单位毫米
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14				 //轮胎周长
#define Get_ARR_Left __HAL_TIM_GetAutoreload(&htim3)   		 //获取自动装载值,本例中为20000
#define Get_ARR_Right __HAL_TIM_GetAutoreload(&htim4)
#define Get_Encoder_Left __HAL_TIM_GetCounter(&htim3)        //获取编码器定时器中的计数值
#define Get_Encoder_Right __HAL_TIM_GetCounter(&htim4)


typedef struct 
{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
}Motor_Left;


typedef struct 
{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
}Motor_Right;


void Encoder_Init(void);

#endif


