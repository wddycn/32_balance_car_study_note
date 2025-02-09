#include "Motor_Control.h"
#include "tim.h"



void Motor_Control_Init(void)//初始化两个轮子的PWM通道
{
		HAL_TIM_PWM_Start (&htim1 ,TIM_CHANNEL_1 );//左轮
	  HAL_TIM_PWM_Start (&htim1 ,TIM_CHANNEL_4 );//右轮
}

void SetPWMleft(int pwm)
{
	if(pwm>=0)//pwm>=0 (AIN1,AIN2)=(0, 1) 正转 顺时针
	{
		HAL_GPIO_WritePin (GPIOB,PB14_Pin ,GPIO_PIN_RESET); //AIN1=0
		HAL_GPIO_WritePin (GPIOB,PB15_Pin ,GPIO_PIN_SET); //AIN2=1
		__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_1 ,pwm);//pwm=0~1000
	}
	else if(pwm<0)//pwm<0 (AIN1, AIN2)=(1, 0) 反转 逆时针
	{
		HAL_GPIO_WritePin (GPIOB,PB14_Pin ,GPIO_PIN_SET); //AIN1=1
		HAL_GPIO_WritePin (GPIOB,PB15_Pin ,GPIO_PIN_RESET); //AIN2=0
		__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_1 ,-pwm);//pwm=-1000~0
	}
}


void SetPWMright(int pwm)
{
	if(pwm>=0)//pwm>=0 (BIN1,BIN2)=(0, 1) 正转 顺时针
	{
		HAL_GPIO_WritePin (GPIOB,PB13_Pin ,GPIO_PIN_RESET); //BIN1=0
		HAL_GPIO_WritePin (GPIOB,PB12_Pin ,GPIO_PIN_SET); //BIN2=1
		__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4,pwm);//pwm=0~1000
	}
	else if(pwm<0)//pwm<0 (BIN1, BIN2)=(1, 0) 反转 逆时针
	{
		HAL_GPIO_WritePin (GPIOB,PB13_Pin ,GPIO_PIN_SET); //BIN1=1
		HAL_GPIO_WritePin (GPIOB,PB12_Pin ,GPIO_PIN_RESET); //BIN2=0
		__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4,-pwm);//pwm=-1000~0
	}
}
