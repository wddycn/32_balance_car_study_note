#ifndef __MPU6050_H
#define __MPU6050_H

int MPU6050_DMP_Init(void);
int  MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw,short *gx,short *gy,short *gz,short *ax,short *ay,short *az);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
