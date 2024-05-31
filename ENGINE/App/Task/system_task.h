#ifndef __SYSTEM_TASK_H
#define	__SYSTEM_TASK_H


void Time_Init(void);
void System_Task(void);
void System_State(void);
void PID_Switch(void);

/**
  * @brief  梯形控制
  * @param  反馈；设定；加速度
  * @retval 输出值
  */
float ramp_control(float ref ,float set,float accel);


#endif
