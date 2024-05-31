#ifndef __SYSTEM_TASK_H
#define	__SYSTEM_TASK_H


void Time_Init(void);
void System_Task(void);
void System_State(void);
void PID_Switch(void);

/**
  * @brief  ���ο���
  * @param  �������趨�����ٶ�
  * @retval ���ֵ
  */
float ramp_control(float ref ,float set,float accel);


#endif
