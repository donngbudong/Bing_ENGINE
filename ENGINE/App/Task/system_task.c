#include "system_task.h"
#include "System.h"
#include "Device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"



Sys_Info_t System = {
	.Rc_State  = RC_LOST,
	.Imu_State = IMU_LOST,
	.System_State = SYSTEM_LOST,
	.System_Pid = Clear_Away,
};



uint64_t Remote_time = 0;	//ң����
uint64_t Imu_time = 0;		//����ϵͳ
uint64_t Vision_time = 0;	//�Ӿ�


void system_task(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {
		System_Task();
    osDelay(3);
  }
  /* USER CODE END StartDefaultTask */
}


//�豸���ӳ�ʼ��
void Time_Init(void)
{
  Remote_time = micros();
  Imu_time = micros();
	Vision_time = micros();
}

void System_Task(void)
{
	System_State();
	RC_State_Report();
//	ws2812_blue(200);
//	Waterfall_light();
	PID_Switch();
}

void System_State(void)
{
    if(System.Rc_State ==RC_ERR || System.Imu_State ==IMU_ERR)
  {
    System.System_State = SYSTEM_ERR;
  } 
    if(System.Rc_State ==RC_NORMAL && System.Imu_State ==IMU_NORMAL)
  {
    System.System_State = SYSTEM_NORMAL;
  }
  if(System.Rc_State ==RC_LOST || System.Imu_State ==IMU_LOST)
  {
    System.System_State = SYSTEM_LOST;
  }
}

void PID_Switch(void)
{
	if(RC_S1==1)
	{
		System.System_Pid = RC;
	}
	if(RC_S1==3)
	{
		System.System_Pid = Clear_Away;
	}
	if(RC_S1==2)
	{
		System.System_Pid = KEY;
	}
}



/**
  * @brief  ���ݷ�Χ����
  * @param  ��������,��Сֵ,���ֵ
  * @retval �������
  */
float limit(float data, float min, float max)
{
	if (data >= max)
		return max;
	if (data <= min)
		return min;
	return data;
}

/**
  * @brief  ���ο���
  * @param  �������趨�����ٶ�
  * @retval ���ֵ
  */
float ramp_control(float ref ,float set,float accel)
{
	float ramp = limit(accel,0,1)*(set - ref);
	return ref + ramp;
}
