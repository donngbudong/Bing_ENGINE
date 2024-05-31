/***      
�� ���̿��ƴ��� ��
***/
#include "chassis_task.h"
#include "Device.h"
#include "math.h"
#include "cmsis_os.h"


/* Init start */
Chassis_Date_t Chassis ={
	.State=SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
	.PID_Type = Clear_Away,
};



void chassis_task(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	Motor_Init();
  /* Infinite loop */
  for(;;)
  {
		Chassis_Task();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}
/**
 * @brief Chassis�ܿ�
 * @param 
 */
void Chassis_Task(void)
{
	Chassis_GET_Info();
	if(RC_S2== RC_SW_DOWN)
	{
		Chassis_RC_Ctrl();
	}
	else if(RC_S1 == RC_SW_DOWN)
	{
		Chassis_KEY_Ctrl();
	}
	else
	{
		Chassis_Stop();
	}
}



/**
* @brief ������Ϣ��ȡ
* @param 
*/
void Chassis_GET_Info(void)
{
	Chassis.State = System.System_State;
	Chassis.PID_Type = System.System_Pid;
	Chassis_Set_PID_Type();
	Chassis_InitPID();
}



/**
 * @brief PID�����ȡ
 * @note  ����ͬ��
 */
void Chassis_Get_PID_Type(PID_Info_t *str )
{
  str->PID_Type = Chassis.PID_Type;
}

void Chassis_Set_PID_Type(void)
{
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_LF].PID);
  
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_RF].PID);
  
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_LB].PID); 
  
  Chassis_Get_PID_Type(&Chassis.Motor_Data[CHAS_RB].PID);  
}



/**
* @brief ����PID�����ܺ���
* @param 
*/
void Chassis_InitPID(void)
{
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_LF].PID);
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_RF].PID);
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_LB].PID);
	Chassis_PID_Switch(&Chassis.Motor_Data[CHAS_RB].PID);
}


/**
 * @brief PID�л�
 * @param 
 */
void Chassis_PID_Switch(PID_Info_t *str)
{
  static int length = sizeof(PID_Parameter_t);
  switch (str->PID_Type)
  {
    case RC:{
      memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_3508][RC],length);
    }break; 
    case KEY:{
      memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_3508][KEY],length);
    }break;  
    default :{
      memcpy(&(str->Speed_Loop.PID_Param),&PID_Speed_Param[M_3508][Clear_Away],length);
		}break;  
	}
};


/**
 * @brief ң����
 * @param 
 */
int16_t yaw_angle_t;
void Chassis_RC_Ctrl(void)
{

		Chassis.RC_Move.FB_Speed = RC_CH3*8000/660;
		Chassis.RC_Move.LR_Speed = RC_CH2*8000/660;
		Chassis.RC_Move.Z_Speed  = RC_CH0*8000/660;;
//	if(RC_S2 == 3)
//	{
//		Chassis.RC_Move.FB_Speed = RC_CH3*6000/660;
//		Chassis.RC_Move.LR_Speed = RC_CH2*6000/660;
//		Chassis.RC_Move.Z_Speed  = -PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
//																							MECH_Mid_Angle,yaw_angle_t);
//	}
	
  /*PID��������*/
//	PID_Debug(&Chassis.Motor_Data[CHAS_LF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RF].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_LB].PID);
//	PID_Debug(&Chassis.Motor_Data[CHAS_RB].PID);

	Chassis_Handler(&Chassis.RC_Move);
}	
/**
 * @brief ����
 * @param 
 */
void Chassis_KEY_Ctrl(void)
{
	
	if(KEY_W == 1){
		Chassis.KEY_Move.FB_Speed += 500;
	}
	else if(KEY_S == 1){
		Chassis.KEY_Move.FB_Speed -= 500;
	}
	else if(KEY_A == 1){
		Chassis.KEY_Move.LR_Speed -= 500;
	}
	else if(KEY_D == 1){
		Chassis.KEY_Move.LR_Speed += 500;
	}
	else{
		Chassis.KEY_Move.FB_Speed = 0;
		Chassis.KEY_Move.LR_Speed = 0;
	}
	XY_Speed(&Chassis.KEY_Move.FB_Speed,1000,-1000);
	XY_Speed(&Chassis.KEY_Move.LR_Speed,1000,-1000);
	Chassis.KEY_Move.Z_Speed  = PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
																							MECH_Mid_Angle,yaw_angle_t);
	Chassis_Handler(&Chassis.KEY_Move);
}

/**
 * @brief �õ������ĸ�����Ľ����ٶ�Ŀ��ֵ
 * @note  ����ķ������ 
 */
float Speed_Adapt(float FB,float LR,float Z)
{
  float res;
  float Adapt_K = 1;
//  if(abs(FB)+abs(LR)+abs(Z) > Speed_MAX)
//    Adapt_K = (Speed_MAX)/(abs(FB)+abs(LR)+abs(Z));
  res = FB*Adapt_K  + LR*Adapt_K  + Z*Adapt_K ;
  return res;
}
//�����ٶȷ��䵽�����ٶ�
void Chassis_Allot_Speed(Chassis_Move_t *str)
{
	str->Motor_SpeedTarget[CHAS_LF] = Speed_Adapt( str->FB_Speed,
																								 str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RF] = Speed_Adapt(-str->FB_Speed,
																								 str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_LB] = Speed_Adapt(-str->FB_Speed,
																								-str->LR_Speed,	
																								 str->Z_Speed);
	str->Motor_SpeedTarget[CHAS_RB] = Speed_Adapt( str->FB_Speed,
																								-str->LR_Speed,	
																								 str->Z_Speed);
}


/**
 * @brief ��MMKNM���ٶȽ��� б�·ָ��ĸ������PIDĿ��ֵ
 * @param 
 */
void Chassis_MotorSpeed(Chassis_Move_t *str)
{
	Chassis.Motor_Data[CHAS_LF].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_LF];
	Chassis.Motor_Data[CHAS_RF].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_RF];
	Chassis.Motor_Data[CHAS_LB].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_LB];
	Chassis.Motor_Data[CHAS_RB].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_RB];
}


/**
 * @brief PID���
*/
float Chassis_GetOutPut(Chassis_Motor_t *str)
{
	PID_Info_t *PID = &(str->PID);
//  Motor_Data_t *DATA = &(str->Motor_Data);
  float res;
	  /*����ֵ = �ٶȻ�PID���*/
  res = PID_Position(&PID->Speed_Loop,  										 \
										str->Motor_Data.PID_Speed_target,				 \
										str->Motor_Data.CAN_GetData.Motor_Speed);\
  return res;
}

/**
 * @brief Chassis������
 * @param 
 */
void Chassis_CanOutPut(void)
{
	static int16_t pid_out[CHAS_MOTOR_CNT] = {0,0,0,0};
	pid_out[CHAS_LF] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_LF]);
	pid_out[CHAS_RF] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_RF]);
	pid_out[CHAS_LB] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_LB]);
	pid_out[CHAS_RB] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_RB]);
	CAN_cmd_chassis(pid_out[CHAS_LF],pid_out[CHAS_RF],pid_out[CHAS_LB],pid_out[CHAS_RB]);
}

/**
 * @brief Chassis�������
 * @param 
 */
void Chassis_Handler(Chassis_Move_t *str)
{
	Chassis_Allot_Speed(str);
	Chassis_MotorSpeed(str);
	Chassis_CanOutPut();
}


/**
 * @brief ж������
 * @param 
 */
void Chassis_Stop(void)
{
	static int16_t pid_out[CHAS_MOTOR_CNT] = {0, 0, 0, 0};
	
	/* �ٶȻ�������� */
  pid_out[CHAS_LF] = 0;
  pid_out[CHAS_RF] = 0;
  pid_out[CHAS_LB] = 0;
  pid_out[CHAS_RB] = 0;
  
	CAN_cmd_chassis(pid_out[CHAS_LF],pid_out[CHAS_RF],pid_out[CHAS_LB],pid_out[CHAS_RB]);	  
}

float SPIN_GetSpeed_LR(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = cos(Angle*2*3.1415926f/8192.f) * LR_ctrl - sin(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;
}
float SPIN_GetSpeed_FB(float Angle , float FB_ctrl , float LR_ctrl)
{
  float res;
  res = sin(Angle*2*3.1415926f/8192.f) * LR_ctrl + cos(Angle*2*3.14f/8192.f) * FB_ctrl;
  return res;  
}


void XY_Speed(float *speed, float MAX,float MIX)
{
	if(*speed > MAX){
		*speed = MAX;
	}
	if(*speed < MIX){ 
	*speed = MIX;
	}
}
