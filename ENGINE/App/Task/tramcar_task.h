#ifndef __TRAMCAR_TASK_H
#define __TRAMCAR_TASK_H
/************************************** Includes **************************************/

#include "Device.h"
#include "system.h"






typedef struct 
{
  float FB_Speed,LR_Speed,Z_Speed ;
//  float FB_Speed_k,LR_Speed_k,Z_Speed_k; //遥控->速度 变化的幅度调节因子
  float Motor_SpeedTarget[4];
}Tramcar_Move_t;






typedef struct 
{
 	Motor_Data_t	Motor_Data;
  PID_Info_t		PID;
}Tramcar_Motor_t;

/* 底盘结构体 */
typedef struct
{
  System_State_t State;
	System_Ctrl_Mode_t Ctrl_Mode;
	Motor_Info_t  Motor_Info;
	PID_Type_t PID_Type;
	Tramcar_Motor_t Motor_Data[4];
	pid_type_def PID[4];
	
	Tramcar_Move_t RC_Move;
	Tramcar_Move_t KEY_Move;

}Tramcar_Date_t;

void Tramcar_RC_Ctrl(void);


extern Tramcar_Date_t Tramcar;
#endif
