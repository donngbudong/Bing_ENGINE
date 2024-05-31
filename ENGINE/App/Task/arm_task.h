#ifndef __ARM_TASK_H
#define __ARM_TASK_H
/************************************** Includes **************************************/

#include "Device.h"
#include "system.h"

/********************************* Exported functions *********************************/

#define dm_LimitMax(input, max,mix)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < mix) \
        {                      \
            input = mix;      \
        }                      \
    }



typedef struct 
{
  float Angle_Inc;
  float Angle_k;//角度增加的幅度调节因子
  float Slow_Inc;
}Arm_Motor_t;


typedef struct
{
  Motor_Info_t  Motor_Info;
  Motor_Data_t  Motor_Data;
	motor_t				DM_Motor_Data;
  PID_Info_t    PID;
	
	Arm_Motor_t RC_Move;
	Arm_Motor_t	KEY_Move;
	
}Arm_Info_t;



/* 底盘结构体 */
typedef struct
{
  System_State_t State;
	System_Ctrl_Mode_t Ctrl_Mode;
	PID_Type_t PID_Type;
	Arm_Info_t  PITCH[2];
	Arm_Info_t	ROLL[2];
}Arm_Date_t;





void Arm_Task(void);
void Arm_GET_Info(void);
void Arm_RC_Ctrl(void);

void silver_ore(void);

void Handle_Angle8191_PID_Over_Zero(float *tar, int16_t *cur);

void Roll_FirstAngle_Ctrl(void);

void RC_Ctrl_PITCH_0(Arm_Info_t *str);
void RC_Ctrl_PITCH_1(Arm_Info_t *str);
void RC_Ctrl_ROOL_0(Arm_Info_t *str);
void RC_Ctrl_ROOL_1(Arm_Info_t *str);
void Roll_FirstAngle(Arm_Info_t *str);
void PITCH_ParamInit(Arm_Info_t *str);

void ROLL_ParamInit(Arm_Info_t *str);
void Angle8192_Over(float *angle);

void ctrl_enable(void);
void ctrl_disable(void);
void ctrl_set(void);
void ctrl_clear_para(void);
void ctrl_clear_err(void);
void ctrl_send(void);


void dm4310_Init(void);

extern Arm_Date_t Arm;

#endif
