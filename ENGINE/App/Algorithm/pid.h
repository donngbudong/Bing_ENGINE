#ifndef __PID_H_
#define __PID_H_
/************************************** Includes **************************************/
#include "stm32f4xx_hal.h"
//#include "arm_task.h"
/********************************* Exported functions *********************************/

typedef enum
{
	Clear_Away  =0,	    //清除
	KEY         =1,			//键盘
	RC          =2,			//遥控
	PID_TYPE_CNT,
}PID_Type_t;


/* PID参数 */
typedef __packed struct 
{
  float P,I,D;
}PID_Parameter_t;


typedef __packed struct
{
  PID_Parameter_t PID_Param;
	float Target;
	float Actual;
  float Err;
  float Last_Err;
	
  float I_Limit;				//I积分限幅
  float I_Limit_Max;	
	
  float PID_P_Out;
  float PID_I_Out;
  float PID_D_Out;
  float PID_Output;
  float PID_Err_Dead;			//死区
  float PID_Output_Max;		//输出限幅
  float PID_I_Out_Max;		
  float PID_P_Out_Max;
}PID_Loop_t;

/*PID数据结构体*/
typedef struct 
{
  PID_Loop_t Speed_Loop;
	PID_Loop_t Angle_Loop;
  PID_Type_t PID_Type;
}PID_Info_t;


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_Debug(PID_Info_t *str);

float PID_Position(PID_Loop_t *pid, float target, float actual);

void Motor_Init(void);
		
void M_3508_Init(void);
void M_3508_ParamInit(PID_Loop_t *str);
		
//void ROLL_ParamInit(Arm_Info_t *str);
//void GIM_PitParamInit(Gimbal_Info_t *str);
void YAW_Follow_PitParamInit(PID_Loop_t *str);

void GM_6020_Init(void);
		
void M_2006_Init(void);
void M_2006_ParamInit(void);
void Fric_3508_Init(void);
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);
#endif
