#ifndef __PID_H_
#define __PID_H_
/************************************** Includes **************************************/
#include "stm32f4xx_hal.h"
//#include "arm_task.h"
/********************************* Exported functions *********************************/

typedef enum
{
	Clear_Away  =0,	    //���
	KEY         =1,			//����
	RC          =2,			//ң��
	PID_TYPE_CNT,
}PID_Type_t;


/* PID���� */
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
	
  float I_Limit;				//I�����޷�
  float I_Limit_Max;	
	
  float PID_P_Out;
  float PID_I_Out;
  float PID_D_Out;
  float PID_Output;
  float PID_Err_Dead;			//����
  float PID_Output_Max;		//����޷�
  float PID_I_Out_Max;		
  float PID_P_Out_Max;
}PID_Loop_t;

/*PID���ݽṹ��*/
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
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

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
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
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
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern float PID_calc(pid_type_def *pid, float ref, float set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);
#endif
