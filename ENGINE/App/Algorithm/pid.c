#include "pid.h"
#include "Device.h"


//反限幅（死区）
float anti_constrain(float amt,float high,float low)
{
	if (amt > low && amt < high)
		return 0;
	else
		return amt;
}


/* 用于调试PID参数的函数 */
PID_Parameter_t PID_SpeedDebug = {
  .P = 0,
  .I = 0,
  .D = 0,
};
PID_Parameter_t PID_AngleDebug = {
  .P = 0,
  .I = 0,
  .D = 0,
};
void PID_Debug(PID_Info_t *str)
{
  str->Speed_Loop.PID_Param.P = PID_SpeedDebug.P;
  str->Speed_Loop.PID_Param.I = PID_SpeedDebug.I;
  str->Speed_Loop.PID_Param.D = PID_SpeedDebug.D;
  
  str->Angle_Loop.PID_Param.P = PID_AngleDebug.P;
  str->Angle_Loop.PID_Param.I = PID_AngleDebug.I;
  str->Angle_Loop.PID_Param.D = PID_AngleDebug.D;
}
			
/**
  * @brief  PID(速度/角度)环控制器
  * @param  目标-真实
  * @retval None
  */
float PID_Position(PID_Loop_t *pid, float target, float actual)
{
	pid->Target=target;
	pid->Actual=actual;
	//误差=目标值-实际值
	pid->Err = target - actual;
  	/*死区*/
  pid->Err = anti_constrain(pid->Err,pid->PID_Err_Dead,-pid->PID_Err_Dead);
	//积分限幅
	pid->I_Limit = pid->I_Limit + pid->Err;
	LimitMax(pid->I_Limit,pid->I_Limit_Max);

	pid->PID_P_Out = pid->PID_Param.P * pid->Err;
	pid->PID_I_Out = pid->PID_Param.I * pid->I_Limit;
	pid->PID_D_Out = pid->PID_Param.D * (pid->Err - pid->Last_Err);
	//I输出限幅
	LimitMax(pid->PID_I_Out,pid->PID_I_Out_Max);
	pid->PID_Output = pid->PID_P_Out+pid->PID_I_Out+pid->PID_D_Out;
	
	pid->Last_Err = pid->Err;
	//总输出限幅
	LimitMax(pid->PID_Output, pid->PID_Output_Max);
	
	return pid->PID_Output;
}



PID_Parameter_t PID_Speed_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
  [M_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 6,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
};

PID_Parameter_t PID_Angle_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT]={
  [M_3508] = {
		 [Clear_Away]   = {
      .P = 0,
      .I = 0,
      .D = 0,
    },
    [RC]   = {
      .P = 6,
      .I = 0,
      .D = 0,
    },
		[KEY]	 = {
      .P = 5,
      .I = 0,
      .D = 0,
		},
	},
};




/**
 * @brief 总电机初始化
 * @param 
 */
void Motor_Init(void)
{
  M_3508_Init();
}

/*底盘M3508电机---------------------------------------*/
/**
 * @brief 底盘电机初始化
 * @param 
 */
void M_3508_Init(void)
{
	Chassis.Motor_Info.Motor_Type = M_3508;
	M_3508_ParamInit(&Chassis.Motor_Data[CHAS_LF].PID.Speed_Loop);
  M_3508_ParamInit(&Chassis.Motor_Data[CHAS_RF].PID.Speed_Loop);
  M_3508_ParamInit(&Chassis.Motor_Data[CHAS_LB].PID.Speed_Loop);
  M_3508_ParamInit(&Chassis.Motor_Data[CHAS_RB].PID.Speed_Loop);	
}



/**
 * @brief PID限幅设置 
 * @param 
 */
void M_3508_ParamInit(PID_Loop_t *str)
{
  /* 这里只针对速度环 */
  str->I_Limit_Max = 50000;
  str->PID_I_Out_Max = 10000; 
  str->PID_P_Out_Max = 50000;
  str->PID_Output_Max = 10000;
  str->PID_Err_Dead = 5;
}



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
void PID_init(pid_type_def *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

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
float PID_calc(pid_type_def *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}




