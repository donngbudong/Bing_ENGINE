/***      
�� ��е�ۿ��ơ�
***/
#include "arm_task.h"
#include "Device.h"
#include "math.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
uint8_t motor_id;
motor_t motor[num];

static PID_Type_t Arm_pid_type= Clear_Away; 

/* Init start */
Arm_Date_t Arm ={
	.State=SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
};


pid_type_def GM_6020_0;
pid_type_def GM_6020_1;
pid_type_def GM_6020_2;
pid_type_def GM_6020_3;


float Rool_0[3] = {17, 0, 0};
float Rool_1[3] = {15, 0, 0};

float Rool_2[3] = {6, 0, 0};
float Rool_3[3] = {20, 0, 0};

float pos_1 = 0;
float pos_2 = 0;
int16_t angle = 3000;
void arm_task(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	vTaskDelay(2000);
	dm4310_Init();
	PITCH_ParamInit(&Arm.PITCH[0]);
	PITCH_ParamInit(&Arm.PITCH[1]);
	ROLL_ParamInit(&Arm.ROLL[0]);
	ROLL_ParamInit(&Arm.ROLL[1]);

	PID_init(&GM_6020_0, PID_POSITION, Rool_0, 24000, 100);
	PID_init(&GM_6020_1, PID_POSITION, Rool_1, 24000, 100);
	PID_init(&GM_6020_2, PID_POSITION, Rool_2, 24000, 100);
	PID_init(&GM_6020_3, PID_POSITION, Rool_3, 24000, 100);
	
  /* Infinite loop */
  for(;;)
  {
//		silver_ore();
//		Arm_Task();
    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}


/**
 * @brief Arm�ܿ�
 * @param 
 */
void Arm_Task(void)
{
	Arm_GET_Info();
	Arm_RC_Ctrl();
	 if(RC_S1 == RC_SW_DOWN)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	else if(RC_S1 == RC_SW_MID)
	{
		if(Arm_pid_type != Arm.PID_Type)
			{/* ��һ�ν���ģʽ */
				Arm_pid_type = Arm.PID_Type;
			}	
	}
	
}
/**
* @brief Arm��Ϣ��ȡ
* @param 
*/
void Arm_GET_Info(void)
{
	Arm.State = System.System_State;
	Arm.PID_Type = System.System_Pid;
}


void dm4310_Init(void)
{
	enable_motor_mode(&hcan2,0x01,POS_MODE);//��
	enable_motor_mode(&hcan2,0x02,POS_MODE);//��
	
	HAL_Delay(200);
//	save_pos_zero(&hcan2,0x01,POS_MODE);
//	save_pos_zero(&hcan2,0x02,POS_MODE);
}


/**
 * @brief ң����
 * @param 
 */
void Arm_RC_Ctrl(void)
{
	//ROOL_0
	Handle_Angle8191_PID_Over_Zero(&Arm.ROLL[0].Motor_Data.PID_Angle_target,&Arm.ROLL[0].Motor_Data.CAN_GetData.Motor_Angle);
	PID_calc(&GM_6020_0, Arm.ROLL[0].Motor_Data.CAN_GetData.Motor_Angle, Arm.ROLL[0].Motor_Data.PID_Angle_target);
	PID_calc(&GM_6020_1, Arm.ROLL[0].Motor_Data.CAN_GetData.Motor_Speed, GM_6020_0.out);

	//ROOL_1
	Handle_Angle8191_PID_Over_Zero(&Arm.ROLL[1].Motor_Data.PID_Angle_target,&Arm.ROLL[1].Motor_Data.CAN_GetData.Motor_Angle);
	PID_calc(&GM_6020_2, Arm.ROLL[1].Motor_Data.CAN_GetData.Motor_Angle, Arm.ROLL[1].Motor_Data.PID_Angle_target);
	PID_calc(&GM_6020_3, Arm.ROLL[1].Motor_Data.CAN_GetData.Motor_Speed, GM_6020_2.out);

	if(RC_S2==RC_SW_UP)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		
		if(RC_S1 == RC_SW_UP){
		Roll_FirstAngle_Ctrl();				//�״ν�ȥ��ģʽ
		
		RC_Ctrl_ROOL_0(&Arm.ROLL[0]);
		RC_Ctrl_PITCH_0(&Arm.PITCH[0]);
		RC_Ctrl_ROOL_1(&Arm.ROLL[1]);
		RC_Ctrl_PITCH_1(&Arm.PITCH[1]);
		}
//		if(RC_S1 ==RC_SW_MID){
//			
//			silver_ore();
//		}
		
		pos_speed_ctrl(&hcan2,0x01,Arm.PITCH[0].Motor_Data.PID_Angle_target,1);
		pos_speed_ctrl(&hcan2,0x02,Arm.PITCH[1].Motor_Data.PID_Angle_target,1);
		
	}

	if(RC_S2 == RC_SW_DOWN)
	{
		enable_motor_mode(&hcan2,0x01,POS_MODE);//��
		enable_motor_mode(&hcan2,0x02,POS_MODE);//��
	}
	if(RC_S1 == RC_SW_MID)
	{
		CAN_cmd_ARM(0,0);
	}else{
		CAN_cmd_ARM(GM_6020_1.out,GM_6020_3.out);
	}
}	


/**
 * @brief	һ��ȡ��ʯ
 * @param 
 */
//rool_0 2625-6790
//pitch_0			-0.303
//rool_1			646
//pitch_1			0.206

uint16_t angle_11= 200;
uint16_t tt;
uint16_t yy;
void silver_ore(void)
{
	Arm.ROLL[0].Motor_Data.PID_Angle_target = ramp_control(Arm.ROLL[0].Motor_Data.CAN_GetData.Motor_Angle,6790,0.1);
	Arm.PITCH[0].Motor_Data.PID_Angle_target = ramp_control(Arm.PITCH[0].DM_Motor_Data.para.pos,-0.303,0.01);

	Arm.ROLL[1].Motor_Data.PID_Angle_target = ramp_control(Arm.ROLL[1].Motor_Data.CAN_GetData.Motor_Angle,646,0.08);
	
	Arm.PITCH[1].Motor_Data.PID_Angle_target = ramp_control(Arm.PITCH[1].DM_Motor_Data.para.pos,0.206,0.01);
	

}
/**
 * @brief GM_6020��Ƕȹ��㴦��
 * @param 
 */
/* �Ƕ�Pidʱ���ڸ���tar��cur֮������ŵ���, �������ٽ���PID����*/
void Handle_Angle8191_PID_Over_Zero(float *tar, int16_t *cur)
{
	if(*tar - *cur > 4096)    //4096 ����Ȧ��е�Ƕ�
	{
		*cur += 8192;        //8191,8192����ν�ˣ���������
	}
	else if(*tar - *cur < -4096)
	{
		*cur = *cur - 8192;
	}
}


void Roll_FirstAngle_Ctrl(void)
{
	if(Arm_pid_type != Arm.PID_Type)
		{/* ��һ�ν���ģʽ */
			Arm_pid_type = Arm.PID_Type;
			Roll_FirstAngle(&Arm.ROLL[0]);
			Roll_FirstAngle(&Arm.ROLL[1]);
		}
}

/**
 * @brief ROOL_0��Ŀ��� ң����ң��ģʽ
 * @param 
 */
void RC_Ctrl_ROOL_0(Arm_Info_t *str)
{
	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * (-RC_CH2);
		
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;

	Angle8192_Over(&str->Motor_Data.PID_Angle_target);
}

/**
 * @brief PITCH_0��Ŀ��� ң����ң��ģʽ
 * @param 
 */
void RC_Ctrl_PITCH_0(Arm_Info_t *str)
{
	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * (RC_CH3);
		
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;
	dm_LimitMax(str->Motor_Data.PID_Angle_target,0.55,-0.85);
}

/**
 * @brief ROOL_1��Ŀ��� ң����ң��ģʽ
 * @param 
 */
void RC_Ctrl_ROOL_1(Arm_Info_t *str)
{
	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * (-RC_CH0);
		
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;

}

/**
 * @brief PICTCH_1��Ŀ��� ң����ң��ģʽ
 * @param 
 */
void RC_Ctrl_PITCH_1(Arm_Info_t *str)
{
	str->RC_Move.Angle_Inc = str->RC_Move.Angle_k * (RC_CH1);
		
	str->Motor_Data.PID_Angle_target += str->RC_Move.Angle_Inc;
	
	dm_LimitMax(str->Motor_Data.PID_Angle_target,2.3,-1.2);

}
/**
 * @brief ��ʼ��ROLL��Ƕ�
 * @param 
 */
void Roll_FirstAngle(Arm_Info_t *str)
{
  str->Motor_Data.PID_Angle_target = str->Motor_Data.CAN_GetData.Motor_Angle;
}

/**
 * @brief ROLL��Ƕȹ�8192����
 *
 */
void Angle8192_Over(float *angle)
{
	if(*angle > 8192)
	{
		*angle -=8191;
	}
	if(*angle < 0)
	{
		*angle += 8192;
	}
}

/**
 * @brief ROLL���PID�޷�����
 * @param 
 */
void ROLL_ParamInit(Arm_Info_t *str)
{
  /* ң��ģʽ */
  str->RC_Move.Angle_Inc = 0;
  str->RC_Move.Angle_k = 0.01;
  str->RC_Move.Slow_Inc = 0;
  /* ����ģʽ */
  str->KEY_Move.Angle_Inc = 0;
  str->KEY_Move.Angle_k = 0.004;//0.001
  str->KEY_Move.Slow_Inc = 0;
	
}
/**
 * @brief PICTH���PID�޷�����
 * @param 
 */
void PITCH_ParamInit(Arm_Info_t *str)
{
  /* ң��ģʽ */
  str->RC_Move.Angle_Inc = 0;
  str->RC_Move.Angle_k = 0.00001;
  str->RC_Move.Slow_Inc = 0;
  /* ����ģʽ */
  str->KEY_Move.Angle_Inc = 0;
  str->KEY_Move.Angle_k = 0.004;//0.001
  str->KEY_Move.Slow_Inc = 0;
	
}


///**
// * @brief ����
// * @param 
// */
//void Chassis_KEY_Ctrl(void)
//{

//	if(KEY_W == 1){
//		Chassis.KEY_Move.FB_Speed += 500;
//	}
//	else if(KEY_S == 1){
//		Chassis.KEY_Move.FB_Speed -= 500;
//	}
//	else if(KEY_A == 1){
//		Chassis.KEY_Move.LR_Speed -= 500;
//	}
//	else if(KEY_D == 1){
//		Chassis.KEY_Move.LR_Speed += 500;
//	}
//	else{
//		Chassis.KEY_Move.FB_Speed = 0;
//		Chassis.KEY_Move.LR_Speed = 0;
//	}
//	XY_Speed(&Chassis.KEY_Move.FB_Speed,5000,-5000);
//	XY_Speed(&Chassis.KEY_Move.LR_Speed,5000,-5000);
//	Chassis.KEY_Move.Z_Speed  = PID_Position(&Chassis.YAW_Follow.PID.Speed_Loop,
//																							MECH_Mid_Angle,yaw_angle_t);
//	Chassis_Handler(&Chassis.KEY_Move);
//}

///**
// * @brief �õ������ĸ�����Ľ����ٶ�Ŀ��ֵ
// * @note  ����ķ������ 
// */
//float Speed_Adapt(float FB,float LR,float Z)
//{
//  float res;
//  float Adapt_K = 1;
////  if(abs(FB)+abs(LR)+abs(Z) > Speed_MAX)
////    Adapt_K = (Speed_MAX)/(abs(FB)+abs(LR)+abs(Z));
//  res = FB*Adapt_K  + LR*Adapt_K  + Z*Adapt_K ;
//  return res;
//}
////�����ٶȷ��䵽�����ٶ�
//void Chassis_Allot_Speed(Chassis_Move_t *str)
//{
//	str->Motor_SpeedTarget[CHAS_LF] = Speed_Adapt(-str->FB_Speed,
//																								 str->LR_Speed,	
//																								 str->Z_Speed);
//	str->Motor_SpeedTarget[CHAS_RF] = Speed_Adapt( str->FB_Speed,
//																								 str->LR_Speed,	
//																								 str->Z_Speed);
//	str->Motor_SpeedTarget[CHAS_LB] = Speed_Adapt( str->FB_Speed,
//																								-str->LR_Speed,	
//																								 str->Z_Speed);
//	str->Motor_SpeedTarget[CHAS_RB] = Speed_Adapt(-str->FB_Speed,
//																								-str->LR_Speed,	
//																								 str->Z_Speed);
//}


///**
// * @brief ��MMKNM���ٶȽ��� б�·ָ��ĸ������PIDĿ��ֵ
// * @param 
// */
//void Chassis_MotorSpeed(Chassis_Move_t *str)
//{
//	Chassis.Motor_Data[CHAS_LF].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_LF];
//	Chassis.Motor_Data[CHAS_RF].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_RF];
//	Chassis.Motor_Data[CHAS_LB].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_LB];
//	Chassis.Motor_Data[CHAS_RB].Motor_Data.PID_Speed_target = str->Motor_SpeedTarget[CHAS_RB];
//}


///**
// * @brief PID���
//*/
//float Chassis_GetOutPut(Chassis_Motor_t *str)
//{
//	PID_Info_t *PID = &(str->PID);
////  Motor_Data_t *DATA = &(str->Motor_Data);
//  float res;
//	  /*����ֵ = �ٶȻ�PID���*/
//  res = PID_Position(&PID->Speed_Loop,  										 \
//										str->Motor_Data.PID_Speed_target,				 \
//										str->Motor_Data.CAN_GetData.Motor_Speed);\
//  return res;
//}

///**
// * @brief Chassis������
// * @param 
// */
//void Chassis_CanOutPut(void)
//{
//	static int16_t pid_out[CHAS_MOTOR_CNT] = {0,0,0,0};
//	pid_out[CHAS_LF] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_LF]);
//	pid_out[CHAS_RF] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_RF]);
//	pid_out[CHAS_LB] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_LB]);
//	pid_out[CHAS_RB] = (int16_t)Chassis_GetOutPut(&Chassis.Motor_Data[CHAS_RB]);
//	CAN_cmd_chassis(pid_out[CHAS_LF],pid_out[CHAS_RF],pid_out[CHAS_LB],pid_out[CHAS_RB]);
//}

///**
// * @brief Chassis�������
// * @param 
// */
//void Chassis_Handler(Chassis_Move_t *str)
//{
//	Chassis_Allot_Speed(str);
//	Chassis_MotorSpeed(str);
//	Chassis_CanOutPut();
//}


///**
// * @brief ж������
// * @param 
// */
//void Chassis_Stop(void)
//{
//	static int16_t pid_out[CHAS_MOTOR_CNT] = {0, 0, 0, 0};
//	
//	/* �ٶȻ�������� */
//  pid_out[CHAS_LF] = 0;
//  pid_out[CHAS_RF] = 0;
//  pid_out[CHAS_LB] = 0;
//  pid_out[CHAS_RB] = 0;
//  
//	CAN_cmd_chassis(pid_out[CHAS_LF],pid_out[CHAS_RF],pid_out[CHAS_LB],pid_out[CHAS_RB]);	  
//}



/**
************************************************************************
* @brief:      	ctrl_enable: ���õ�����ƺ���
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ�ĵ�����ơ�
*               ����ָ�������������־��������dm4310_enable�������õ����
************************************************************************
**/
void ctrl_enable(void)
{
	switch(motor_id)
	{
		case 1:
			// ����Motor1�ĵ������
			motor[Motor1].start_flag = 1;
			dm4310_enable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// ����Motor2�ĵ������
			motor[Motor2].start_flag = 1;
			dm4310_enable(&hcan2, &motor[Motor2]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_disable: ���õ�����ƺ���
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ�ĵ�����ơ�
*               ����ָ�������������־Ϊ0��������dm4310_disable�������õ����
************************************************************************
**/
void ctrl_disable(void)
{
	switch(motor_id)
	{
		case 1:
			// ����Motor1�ĵ������
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// ����Motor2�ĵ������
			motor[Motor2].start_flag = 0;
			dm4310_disable(&hcan2, &motor[Motor2]);
			break;

	}
}
/**
************************************************************************
* @brief:      	ctrl_set: ���õ����������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ����Ĳ�����
*               ����dm4310_set��������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_set(void)
{
	switch(motor_id)
	{
		case 1:
			// ����Motor1�ĵ������
			dm4310_set(&motor[Motor1]);
			break;
		case 2:
			// ����Motor2�ĵ������
			dm4310_set(&motor[Motor2]);
			break;

	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_para: ��������������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id���������Ӧ����Ĳ�����
*               ����dm4310_clear�������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_clear_para(void)
{
	switch(motor_id)
	{
		case 1:
			// ���Motor1�ĵ������
			dm4310_clear_para(&motor[Motor1]);
			break;
		case 2:
			// ���Motor2�ĵ������
			dm4310_clear_para(&motor[Motor2]);
			break;

	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: ������������Ϣ
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id���������Ӧ����Ĳ�����
*               ����dm4310_clear�������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_clear_err(void)
{
	switch(motor_id)
	{
		case 1:
			// ���Motor1�ĵ���������
			dm4310_clear_err(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// ���Motor2�ĵ���������
			dm4310_clear_err(&hcan2, &motor[Motor2]);
			break;

	}
}

/**
************************************************************************
* @brief:      	ctrl_send: ���͵�����������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������Ӧ������Ϳ������
*               ����dm4310_ctrl_send������ָ��������Ϳ����������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_send(void)
{
	switch(motor_id)
	{
		case 1:
			 // ��Motor1���Ϳ�������
			dm4310_ctrl_send(&hcan1, &motor[Motor1]);
			break;
		case 2:
			 // ��Motor2���Ϳ�������
			dm4310_ctrl_send(&hcan2, &motor[Motor2]);
			break;
	}
}
