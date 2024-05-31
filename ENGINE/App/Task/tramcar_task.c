/***      
°∂ …Ï’πøÿ÷∆¥˙¬Î°∑
***/
#include "tramcar_task.h"
#include "cmsis_os.h"


Tramcar_Date_t Tramcar={
	.State=SYSTEM_LOST,
	.Ctrl_Mode = Ctrl_Err,
	.PID_Type = Clear_Away,
};

float M_1[3] = {8, 0, 0};
float M_2[3] = {8, 0, 0};

float M_3[3] = {8, 0.1, 0};
float M_4[3] = {8, 0.1, 0};

/**
 *	@brief	
 */
extern Relative_Angle_t Relative_Angle;
float angle1 = 30;
pid_type_def M3508_0;
pid_type_def M3508_1;

pid_type_def M3508_2;
pid_type_def M3508_3;
pid_type_def M3508_4;
pid_type_def M3508_5;



void tramcar_task(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	PID_init(&Tramcar.PID[0], PID_POSITION, M_1, 12000, 100);
	PID_init(&Tramcar.PID[1], PID_POSITION, M_2, 12000, 100);
	PID_init(&Tramcar.PID[2], PID_POSITION, M_3, 3000, 100);
	PID_init(&Tramcar.PID[3], PID_POSITION, M_4, 3000, 100);
  /* Infinite loop */
  for(;;)
  {
//		if(RC_S2 == RC_SW_MID)
//		{
//			Tramcar_RC_Ctrl();
//		}
////		if(RC_S1 == RC_SW_DOWN)
////		{
////			
////		}
//		else{
//		PID_calc(&Tramcar.PID[0],Tramcar.Motor_Data[0].Motor_Data.CAN_GetData.Motor_Speed, 400);
//		PID_calc(&Tramcar.PID[1],Tramcar.Motor_Data[1].Motor_Data.CAN_GetData.Motor_Speed, -400);
//		}
//		CAN_cmd_tramcar_0(Tramcar.PID[0].out,Tramcar.PID[1].out,0,0);
//		CAN_cmd_tramcar  (Tramcar.PID[2].out,Tramcar.PID[3].out,0,0);
		osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

void Tramcar_RC_Ctrl(void)
{
	get_total_angle(&Tramcar.Motor_Data[0].Motor_Data.CAN_GetData);
	PID_calc(&Tramcar.PID[0], PID_calc(&Tramcar.PID[1], angle1, Relative_Angle.conversion_angle),
	Tramcar.Motor_Data[0].Motor_Data.CAN_GetData.Motor_Speed);

//	Shoot_pid_out = PID_Position(&Shoot.Motor_Data[DRIVER].PID.Speed_Loop,
//	PID_Position(&Shoot.Motor_Data[DRIVER].PID.Angle_Loop,-45.0f*36,Relative_Angle.conversion_angle),
//	Shoot.Motor_Data[DRIVER].Motor_Data.CAN_GetData.Motor_Speed);
	if(RC_CH3 !=0)
	{
		PID_calc(&Tramcar.PID[0],Tramcar.Motor_Data[0].Motor_Data.CAN_GetData.Motor_Speed, RC_CH3*3);
		PID_calc(&Tramcar.PID[1],Tramcar.Motor_Data[1].Motor_Data.CAN_GetData.Motor_Speed,-RC_CH3*3);
	}
	else{
			PID_calc(&Tramcar.PID[0],Tramcar.Motor_Data[0].Motor_Data.CAN_GetData.Motor_Speed, 400);
			PID_calc(&Tramcar.PID[1],Tramcar.Motor_Data[1].Motor_Data.CAN_GetData.Motor_Speed, -400);
//		PID_calc(&Tramcar.PID[2],Tramcar.Motor_Data[2].Motor_Data.CAN_GetData.Motor_Speed, 0);
//		PID_calc(&Tramcar.PID[3],Tramcar.Motor_Data[3].Motor_Data.CAN_GetData.Motor_Speed, 0);
	}
		PID_calc(&Tramcar.PID[2],Tramcar.Motor_Data[2].Motor_Data.CAN_GetData.Motor_Speed, -RC_CH1*3);
		PID_calc(&Tramcar.PID[3],Tramcar.Motor_Data[3].Motor_Data.CAN_GetData.Motor_Speed,  RC_CH1*3);
	
}


void Tramcar_KEY_Ctrl(void)
{
	//position_loop
	get_total_angle(&Tramcar.Motor_Data[0].Motor_Data.CAN_GetData);
	PID_calc(&Tramcar.PID[0], PID_calc(&Tramcar.PID[1], angle1, Relative_Angle.conversion_angle),
	Tramcar.Motor_Data[0].Motor_Data.CAN_GetData.Motor_Speed);
	
	
	//speed__loop
	PID_calc(&Tramcar.PID[0],Tramcar.Motor_Data[0].Motor_Data.CAN_GetData.Motor_Speed, RC_CH3*3);
	PID_calc(&Tramcar.PID[1],Tramcar.Motor_Data[1].Motor_Data.CAN_GetData.Motor_Speed,-RC_CH3*3);
	
	
}