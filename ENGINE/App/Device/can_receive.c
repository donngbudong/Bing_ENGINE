#include "can_receive.h"
#include "Device.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

static CAN_TxHeaderTypeDef  arm_tx_message;
static uint8_t              arm_can_send_data[8];

static CAN_TxHeaderTypeDef  tramcar_tx_message;
static uint8_t              tramcar_can_send_data[8];

CAN_GET_DATA_t motor_1;
void CAN_Date_Decode(CAN_GET_DATA_t *str, uint8_t *Data)                                    
{                         
	str->Motor_Angle = (uint16_t)((Data)[0] << 8 | (Data)[1]);            
	str->Motor_Speed = (uint16_t)((Data)[2] << 8 | (Data)[3]);      
	str->Motor_ELC   = (int16_t)((Data)[4] << 8 | (Data)[5]);  
	str->Motor_Temp  = (Data)[6];                                   
}


extern motor_t motor[num];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
		if(hcan->Instance==CAN1)
		{
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
			{
					case CAN_3508_M1_ID:
							CAN_Date_Decode(&Chassis.Motor_Data[CHAS_LF].Motor_Data.CAN_GetData, rx_data);                                    
						break;
					case CAN_3508_M2_ID:
							CAN_Date_Decode(&Chassis.Motor_Data[CHAS_RF].Motor_Data.CAN_GetData, rx_data);                                    
						break;
					case CAN_3508_M3_ID:
							CAN_Date_Decode(&Chassis.Motor_Data[CHAS_LB].Motor_Data.CAN_GetData, rx_data);                                    
						break;
					case CAN_3508_M4_ID:
							CAN_Date_Decode(&Chassis.Motor_Data[CHAS_RB].Motor_Data.CAN_GetData, rx_data);
						break;
					case 0x205:
							CAN_Date_Decode(&Tramcar.Motor_Data[0].Motor_Data.CAN_GetData, rx_data);                                    
						break;
					case 0x206:
							CAN_Date_Decode(&Tramcar.Motor_Data[1].Motor_Data.CAN_GetData, rx_data);                                    
						break;
					case 0x207:
							CAN_Date_Decode(&motor_1, rx_data);                                    
						break;
					default: {break;}	
			}
		}
		if(hcan->Instance==CAN2)
		{
			HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
			{
				case 0x201:
						CAN_Date_Decode(&Tramcar.Motor_Data[2].Motor_Data.CAN_GetData, rx_data);                                    
					break;
				case 0x202:
						CAN_Date_Decode(&Tramcar.Motor_Data[3].Motor_Data.CAN_GetData, rx_data);                                    
					break;
				case 0x205:
						CAN_Date_Decode(&Arm.ROLL[0].Motor_Data.CAN_GetData, rx_data);
					break;
				case 0x206:
						CAN_Date_Decode(&Arm.ROLL[1].Motor_Data.CAN_GetData, rx_data); 	
					break;
				case 0x05:	//0x01
						dm4310_fbdata(&Arm.PITCH[0].DM_Motor_Data, rx_data);					
					break;
				case 0x06:  //0x02
						dm4310_fbdata(&Arm.PITCH[1].DM_Motor_Data, rx_data);					
					break;
				default: {break;}	
			}
		}
}





/**
 * @brief CAN1���̵������
*/
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CHASSIS_ALL_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}


/**
 * @brief ARM_ROOL�������
*/
void CAN_cmd_ARM(int16_t rool1,int16_t rool2)
{
	
		uint32_t send_mail_box;

    arm_tx_message.StdId = 0x1FF;
    arm_tx_message.IDE = CAN_ID_STD;
    arm_tx_message.RTR = CAN_RTR_DATA;
    arm_tx_message.DLC = 0x04;
    
    arm_can_send_data[0] = rool1 >> 8;
    arm_can_send_data[1] = rool1;
    arm_can_send_data[2] = rool2 >> 8;
    arm_can_send_data[3] = rool2;
		HAL_CAN_AddTxMessage(&hcan2, &arm_tx_message, arm_can_send_data, &send_mail_box);

//	//�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
//	if(HAL_CAN_AddTxMessage(&hcan1, &arm_tx_message,arm_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//	if(HAL_CAN_AddTxMessage(&hcan1, &arm_tx_message,arm_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
//	{
//		HAL_CAN_AddTxMessage(&hcan1, &arm_tx_message,arm_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
//	}
//	}
}



void CAN_cmd_tramcar(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;

	tramcar_tx_message.StdId = CHASSIS_ALL_ID;
	tramcar_tx_message.IDE = CAN_ID_STD;
	tramcar_tx_message.RTR = CAN_RTR_DATA;
	tramcar_tx_message.DLC = 0x08;
	
	tramcar_can_send_data[0] = motor1 >> 8;
	tramcar_can_send_data[1] = motor1;
	tramcar_can_send_data[2] = motor2 >> 8;
	tramcar_can_send_data[3] = motor2;
	tramcar_can_send_data[4] = motor3 >> 8;
	tramcar_can_send_data[5] = motor3;
	tramcar_can_send_data[6] = motor4 >> 8;
	tramcar_can_send_data[7] = motor4;

		HAL_CAN_AddTxMessage(&hcan2, &tramcar_tx_message, tramcar_can_send_data, &send_mail_box);

	//�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
//	if(HAL_CAN_AddTxMessage(&hcan2, &tramcar_tx_message,tramcar_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//	if(HAL_CAN_AddTxMessage(&hcan2, &tramcar_tx_message,tramcar_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
//	{
//		HAL_CAN_AddTxMessage(&hcan2, &tramcar_tx_message,tramcar_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
//	}
//	}	
}

void CAN_cmd_tramcar_0(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;

	tramcar_tx_message.StdId = 0X1FF;
	tramcar_tx_message.IDE = CAN_ID_STD;
	tramcar_tx_message.RTR = CAN_RTR_DATA;
	tramcar_tx_message.DLC = 0x08;
	
	tramcar_can_send_data[0] = motor1 >> 8;
	tramcar_can_send_data[1] = motor1;
	tramcar_can_send_data[2] = motor2 >> 8;
	tramcar_can_send_data[3] = motor2;
	tramcar_can_send_data[4] = motor3 >> 8;
	tramcar_can_send_data[5] = motor3;
	tramcar_can_send_data[6] = motor4 >> 8;
	tramcar_can_send_data[7] = motor4;

		HAL_CAN_AddTxMessage(&hcan1, &tramcar_tx_message, tramcar_can_send_data, &send_mail_box);

	//�ҵ��յķ������䣬�����ݷ��ͳ�ȥ
//	if(HAL_CAN_AddTxMessage(&hcan2, &tramcar_tx_message,tramcar_can_send_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//	if(HAL_CAN_AddTxMessage(&hcan2, &tramcar_tx_message,tramcar_can_send_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) //
//	{
//		HAL_CAN_AddTxMessage(&hcan2, &tramcar_tx_message,tramcar_can_send_data, (uint32_t*)CAN_TX_MAILBOX2);
//	}
//	}	
}

/**
 *	@brief	����ϵ�Ƕ�=0��֮���������������2006�������Կ�����Ϊ0������ԽǶȡ�������ת��ת���ĽǶ����м��ٱ���Ŀ��Ƕ�Ҫ�˼��ٱȣ������á�
 */
Relative_Angle_t Relative_Angle;
void get_total_angle(CAN_GET_DATA_t *motor_get)
{
	Relative_Angle.angle=(float)motor_get->Motor_Angle;
	float res1, res2, delta;
	if(Relative_Angle.angle < Relative_Angle.lastAngle){			        //���ܵ����
		res1 = Relative_Angle.angle + 8192.0f - Relative_Angle.lastAngle;	//��ת  
		res2 = Relative_Angle.angle - Relative_Angle.lastAngle;				  //��ת	
	}else{	
		res1 = Relative_Angle.angle - 8192.0f -Relative_Angle.lastAngle ; //��ת	
		res2 = Relative_Angle.angle - Relative_Angle.lastAngle;				  //��ת	
	}
	
	/*��������ת���϶���ת�ĽǶ�С���Ǹ������*/ 
	//->ÿ����8000ת��ÿ����133ת��ÿ����0.133ת
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;
	Relative_Angle.totalAngle += delta;
	Relative_Angle.conversion_angle=Relative_Angle.totalAngle;
	Relative_Angle.lastAngle = Relative_Angle.angle;
}

/**
************************************************************************
* @brief:      	dm4310_enable: ����DM4310�������ģʽ����
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ��ģʽ��ͨ��CAN���߷�����������
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
************************************************************************
**/
void dm4310_enable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			enable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_disable: ����DM4310�������ģʽ����
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ��ģʽ��ͨ��CAN���߷��ͽ�������
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
************************************************************************
**/
void dm4310_disable(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			disable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			disable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			disable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
	dm4310_clear_para(motor);
}
/**
************************************************************************
* @brief:      	dm4310_ctrl_send: ����DM4310������������
* @param[in]:   hcan:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���ݵ������ģʽ������Ӧ�����DM4310���
*               ֧�ֵĿ���ģʽ����λ��ģʽ��λ���ٶȿ���ģʽ���ٶȿ���ģʽ
************************************************************************
**/
void dm4310_ctrl_send(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case 1:
			pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case 2:
			speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_set: ����DM4310������Ʋ�������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	���������������DM4310����Ŀ��Ʋ���������λ�á��ٶȡ�
*               ��������(KP)��΢������(KD)��Ť��
************************************************************************
**/
void dm4310_set(motor_t *motor)
{
	motor->ctrl.kd_set 	= motor->cmd.kd_set;
	motor->ctrl.kp_set	= motor->cmd.kp_set;
	motor->ctrl.pos_set	= motor->cmd.pos_set;
	motor->ctrl.vel_set	= motor->cmd.vel_set;
	motor->ctrl.tor_set	= motor->cmd.tor_set;

}
/**
************************************************************************
* @brief:      	dm4310_clear: ���DM4310������Ʋ�������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬������������Ϣ�Ϳ��Ʋ���
* @retval:     	void
* @details:    	��DM4310�������������Ϳ��Ʋ������㣬����λ�á��ٶȡ�
*               ��������(KP)��΢������(KD)��Ť��
************************************************************************
**/
void dm4310_clear_para(motor_t *motor)
{
	motor->cmd.kd_set 	= 0;
	motor->cmd.kp_set	 	= 0;
	motor->cmd.pos_set 	= 0;
	motor->cmd.vel_set 	= 0;
	motor->cmd.tor_set 	= 0;
	
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
}
/**
************************************************************************
* @brief:      	dm4310_clear_err: ���DM4310���������
* @param[in]:   hcan: 	 ָ��CAN���ƽṹ���ָ��
* @param[in]:  	motor:   ָ�����ṹ���ָ��
* @retval:     	void
* @details:    	���ݵ���Ŀ���ģʽ�����ö�Ӧģʽ�����������
************************************************************************
**/
void dm4310_clear_err(hcan_t* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			clear_err(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			clear_err(hcan, motor->id, POS_MODE);
			break;
		case 2:
			clear_err(hcan, motor->id, SPEED_MODE);
			break;
	}	
}
/**
************************************************************************
* @brief:      	dm4310_fbdata: ��ȡDM4310����������ݺ���
* @param[in]:   motor:    ָ��motor_t�ṹ��ָ�룬������������Ϣ�ͷ�������
* @param[in]:   rx_data:  ָ������������ݵ�����ָ��
* @retval:     	void
* @details:    	�ӽ��յ�����������ȡDM4310����ķ�����Ϣ���������ID��
*               ״̬��λ�á��ٶȡ�Ť���Լ�����¶Ȳ���
************************************************************************
**/
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

/**
************************************************************************
* @brief:      	float_to_uint: ������ת��Ϊ�޷�����������
* @param[in]:   x_float:	��ת���ĸ�����
* @param[in]:   x_min:		��Χ��Сֵ
* @param[in]:   x_max:		��Χ���ֵ
* @param[in]:   bits: 		Ŀ���޷���������λ��
* @retval:     	�޷����������
* @details:    	�������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷�������
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: �޷�������ת��Ϊ����������
* @param[in]:   x_int: ��ת�����޷�������
* @param[in]:   x_min: ��Χ��Сֵ
* @param[in]:   x_max: ��Χ���ֵ
* @param[in]:   bits:  �޷���������λ��
* @retval:     	���������
* @details:    	���������޷������� x_int ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��������
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������ض�ģʽ������
************************************************************************
**/
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ���õ�ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͽ����ض�ģʽ������
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	save_pos_zero: ����λ����㺯��
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ����λ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͱ���λ����������
************************************************************************
**/
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	clear_err: ������������
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ��������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������������
************************************************************************
**/
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	mit_ctrl: MITģʽ�µĵ�����ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   kp:				λ�ñ���ϵ��
* @param[in]:   kd:				λ��΢��ϵ��
* @param[in]:   torq:			ת�ظ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������MITģʽ�µĿ���֡��
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: λ���ٶȿ��ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   vel:			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������λ���ٶȿ�������
************************************************************************
**/
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: �ٶȿ��ƺ���
* @param[in]:   hcan: 		ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   vel: 			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN�������������ٶȿ�������
************************************************************************
**/
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}



uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{	
	CAN_TxHeaderTypeDef	tx_header;
	
	tx_header.StdId = id;
	tx_header.ExtId = 0;
	tx_header.IDE   = 0;
	tx_header.RTR   = 0;
	tx_header.DLC   = len;
  /*�ҵ��յķ������䣬�����ݷ��ͳ�ȥ*/
	if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
		if(HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK) {
			HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }
  return 0;
}

/**
************************************************************************
* @brief:      	canx_send_data: �û������㷢�ͺ���
* @param:       hcan: CAN���
* @param:       id: 	CAN�豸ID
* @param:       data: ���͵�����
* @param:       len:  ���͵����ݳ���
* @retval:     	void
* @details:    	CAN���߷�������
************************************************************************
**/
void canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	canx_bsp_send_data(hcan, id, data, len);
}
