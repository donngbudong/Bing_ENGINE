#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stm32f4xx.h"                  // Device header
#include "pid.h"
typedef CAN_HandleTypeDef hcan_t;

#define ABS(x)	( (x>0) ? (x) : (-x) ) /*���x�ľ���ֵ*/
#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f


// ����ش���Ϣ�ṹ��
typedef struct 
{
	int id;			//���ID
	int state;	//���״̬8������ѹ������Ƿѹ������������������MOS ���£�C���������Ȧ���£�D����ͨѶ��ʧ��E�������أ�
	int p_int;	//�����λ����Ϣ
	int v_int;	//������ٶ���Ϣ
	int t_int;	//�����Ť����Ϣ
	int kp_int;
	int kd_int;
	float pos;	//�����λ����Ϣ
	float vel;	//������ٶ���Ϣ
	float tor;	//�����Ť����Ϣ
	float Kp;	
	float Kd;
	float Tmos;	//������ MOS ��ƽ���¶�
	float Tcoil;//����ڲ���Ȧ��ƽ���¶�
}motor_fbpara_t;

// ����������ýṹ��
typedef struct 
{
	int8_t mode;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
}motor_ctrl_t;

typedef struct
{
	int8_t id;
	uint8_t start_flag;
	motor_fbpara_t para;
	motor_ctrl_t ctrl;
	motor_ctrl_t cmd;
}motor_t;

typedef enum
{
	Motor1,
	Motor2,
	num
} motor_num;

/* �������ö�� */
typedef enum 
{
  M_3508 	= 0,
	GM_6020 = 1,
	DM_4310 = 2,
  MOTOR_TYPE_CNT,
}Motor_Type_t;

/*���̵��ID��Ϣ*/
typedef enum
{
	CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
}CHASSIS_MOTOR_ID;

///* ��չ���ID��Ϣ */
//typedef enum 
//{
//	TRAMCAR_ALL_ID 	= 0x200,	 	//��̨���ID
//  TRAMCAR_M1_ID		= 0x201,   	//UP
//  TRAMCAR_M2_ID   = 0x202,		//UP
//	TRAMCAR_M3_ID		= 0x203,	 	//FRONT
//	TRAMCAR_M4_ID		= 0x204,		//FRONT
//}TRAMCAR_MOTOR_ID;

/* ��е�۵��ID��Ϣ */
typedef enum 
{
	ARM_ALL_ID 	= 0x1FF,		//��̨���ID
  ARM_M1_ID		= 0x205,   	//YAW1
  ARM_M2_ID   = 0x206,   	//YAW2

}ARM_MOTOR_ID;

/*�����Ϣ�ṹ��*/
typedef struct 
{
  Motor_Type_t Motor_Type;
}Motor_Info_t;


/*CAN���յ������� �ṹ��*/
typedef  struct 
{
  int16_t Motor_Angle;
	int16_t Motor_Speed;
  int16_t Motor_ELC;
  uint8_t Motor_Temp;   
}CAN_GET_DATA_t;


/*����PID������ �ṹ��*/
typedef  struct 
{
  float angle;						//�����е�Ƕ�
	float lastAngle;				//�����һ�̻�е�Ƕ�
	float totalAngle;				//����ܵĻ�е�Ƕ�
	float conversion_angle;	//���ת���Ƕ�
}Relative_Angle_t;


/*���IMU���յ������� �ṹ��*/
typedef __packed struct
{
  float IMU_Speed;
  float IMU_Angle;
}IMU_CAN_DATA_t;

typedef struct
{
	CAN_GET_DATA_t CAN_GetData;
	IMU_CAN_DATA_t IMU_GetData;
  /* ��������pid���� */
  float PID_Speed;
  float PID_Speed_target;
  float PID_Angle; 
  float PID_Angle_target; 
}Motor_Data_t;






void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_ARM(int16_t rool1,int16_t rool2);
void CAN_cmd_tramcar(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_tramcar_0(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void get_total_angle(CAN_GET_DATA_t *motor_get);



float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
void dm4310_ctrl_send(hcan_t* hcan, motor_t *motor);
void dm4310_enable(hcan_t* hcan, motor_t *motor);
void dm4310_disable(hcan_t* hcan, motor_t *motor);
void dm4310_set(motor_t *motor);
void dm4310_clear_para(motor_t *motor);
void dm4310_clear_err(hcan_t* hcan, motor_t *motor);
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data);

void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);
void save_pos_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
void canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);

extern PID_Parameter_t PID_Speed_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT];
extern PID_Parameter_t PID_Angle_Param[MOTOR_TYPE_CNT][PID_TYPE_CNT];

#endif

