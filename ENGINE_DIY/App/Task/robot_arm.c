#include "robot_arm.h"
#include "cmsis_os.h"
#include "crc.h"
#include "string.h"
#include "usart.h"
#include "HI229.h"






/***********����������******************/

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);

/*************ȫ�ֱ�����*****************/
uint8_t data[DATA_LENGTH];
DIY_data_t DIY_data;
extern IMU_GET_DATA_t IMU_Get_Data;
Controller_t tx_data; // �Զ�����������͵�����
void BING_DIY(void const * argument)
{
  /* USER CODE BEGIN IMU_TASK */
  /* Infinite loop */
    // uint8_t index = 0;
    uint32_t wait_time = xTaskGetTickCount();
    for (;;)
    {		
			DIY_data.rool_0 = 15;
			DIY_data.picth = IMU_Get_Data.IMU_Eular[0];
			DIY_data.yaw	 = IMU_Get_Data.IMU_Eular[2];
			DIY_data.rool_1= 100;
			memcpy(&DIY_data, data, sizeof(DIY_data));
			Data_Concatenation(data, DATA_LENGTH);
			HAL_UART_Transmit(&huart4, (uint8_t *)(&tx_data), sizeof(tx_data), 50);
			osDelayUntil(&wait_time, 500);
    }
  /* USER CODE END IMU_TASK */
}



/**
 * @brief ����ƴ�Ӻ�������֡ͷ�������롢���ݶΡ�֡βͷƴ�ӳ�һ������
 * @param data ���ݶε�����ָ��
 * @param data_lenth ���ݶγ���
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// ֡ͷ����
    tx_data.frame_header.sof = 0xA5;                              // ����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5
    tx_data.frame_header.data_length = data_lenth;                // ����֡�����ݶεĳ���
    tx_data.frame_header.seq = seq++;                             // �����
    Append_CRC8_Check_Sum((uint8_t *)(&tx_data.frame_header), 5); // ���֡ͷ CRC8 У��λ
    /// ������ID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    /// ���ݶ�
    memcpy(tx_data.data, data, data_lenth);
    /// ֡βCRC16������У��
    Append_CRC16_Check_Sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}
