#include "robot_arm.h"
#include "cmsis_os.h"
#include "crc.h"
#include "string.h"
#include "usart.h"
#include "HI229.h"






/***********函数定义区******************/

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);

/*************全局变量区*****************/
uint8_t data[DATA_LENGTH];
DIY_data_t DIY_data;
extern IMU_GET_DATA_t IMU_Get_Data;
Controller_t tx_data; // 自定义控制器发送的数据
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
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// 帧头数据
    tx_data.frame_header.sof = 0xA5;                              // 数据帧起始字节，固定值为 0xA5
    tx_data.frame_header.data_length = data_lenth;                // 数据帧中数据段的长度
    tx_data.frame_header.seq = seq++;                             // 包序号
    Append_CRC8_Check_Sum((uint8_t *)(&tx_data.frame_header), 5); // 添加帧头 CRC8 校验位
    /// 命令码ID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    /// 数据段
    memcpy(tx_data.data, data, data_lenth);
    /// 帧尾CRC16，整包校验
    Append_CRC16_Check_Sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}
