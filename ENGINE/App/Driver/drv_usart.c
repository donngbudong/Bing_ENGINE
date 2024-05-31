#include "drv_usart.h"
#include "Device.h"
#include "system.h"

uint8_t Judge_Buffer[JUDGE_BUFFER_LEN] = {0};
uint8_t Dbus_Buffer[2][SBUS_RX_BUF_NUM];
uint8_t diy_control[128];

extern UART_HandleTypeDef huart1;/*裁判系统*/
extern UART_HandleTypeDef huart2;/*遥控器*/
extern UART_HandleTypeDef huart3;/*图传链路*/
extern UART_HandleTypeDef huart4;/*图传链路*/
extern UART_HandleTypeDef huart6;/*Vofa*/

extern DMA_HandleTypeDef hdma_usart2_rx;


//static void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

/**
  * @brief          UART初始化
  * @param[in]      none
  * @retval         none
  */
void USART_Init(void)
{
	Remote_Control_Init();//Remote_Control
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//Referee_System
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);//DBUS
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);/*图传链路*/
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);/*图传链路*/
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);//VOFA
}

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void Remote_Control_Init(void)
{
	RC_Init(Dbus_Buffer[0],Dbus_Buffer[1],SBUS_RX_BUF_NUM);
}



void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//使能DMA串口接收
	SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
   //失效DMA
	__HAL_DMA_DISABLE(&hdma_usart2_rx);
	while(hdma_usart2_rx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart2_rx);
	}
	hdma_usart2_rx.Instance->PAR = (uint32_t) & (USART2->DR);
    //内存缓冲区1
	hdma_usart2_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
	hdma_usart2_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
	hdma_usart2_rx.Instance->NDTR = dma_buf_num;
   //使能双缓冲区
	SET_BIT(hdma_usart2_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
	__HAL_DMA_ENABLE(&hdma_usart2_rx);
		
}

/**
  * @brief          中断IT
  * @param[in]      huart
  * @retval         none
  */
 uint16_t rx_len = 0;
void UART_IRQHandler_IT(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)//Referee_System
	{
		if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET) 
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			HAL_UART_DMAStop(&huart1);
			HAL_UART_Receive_DMA(&huart1,Judge_Buffer,JUDGE_BUFFER_LEN);
			Referee_Decode(Judge_Buffer);
			memset(Judge_Buffer, 0, 128);//清除接收缓存
		}
	}
	else if(huart==&huart2)//Remote_Control
	{
		if(huart2.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
			__HAL_UART_CLEAR_PEFLAG(&huart2);
    }
		 else if(huart2.Instance->SR & UART_FLAG_IDLE)
		{
			__HAL_UART_CLEAR_PEFLAG(&huart2);

			static uint16_t this_time_rx_len = 0;
			
			if ((huart2.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				/* Current memory buffer used is Memory 0 */
				//失效DMA
				__HAL_DMA_DISABLE(huart2.hdmarx);
        		//获取接收数据长度,长度 = 设定长度 - 剩余长度
				this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
				//重新设定数据长度
				__HAL_DMA_SET_COUNTER(huart2.hdmarx, SBUS_RX_BUF_NUM);
				//设定缓冲区1
				huart2.hdmarx->Instance->CR |= DMA_SxCR_CT;
				//使能DMA
				__HAL_DMA_ENABLE(huart2.hdmarx);
				if(this_time_rx_len == RC_FRAME_LENGTH)
				{
						sbus_to_rc(Dbus_Buffer[0], &RC_Ctrl);
				}
			}
			else
			{
				/* Current memory buffer used is Memory 1 */
				//失效DMA
					__HAL_DMA_DISABLE(huart2.hdmarx);
        		//获取接收数据长度,长度 = 设定长度 - 剩余长度
					this_time_rx_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
				//重新设定数据长度
					__HAL_DMA_SET_COUNTER(huart2.hdmarx, SBUS_RX_BUF_NUM);
				//设定缓冲区0
					huart2.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
				//使能DMA
					__HAL_DMA_ENABLE(huart2.hdmarx);
				if(this_time_rx_len == RC_FRAME_LENGTH)
				{
						//处理遥控器数据
					sbus_to_rc(Dbus_Buffer[1], &RC_Ctrl);
				}
			}
		Remote_time = micros() + 30000;
	 }
	}
	else if(huart==&huart3)//图传
	{
		if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET) 
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);
			HAL_UART_DMAStop(&huart3);
			HAL_UART_Receive_DMA(&huart3,diy_control,128);
			Referee_Decode(diy_control);
			memset(diy_control, 0, 128);//清除接收缓
		}
	}
	else if(huart==&huart4)//Vision
	{
		if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE)!=RESET) 
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart4);
			HAL_UART_DMAStop(&huart4);
			HAL_UART_Receive_DMA(&huart4,diy_control,128);
			Referee_Decode(diy_control);
			memset(diy_control, 0, 128);//清除接收缓
		}
	}
}
