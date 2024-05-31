#include "referee_UI.h"
#include "crc.h"
#define Referee_UART huart1
extern UART_HandleTypeDef huart1;

/*==============================================================================
              ##### UI����ͼ�λ��ƺ��� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ֱ�� UI_Draw_Line
      (+) ���ƾ��� UI_Draw_Rectangle
      (+) ������Բ UI_Draw_Circle
      (+) ������Բ UI_Draw_Ellipse
      (+) ����Բ�� UI_Draw_Arc
      (+) ����С�� UI_Draw_Float
      (+) �������� UI_Draw_Int
      (+) �����ַ� UI_Draw_String
*/
void UI_Draw_Line(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
									uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									uint16_t               Width,        //�߿�
									uint16_t               StartX,       //��ʼ����X
									uint16_t               StartY,       //��ʼ����Y
									uint16_t               EndX,         //��ֹ����X
									uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Line;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                     char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									     uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
							     	   uint16_t               Width,        //�߿�
							     		 uint16_t               StartX,       //��ʼ����X
							     		 uint16_t               StartY,       //��ʼ����Y
							     		 uint16_t               EndX,         //��ֹ����X
							     		 uint16_t               EndY)         //��ֹ����Y
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Rectangle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->end_x           = EndX;
	Graph->end_y           = EndY;
}

void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                  char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									  uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									  uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t               Width,        //�߿�
										uint16_t               CenterX,      //Բ������X
							      uint16_t               CenterY,      //Բ������Y
										uint16_t               Radius)       //�뾶
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Circle;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->radius          = Radius;
}

void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                   char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
									   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
									   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     	 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										 uint16_t               Width,        //�߿�
										 uint16_t               CenterX,      //Բ������X
							       uint16_t               CenterY,      //Բ������Y
										 uint16_t               XHalfAxis,    //X���᳤
										 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Ellipse;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               StartAngle,   //��ʼ�Ƕ� [0,360]
								 uint16_t               EndAngle,     //��ֹ�Ƕ� [0,360]
								 uint16_t               Width,        //�߿�
								 uint16_t               CenterX,      //Բ������X
							   uint16_t               CenterY,      //Բ������Y
								 uint16_t               XHalfAxis,    //X���᳤
								 uint16_t               YHalfAxis)    //Y���᳤
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Arc;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = StartAngle;
	Graph->end_angle       = EndAngle;
	Graph->width           = Width;
	Graph->start_x         = CenterX;
	Graph->start_y         = CenterY;
	Graph->end_x           = XHalfAxis;
	Graph->end_y           = YHalfAxis;
}

void UI_Draw_Float(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	                 char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							     uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
									 uint16_t               NumberSize,   //�����С
									 uint16_t               Significant,  //��Чλ��
									 uint16_t               Width,        //�߿�
							     uint16_t               StartX,       //��ʼ����X
							     uint16_t               StartY,       //��ʼ����Y
									 float                  FloatData)    //��������
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Float;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->end_angle       = Significant;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	int32_t IntData = FloatData * 1000;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_Int(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
	               char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
							   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
								 uint16_t               NumberSize,   //�����С
								 uint16_t               Width,        //�߿�
							   uint16_t               StartX,       //��ʼ����X
							   uint16_t               StartY,       //��ʼ����Y
								 int32_t                IntData)      //��������
{
	Graph->graphic_name[0] = GraphName[0];
	Graph->graphic_name[1] = GraphName[1];
	Graph->graphic_name[2] = GraphName[2];
	Graph->operate_tpye    = GraphOperate;
	Graph->graphic_tpye    = UI_Graph_Int;
	Graph->layer           = Layer;
	Graph->color           = Color;
	Graph->start_angle     = NumberSize;
	Graph->width           = Width;
	Graph->start_x         = StartX;
	Graph->start_y         = StartY;
	Graph->radius          = (IntData & 0x000003ff) >>  0;
	Graph->end_x           = (IntData & 0x001ffc00) >> 10;
	Graph->end_y           = (IntData & 0xffe00000) >> 21;
}

void UI_Draw_String(string_data_struct_t *String,        //UIͼ�����ݽṹ��ָ��
	                  char                  StringName[3], //ͼ���� ��Ϊ�ͻ��˵�����
							      uint8_t               StringOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
								    uint8_t               Layer,         //UIͼ��ͼ�� [0,9]
							      uint8_t               Color,         //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
										uint16_t              CharSize,      //�����С
									  uint16_t              StringLength,  //�ַ�������
									  uint16_t              Width,         //�߿�
							      uint16_t              StartX,        //��ʼ����X
							      uint16_t              StartY,        //��ʼ����Y
										char                 *StringData)    //�ַ�������
{
	String->string_name[0] = StringName[0];
	String->string_name[1] = StringName[1];
	String->string_name[2] = StringName[2];
	String->operate_tpye   = StringOperate;
	String->graphic_tpye   = UI_Graph_String;
	String->layer          = Layer;
	String->color          = Color;
	String->start_angle    = CharSize;
	String->end_angle      = StringLength;
	String->width          = Width;
	String->start_x        = StartX;
	String->start_y        = StartY;
	for(int i = 0; i < StringLength; i ++) String->stringdata[i] = *StringData ++;
}

/*==============================================================================
              ##### UI����ͼ�����ͺ��� #####
  ==============================================================================
    [..]  �ò����ṩ���º���:
		  (+) ����ͼ�� UI_PushUp_Graphs
			(+) �����ַ� UI_PushUp_String
			(+) ɾ��ͼ�� UI_PushUp_Delete
*/
void UI_PushUp_Graphs(uint8_t Counter /* 1,2,5,7 */, void *Graphs /* ��Counter��һ�µ�UI_Graphx�ṹ��ͷָ�� */, uint8_t RobotID)
{
	UI_Graph1_t *Graph = (UI_Graph1_t *)Graphs; //����ֻ��һ������ͼ��
	
	/* ��� frame_header */
	Graph->Referee_Transmit_Header.SOF  = HEADER_SOF;
	     if(Counter == 1) Graph->Referee_Transmit_Header.data_length = 6 + 1 * 15;
	else if(Counter == 2) Graph->Referee_Transmit_Header.data_length = 6 + 2 * 15;
	else if(Counter == 5) Graph->Referee_Transmit_Header.data_length = 6 + 5 * 15;
	else if(Counter == 7) Graph->Referee_Transmit_Header.data_length = 6 + 7 * 15;
	Graph->Referee_Transmit_Header.seq  = Graph->Referee_Transmit_Header.seq + 1;
	Graph->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Graph->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Graph->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	     if(Counter == 1) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw1;
	else if(Counter == 2) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw2;
	else if(Counter == 5) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw5;
	else if(Counter == 7) Graph->Interactive_Header.data_cmd_id = UI_DataID_Draw7;
	Graph->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Graph->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	     if(Counter == 1)
	{
		UI_Graph1_t *Graph1 = (UI_Graph1_t *)Graphs;
		Graph1->CRC16 = CRC16_Calculate((uint8_t *)Graph1, sizeof(UI_Graph1_t) - 2);
	}
	else if(Counter == 2)
	{
		UI_Graph2_t *Graph2 = (UI_Graph2_t *)Graphs;
		Graph2->CRC16 = CRC16_Calculate((uint8_t *)Graph2, sizeof(UI_Graph2_t) - 2);
	}
	else if(Counter == 5)
	{
		UI_Graph5_t *Graph5 = (UI_Graph5_t *)Graphs;
		Graph5->CRC16 = CRC16_Calculate((uint8_t *)Graph5, sizeof(UI_Graph5_t) - 2);
	}
	else if(Counter == 7)
	{
		UI_Graph7_t *Graph7 = (UI_Graph7_t *)Graphs;
		Graph7->CRC16 = CRC16_Calculate((uint8_t *)Graph7, sizeof(UI_Graph7_t) - 2);
	}
	
	/* ʹ�ô���PushUp������ϵͳ */
	     if(Counter == 1) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph1_t));
	else if(Counter == 2) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph2_t));
	else if(Counter == 5) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph5_t));
	else if(Counter == 7) HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Graph, sizeof(UI_Graph7_t));
}

void UI_PushUp_String(UI_String_t *String, uint8_t RobotID)
{
	/* ��� frame_header */
	String->Referee_Transmit_Header.SOF  = HEADER_SOF;
	String->Referee_Transmit_Header.data_length = 6 + 45;
	String->Referee_Transmit_Header.seq  = String->Referee_Transmit_Header.seq + 1;
	String->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&String->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	String->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	String->Interactive_Header.data_cmd_id = UI_DataID_DrawChar;
	String->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	String->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	String->CRC16 = CRC16_Calculate((uint8_t *)String, sizeof(UI_String_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)String, sizeof(UI_String_t));
}

void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID)
{
	/* ��� frame_header */
	Delete->Referee_Transmit_Header.SOF  = HEADER_SOF;
	Delete->Referee_Transmit_Header.data_length = 6 + 2;
	Delete->Referee_Transmit_Header.seq  = Delete->Referee_Transmit_Header.seq + 1;
	Delete->Referee_Transmit_Header.CRC8 = CRC08_Calculate((uint8_t *)(&Delete->Referee_Transmit_Header), 4);
	
	/* ��� cmd_id */
	Delete->CMD_ID = STUDENT_INTERACTIVE_DATA_CMD_ID;
	
	/* ��� student_interactive_header */
	Delete->Interactive_Header.data_cmd_id = UI_DataID_Delete;
	Delete->Interactive_Header.sender_ID   = RobotID ;      //��ǰ������ID
	Delete->Interactive_Header.receiver_ID = RobotID + 256; //��Ӧ������ID
	
	/* ��� frame_tail ��CRC16 */
	Delete->CRC16 = CRC16_Calculate((uint8_t *)Delete, sizeof(UI_Delete_t) - 2);
	
	/* ʹ�ô���PushUp������ϵͳ */
	HAL_UART_Transmit_DMA(&Referee_UART, (uint8_t *)Delete, sizeof(UI_Delete_t));
}


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#define Robot_ID_Current 104

/* ��̬UI���ݱ��� */
uint8_t R_UI = 0;    //Ħ�����Ƿ���
/* �����߸߶ȱ��� */
uint16_t y01 = 680;
uint16_t y02 = 280;
uint16_t y03 = 280;
uint16_t y04 = 230;
/* ����UIר�ýṹ�� */
UI_Graph1_t UI_Graph1;
UI_Graph2_t UI_Graph2;
UI_Graph5_t UI_Graph5;
UI_Graph7_t UI_Graph7;
UI_String_t UI_String;
UI_Delete_t UI_Delete;
void JUDGE_TASK(void const * argument)
{
	/* ��̬UI���Ʊ��� */
	uint16_t UI_PushUp_Counter = 261;
	
	/* ����ϵͳ��ʼ�� */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		/* ��������ϵͳ���� */
		vTaskDelay(10);
		
		/* UI���� */
		UI_PushUp_Counter++;
		if(UI_PushUp_Counter % 301 == 0) //��̬UIԤ���� ������1
		{
			
			UI_Draw_Line(&UI_Graph5.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  720,  y01,  1200,   y01);	//�������ϱ�
			UI_Draw_Line(&UI_Graph5.Graphic[2], "002", UI_Graph_Add, 0, UI_Color_Green, 1, 1200,  y01, 1200,   y02); 	//�������ұ�
			UI_Draw_Line(&UI_Graph5.Graphic[3], "003", UI_Graph_Add, 0, UI_Color_Green, 1,  720,  y01, 720,   y02); 	//���������
			UI_Draw_Line(&UI_Graph5.Graphic[4], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  720,   y02,  1200,  y02); //�������±�
			UI_Draw_Line(&UI_Graph5.Graphic[5], "005", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //�ڶ������ĵ�
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 311 == 0) //��̬UIԤ���� ������2
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //������������
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 321 == 0) //��̬UIԤ���� С����Ԥ����
		{
			UI_Draw_Line(&UI_Graph5.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph5.Graphic[4], "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 331 == 0) //��̬UIԤ���� ͼ��
		{
			UI_Draw_Float (&UI_Graph2.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith��Ƕ�
			UI_Draw_Line  (&UI_Graph2.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Orange, 20, 1829, 330, 1870, 334);      //��������
			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 341 == 0) //��̬UIԤ���� �ַ���1
		{
			UI_Draw_String(&UI_String.String,     "203", UI_Graph_Add, 2, UI_Color_Black,  22, 8, 3,  400, 632, "R OFF"); //Ħ�����Ƿ���
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			continue;
		}
//		if(UI_PushUp_Counter % 21 == 0) //��̬UI���� �ַ���1
//		{
//			if(UI_fric_is_on == 1) 
//			{
//				if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==0)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre  NO");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==0)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre  NO");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==0)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre  NO");
//				}		
//				else if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==1)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre YES");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==1)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre YES");
//				}
//				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==1)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre YES");
//				}		
//				
//				else if(autoaim_mode==0x03)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nXFu\n        \n       ");
//				}
//				else if(autoaim_mode==0x04)
//				{
//					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nDFu\n        \n       ");
//				}
//			}
//			if(UI_fric_is_on == 0) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Black, 22, 8+4+9+8, 3,  100, 700, "Fric OFF\n   \n        \n       ");
//			UI_PushUp_String(&UI_String, Robot_ID_Current);
//			continue;
//		}
//		if(UI_PushUp_Counter % 10 == 0)  //��̬UI���� ͼ��
//		{
//			/* Pitch�ᵱǰ�Ƕ� */
//			UI_Draw_Float(&UI_Graph2.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
//			
//			/* ������������ */
//			UI_Capacitance = Max(UI_Capacitance, 30);
//			Capacitance_X  = 1870.0f - 4.1f * UI_Capacitance;
//			if(50 < UI_Capacitance && UI_Capacitance <= 100) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Green , 20, Capacitance_X, 334, 1870, 334);
//			if(35 < UI_Capacitance && UI_Capacitance <=  50) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Yellow, 20, Capacitance_X, 334, 1870, 334);
//			if(0  < UI_Capacitance && UI_Capacitance <=  35) UI_Draw_Line(&UI_Graph2.Graphic[1], "202", UI_Graph_Change, 2, UI_Color_Orange, 20, Capacitance_X, 334, 1870, 334);
//			

//			UI_PushUp_Graphs(2, &UI_Graph2, Robot_ID_Current);
//			continue;
//		}
	}
}

