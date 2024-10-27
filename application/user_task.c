//������˵������������û������ϻ���ͼ�κ���ʾ��Ϣ��������ϵͳ״̬��̬������Щͼ�κ���Ϣ�ģ�Ҳ������Ϸ���ڣ�
#include "User_Task.h"
#include "main.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_Receive.h"//����Ħ���ַ���ջ���ʵת��
#include "FreeRTOS.h"//* pvParameters
#include "task.h"//vTaskDelay
#include "chassis_power_control.h"//���ʿ���
#include "shoot.h"
#include "chassis_task.h"
#define PI 3.1415936

extern power_heat_data_t power_heat_data;//��������
extern shoot_control_t shoot_control;          //�������
extern chassis_move_t chassis_move;						//�����˶�����

Graph_Data G1,G2,G3,G4,G5,G6,G7,G8,G9;
Graph_Data GImpactLeftLine,GImpactRightLine,GImpactFrontLine;
Float_Data FL_POWER;
String_Data CH_SHOOT;
//String_Data CH_FLRB;
//String_Data CH_TEST;
//String_Data CH_POWER;
String_Data CH_SPIN;
char shoot_arr[5]="shoot";
char spin_arr[4]="spin";
//char flrb_arr[4]="FRBL";
//char test_arr[]="BeiJiXiong2024";
//char power_arr[4];
void UserTask(void const * pvParameters)
{
//		fp32 POWER=power_heat_data.chassis_power;
//		memcpy(power_arr,&POWER,sizeof(power_arr));
	
		memset(&G1,0,sizeof(G1));//���Ĵ���
		memset(&G2,0,sizeof(G2));//�ϻ�����
		memset(&G3,0,sizeof(G3));//����ˮƽ��
		memset(&G4,0,sizeof(G4));//ǹ��������
		memset(&G5,0,sizeof(G5));//�»�����
		memset(&G6,0,sizeof(G6));//Զ���������
		memset(&G7,0,sizeof(G7));//Ħ����״̬
		memset(&G8,0,sizeof(G8));//С����״̬
		memset(&G9,0,sizeof(G9));//
		memset(&CH_SHOOT,0,sizeof(CH_SHOOT));//Ħ���ֱ�ʶ
		memset(&CH_SPIN,0,sizeof(CH_SPIN));//С���� ��ʶ
//		memset(&CH_TEST,0,sizeof(CH_TEST));//����һ������
		memset(&GImpactLeftLine,0,sizeof(GImpactLeftLine));//
		memset(&GImpactRightLine,0,sizeof(GImpactRightLine));//
		memset(&GImpactFrontLine,0,sizeof(GImpactFrontLine));//

        Line_Draw(&G1,"091",UI_Graph_ADD,9,UI_Color_Purplish_red,1,960,330,960,620);
		Line_Draw(&G2,"092",UI_Graph_ADD,9,UI_Color_Purplish_red,1,880,580,1040,580);
		Line_Draw(&G3,"093",UI_Graph_ADD,9,UI_Color_Purplish_red,1,800,540,1120,540);
		Line_Draw(&G4,"094",UI_Graph_ADD,9,UI_Color_Purplish_red,1,880,500,1040,500);
		Line_Draw(&G5,"095",UI_Graph_ADD,9,UI_Color_Purplish_red,1,900,420,1020,420);
	    Line_Draw(&G6,"096",UI_Graph_ADD,9,UI_Color_Purplish_red,1,920,370,1000,370);
	    Circle_Draw(&G7,"097",UI_Graph_ADD,9,UI_Color_Yellow,15,230,770,15);
	    Char_Draw(&CH_SHOOT,"087",UI_Graph_ADD,8 ,UI_Color_Yellow,24,5,4,80,780,&shoot_arr[0]);
			UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7); //����ͼ�� 
			Char_ReFresh(CH_SHOOT);

		
        
//				Char_Draw(&CH_TEST,"088",UI_Graph_ADD,4,UI_Color_Pink,24,sizeof(test_arr),4,80,300,&test_arr[0]);
//				Char_ReFresh(CH_TEST);
//				Char_Draw(&CH_POWER,"089",UI_Graph_ADD,4,UI_Color_Pink,24,sizeof(power_arr),4,80,500,&power_arr[0]);
//				Char_ReFresh(CH_POWER);

	  	Circle_Draw(&G8,"098",UI_Graph_ADD,9,UI_Color_Yellow,15,230,670,15);
			Char_Draw(&CH_SPIN,"088",UI_Graph_ADD,8 ,UI_Color_Yellow,24,4,4,80,680,&spin_arr[0]);
			Char_ReFresh(CH_SPIN);
//			Line_Draw(&GImpactLeftLine,"081",UI_Graph_ADD,8,UI_Color_Yellow,3,500,100,700,300);
//			Line_Draw(&GImpactRightLine,"082",UI_Graph_ADD,8,UI_Color_Yellow,3,1250,300,1450,100);
//			Line_Draw(&GImpactFrontLine,"083",UI_Graph_ADD,8,UI_Color_Yellow,3,700,300,1250,300);
//			UI_ReFresh(5,GImpactLeftLine,GImpactRightLine,GImpactFrontLine,G8,G9);
				
  while(1)
	{
		
//		Float_ReFresh(FL_POWER);//���Ƹ�����	
//		//UI_ReFresh(1,FL_POWER);
//		Float_Draw(&FL_POWER,"088",UI_Graph_ADD,8,UI_Color_Black,5,4,50,320,370,0.10086);	
//		vTaskDelay(100);
		
		//��Ҫˢ�µ�ui��Ҫ��ѭ�����ض���һ�� ��Ȼû��changeʵ��
//		Circle_Draw(&G7,"097",UI_Graph_ADD,9,UI_Color_Yellow,15,230,770,15);
		
		//����Ħ���ַ���UI,��ʼ����ɫ�������Ϻ�ɫ��������ɫ
		//if(Shoot_Right_Date.V&&Shoot_Right_Date.V)
		if(shoot_control.shoot_mode >= 1)//SHOOT_READY_FRIC)
			Circle_Draw(&G7,"007",UI_Graph_ADD,9,UI_Color_Green,15,230,770,15);//δ֪bug Change���ͼˢû
		else
			;//Circle_Draw(&G7,"007",UI_Graph_ADD,9,UI_Color_Purplish_red,15,230,770,15);
		
		UI_ReFresh(7,G1,G2,G3,G4,G5,G6,G7); //����ͼ��
		Circle_Draw(&G7,"007",UI_Graph_Del,9,UI_Color_Green,15,230,770,15);//ˢ��ɾ�� ����Change
		vTaskDelay(100);
		
		Char_ReFresh(CH_SHOOT);
		vTaskDelay(100);		
		
		Char_ReFresh(CH_SPIN);
		vTaskDelay(100);
		
		if(chassis_move.chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW_SPIN)
			Circle_Draw(&G8,"008",UI_Graph_ADD,9,UI_Color_Green,15,230,670,15);
		else
			;
		
		UI_ReFresh(1,G8);
//		UI_ReFresh(5,GImpactLeftLine,GImpactRightLine,GImpactFrontLine,G8,G9);																																																																																																																																																																																																																																																																																																																																						
		Circle_Draw(&G8,"008",UI_Graph_Del,9,UI_Color_Green,15,230,670,15);//ˢ��ɾ�� ����Change
//		memcpy(power_arr,&POWER,sizeof(power_arr));
//		Char_Draw(&CH_POWER,"089",UI_Graph_Change,4,UI_Color_Pink,24,sizeof(power_arr),4,80,500,&power_arr[0]);
//		Char_ReFresh(CH_POWER);

		
		
		//Ƶ�ʿ���10hz
		vTaskDelay(100);
		//osDelay(100);
	}		
}
