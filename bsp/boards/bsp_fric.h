#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1400  //1400        //Ħ���ָ���ģʽ����ٶ�
#define FRIC_DOWN 2000	//1320   //Ħ�������ת��
#define FRIC_OFF 1000  //1000        //Ħ������С�ٶȣ�Ħ�����ٶ�����Ϊ1100������1100��ת

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
