#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"

#define FRIC_UP 1400  //1400        //摩擦轮高速模式最大速度
#define FRIC_DOWN 2000	//1320   //摩擦轮最大转速
#define FRIC_OFF 1000  //1000        //摩擦轮最小速度，摩擦轮速度下限为1100，低于1100不转

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
