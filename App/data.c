#include "data.h"
speed Left_Speed;						//声明一个"speed类"的"对象"，左轮数据
speed Right_Speed;						//声明一个"speed类"的"对象"，右轮数据

inductance Road_Data[AMP_MAX];			//声明一个"inductance类"的"对象"数组，电感信息

direction Direction;					//声明一个"direction类"的"对象"，方向信息

fuzzy_direction Fuzzy_Direction;		//声明一个"Fuzzy_Direction类"的"对象"，模糊控制方向信息

service Service;						//声明一个"service类"的"对象"，串口发送等服务信息