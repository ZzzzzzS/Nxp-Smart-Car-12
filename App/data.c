#include "data.h"
speed Speed;											//声明一个"speed类"的"对象"，全局储存速度信息

inductance Road_Data[AMP_MAX];			//声明一个"inductance类"的"对象"数组，电感信息

direction Direction;									//声明一个"direction类"的"对象"，方向信息

service Service;											//声明一个"service类"的"对象"，串口发送等服务信息

int16 TempSpeed;