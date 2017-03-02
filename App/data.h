#ifndef  __DATA_H__
#define  __DATA_H__
#include "common.h"

/*============================================
电机控制相关宏定义
==========================================*/

#define MAX_SPEED			1000				//定义最大速度
#define MIN_SPEED			0					//定义最小速度
				
#define LEFT_MOTOR			PTA24				//定义电机控制管脚
#define RIGHT_MOTOR			PTA27

/*============================================
电感数据采集相关宏定义
==========================================*/

#define AMP1     ADC0_SE8						//PTB0
#define AMP2     ADC0_SE9						//PTB1
#define AMP3     ADC0_SE12						//PTB2
#define AMP4     ADC0_SE13						//PTB3

enum Inductance_Direction
{
	LEFT_OUTSIDE = 0,
	LEFT_INSIDE  = 1,
	RIGHT_INSIDE = 2,
	RIGHT_OUTSIDE= 3
};


#define DEFAULT_MAX_VALUE 0
#define DEFAULT_MIN_VALUE 300

/*============================================
方向计算相关宏定义
==========================================*/

#define LEFT_OUTSIDE_WEIGHT  10
#define LEFT_INSIDE_WEIGHT   1
#define RIGHT_INSIDE_WEIGHT  1
#define RIGHT_OUTSIDE_WEIGHT 10

#define DIRECTION_WEIGHT 10

/*============================================
其它宏定义和typedef
==========================================*/

#define RESET  PTC12_OUT						//OLED相关宏定义
#define DC     PTC10_OUT						//OLED相关宏定义
#define D1     PTC14_OUT						//OLED相关宏定义
#define D0     PTC16_OUT						//OLED相关宏定义
#define Bluetooth		UART4					//宏定义Bluetooth®发送端口
#define SECTOR_NUM		(FLASH_SECTOR_NUM-1)	//flash扇区宏定义，尽量用最后面的扇区，确保安全
#define CAR_STOP		PTE8					//定义停车检测管脚
#define CAR_STOP_NUM	8						//定义停车管脚号
#define true			1						//定义逻辑真
#define false			0						//定义逻辑假
typedef char			bool;					//定义bool类型

/*============================================
速度相关数据结构体
==========================================*/

typedef struct speed
{
	int16 Out_Speed;							//最终输出到电机的速度
	int16 PID_Out_Speed;						//PID处理后的速度
	float P;									//pid常量
	float I;									//pid常量
	float D;									//pid常量
	int16 Aim_Speed;							//目标速度
	int16 Turn_Speed;							//转向差速度
	int16 Go_Speed;								//正常前进速度
	int16 Now_Speed;							//正交解码得出的当前速度
	int16 Error_Speed;							//目标速度与当前速度的差值
	float err_next;								//定义上一个偏差值    
	float err_last;								//定义最上前的偏差值
}speed;

/*============================================
电感相关数据结构体
==========================================*/

typedef struct
{
	int16 AD_Value;								//ADC数模转换器采集到的值
	int16 Max_AD_Value;							//ADC数模转换器采集到的最大值
	int16 Min_AD_Value;							//ADC数模转换器采集到的最小值
	int16 Normalized_Value;						//归一化的电感值
	int16 normalization;						//归一化的分母值
	int16 AD_Value_Old[4];						//权重向前滤波算法储存的前几次采集到的值
	char AD_Weight[4];							//权重向前滤波算法权重值
}inductance;

typedef struct direction
{
	int16 err;
}direction;

typedef struct service
{
	bool Debug;
}service;

extern speed Left_Speed;						//声明一个"Speed类"的"对象"，左轮数据
extern speed Right_Speed;						//声明一个"Speed类"的"对象"，右轮数据

extern inductance Road_Data[4];					//声明一个"Inductance类"的"对象"数组，电感信息

extern direction Direction;						//声明一个"Direction类"的"对象"，方向信息

extern service Service;							//声明一个"service类"的"对象"，串口发送等服务信息
#endif  //__DATA_H__