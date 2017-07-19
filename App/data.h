#ifndef  __DATA_H__
#define  __DATA_H__
#include "common.h"


/*============================================
PID相关数据结构体
==========================================*/
typedef enum PID_Error						//PID误差枚举
{
	Now_Error,
	last_Error,
	lastest_Error
}PID_Error;

typedef struct pidbasespeed
{
	float P;												//pid常量
	float I;												//pid常量
	float D;												//pid常量;
	int16 Aim_Speed;								//pid目标速度
	int16 Now_Speed;								//pid当前速度
	int16 Error_Speed[3];							//pid误差速度
	int16 Intergate_Speed;						//定义积分速度
	int16 IncrementSpeed;						//速度增量
	int16 PID_Out_Speed;						//最终输出速度
}pidbasespeed;

/*============================================
电机控制相关定义
==========================================*/
#define Stable_Times		5					//定义电机滤波速度
#define MAX_SPEED			95					//定义最大速度
#define MIN_SPEED			-50				//定义最小速度

#define MOTOR_FTM			FTM0
#define RIGHT_PWM_BACK	FTM_CH3
#define RIGHT_PWM			FTM_CH4
#define LEFT_PWM				FTM_CH1
#define LEFT_PWM_BACK		FTM_CH2

#define MOTOR_HZ    20*1000				//定义电机工作频率

typedef struct
{
	int16 Out_Speed;								//最终输出到电机的速度
	int16 Turn_Speed;								//转向差速度
	int16 Now_Speed;								//正交解码得出的当前速度
	int16 Speed_Old[Stable_Times];			//电机功率滤波
}wheel;

typedef struct
{
	wheel Left;
	wheel Right;
	pidbasespeed Base;
}speed;

/*============================================
电感数据采集相关宏定义
==========================================*/

#define AD1			ADC0_SE8					//PTB0
#define AD2			ADC0_SE9					//PTB1
#define AD3			ADC0_SE12				//PTB2
#define AD4			ADC0_SE13				// PTB3		


#define AMP_MAX	4									//定义最大ADC端口数

typedef enum Inductance_Position				//枚举定义电感位置
{
	LEFT,
    FRONT,
    MIDDLE,
	RIGHT
}Inductance_Position;

typedef struct
{
	int16 AD_Value;									//ADC数模转换器采集到的值,8bit
	int16 AD_Value_fixed;						//滤波后的值
	int16 AD_Value_Old[10];						//权重向前滤波算法储存的前几次采集到的值
}inductance;

/*============================================
方向计算相关定义
==========================================*/

#define MAX_FUZZY_RULE		6						//模糊论域大小

#define Lk				0.7507
#define Lb			-0.2793
#define Rk			-0.7098
#define Rb			-0.2622

typedef struct direction							//差比和法方向控制
{
	int16 err;											//偏差误差
	int16 sum[3];										//差比和相关定义
	pidbasespeed PIDbase;						//PID计算类型
}direction;

/*============================================
OLED显示相关定义
==========================================*/
#define RESET	 PTC14_OUT						//OLED相关宏定义
#define DC		 PTC16_OUT						//OLED相关宏定义
#define D1		 PTC12_OUT						//OLED相关宏定义
#define D0		 PTC10_OUT						//OLED相关宏定义

#define Position(OLED_Line)				0,(OLED_Line)	//坐标定义

typedef enum OLED_Line											//定义OLED显示位置
{
	Line1 = 0,
	Line2 = 2,
	Line3 = 4,
	Line4 = 6,

	MAX_Line
}OLED_Line;

typedef enum Debug_Interface									//定义调试模式OLED界面编号
{
	Inductance_Interface = 1,
	Speed_Interface,
	Direction_Interface,

	MAX_Interface
}Debug_Interface;

typedef struct OLED
{
	char OLED_Interface;
	char OLED_Renew;
}OLED;

/*============================================
调试模式定义
==========================================*/

#define   hasData(x)	strstr(Service.BlueToothBase.ReceiveArea,(x))!=NULL

typedef enum									//定义系统错误编号
{
	Motor_Stop,
	Taget_Lost,
	Car_Stop,
	No_Mode,
	user_Stop,
	hardfault,

	MAX_error
}Error_Num;

typedef enum Run_Mode											//定义调试模式编号
{
	inductance_Mode = 1,
	FastMode,
	SlowMode,

	Max_Mode
}Run_Mode;

typedef struct UserInformation
{
	unsigned char speed;
	float P;
	float I;
	float D;
}UserInformation;

typedef struct BlueTooth
{
	char AllowedSendData;					//发送允许位
	char AllowedReceiveData;				//接收允许位
	unsigned char ReceiveArea[20];		//接收区临时缓存
	UserInformation Information;		//储存用户数据
	union convert_data						//浮点数转换共用体
	{
		float Float_Base;
		unsigned char Receive_Base[4];
	};
}BlueTooth;

typedef struct Inductance
{
	char InductanceLost;
}Inductance;

typedef struct Motor
{
	char AllowRun;
	char GetSpeedAbs;
}Motor;

typedef struct service
{
	char RunMode;									//运行模式
	OLED OLEDbase;								//OLED相关配置
	BlueTooth BlueToothBase;					//Bluetooth®相关配置
	Inductance InductanceBase;				//电感丢线相关信息
	Motor MotorBase;								//电机转动相关信息
}service;

/*============================================
其它宏定义和typedef
==========================================*/
#define true		1
#define false		0
#define ture		1
#define flase		0
typedef char		bool;

#define Key5  PTC13									//按键管脚定义
#define Key2   PTC11									//按键管脚定义
#define Key3   PTC9									//按键管脚定义,这个按钮有问题
#define Key4   PTC7									//按键管脚定义
#define Key1    PTA25									//按键管脚定义

#define Bluetooth		UART4						//宏定义Bluetooth®发送端口
#define Bluetooth_Band	9600				//宏定义Bluetooth®发送波特率

#define REED		PTC5						//定义停车检测管脚

typedef unsigned char		counter;			//定义累加器类型
typedef unsigned char		error;				//定义错误类型

/*============================================
"对象""实例化"
==========================================*/

extern speed Speed;											//声明一个"speed类"的"对象"，全局储存速度信息

extern inductance Road_Data[AMP_MAX];			//声明一个"Inductance类"的"对象"数组，电感信息,修改时记得修改蓝牙发送

extern direction Direction;									//声明一个"Direction类"的"对象"，方向信息

extern service Service;										//声明一个"service类"的"对象"，串口发送等服务信息

extern int16 TempSpeed;

#endif  //__DATA_H__