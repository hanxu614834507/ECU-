#ifndef _COMMAND_H_
#define _COMMAND_H_

#include "stm32f4xx.h"

/****************************************
上装指令ID定义
*****************************************/
#define GEAR_MODE_CMD_ID 	0xA1	//整车状态模式切换模式ID
#define STEER_CMD_ID		0xA2	//转向指令ID
#define DRIVE_CMD_ID		0xA3	//驱动指令ID
#define BRAKE_CMD_ID		0xA4	//刹车指令ID
#define	PARK_CMD_ID			0xA5	//驻车指令ID

#define	VCU_FEEDBACK_ID		0xC1	//VCU状态反馈数据ID
#define	BMS_FEEDBACK_ID		0xC2	//BMS状态反馈数据ID
#define	ENABLE_FEEDBACK_ID	0xC3	//ENABLE状态反馈ID
#define	ERR_FEEDBACK_ID		0xE1	//故障码状态反馈ID

#define BRAKE_ID			0x41	//刹车电机ID
#define STEERING_ID			0x42	//转向电机ID
#define DRIVE_ID			0x44	//驱动电机ID
#define	BMS_ID				0x43	//电池BMSID

#define BRAKE_FdB_ID		0x81	//刹车电机反馈数据ID
#define STEERING_FdB_ID		0x82	//转向电机反馈数据ID
#define DRIVE_FdB_ID		0x84	//驱动电机反馈数据ID
#define	BMS_FdB_ID			0x83	//电池BMS电机反馈数据ID

#define CMD_ENABLE 	1
#define CMD_DISABLE 0

#define PARKING		1				//驻车
#define NULL		2				//空挡
#define	FORWARD		3				//前进
#define	BACK		4				//后退
#define INVALID		5				//无效

#define LOW_SPEED_GEAR 		1		//低速档
#define NORMAL_SPEED_GEAR 	2		//正常速度档
#define HIGH_SPEED_GEAR 	3		//高速档

#define MOTOR_STOP_VALUE	120		//电机停止时的PWM值
#define MOTOR_MAX_SPEED_VALUE	300	//电机最高速度时的PWM值

#define BRAKE_PIN 			GPIOE , GPIO_Pin_10		//刹车控制
#define BACK_PIN 			GPIOD , GPIO_Pin_10		//倒车控制
#define LOW_SPEED_PIN 		GPIOD , GPIO_Pin_11		//低速控制
#define HIGH_SPEED_PIN 		GPIOA , GPIO_Pin_12		//高速控制

#define BRAKE_OFF 			GPIO_ResetBits(GPIOE , GPIO_Pin_10)	//刹车失能
#define BACK_OFF 			GPIO_ResetBits(GPIOD , GPIO_Pin_10)	//后退失能
#define LOW_SPEED_OFF 		GPIO_ResetBits(GPIOD , GPIO_Pin_11)	//低速失能
#define HIGH_SPEED_OFF 		GPIO_ResetBits(GPIOA , GPIO_Pin_12)	//高速失能

#define BRAKE_ON 			GPIO_SetBits(GPIOE , GPIO_Pin_10)	//刹车使能
#define BACK_ON 			GPIO_SetBits(GPIOD , GPIO_Pin_10)	//后退使能
#define LOW_SPEED_ON 		GPIO_SetBits(GPIOD , GPIO_Pin_11)	//低速使能
#define HIGH_SPEED_ON 		GPIO_SetBits(GPIOA , GPIO_Pin_12)	//高速使能

/****************************************
ECU接收上装指令代码定义
*****************************************/
//上装发送的整车状态命令
typedef struct GearModeCmd
{
	uint8_t Gear_Enb;			//挡位使能信号
	uint8_t Gear_Shift_Req;		//挡位切换信号
	uint8_t IPC_ModeCtrl;		//IPC下发模式信号
	uint8_t IPC_Stop_Eme;		//上装急停指令
}GearShiftCmdType;
//上装发送的转向命令
typedef struct SteeringCmd
{
	uint8_t Steering_Enb;		//转向器使能信号
	uint8_t Steering_Pos_Req_L;	//转向角度值
	uint8_t Steering_Pos_Req_H;	//转向角度值
	uint8_t Reserved;			//保留位
}SteeringControlCmdType;
//上装发送的驱动电机命令
typedef struct DriveCmd
{
	uint8_t Drive_Enb;			//驱动电机使能信号
	uint8_t Drive_Tq_Req_L;		//驱动电机力矩值
	uint8_t Drive_Tq_Req_H;		//驱动电机力矩值
	uint8_t Reserved;			//保留位
}DriveControlCmdType;
//上装发送的刹车命令
typedef struct BrakeCmd
{
	uint8_t Brake_Enb;			//刹车使能信号
	uint8_t Brake_Tq_Req_L;		//刹车电机力矩值
	uint8_t Brake_Tq_Req_H;		//刹车电机力矩值
	uint8_t Reserved;			//保留位
}BrakeControlCmdType;
//上装发送的驻车命令
typedef struct ParkingCmd
{
	uint8_t Parking_Enb;		//驻车使能信号
	uint8_t Parking_Cmd;		//驻车请求值
	uint8_t Reserved;			//保留位
}ParkingCmdType;

/****************************************
ECU反馈给上装的数据代码定义
*****************************************/
//反馈给上装的整车状态数据
typedef struct VCUData
{
	uint8_t Speed_L;			//速度值
	uint8_t Speed_H;			//速度值
	uint8_t Steering_Ang_L;		//转向角度
	uint8_t Steering_Ang_H;		//转向角度
	uint8_t Gear_Pos;			//挡位信号
	uint8_t Brake_Tq_L;			//刹车力矩
	uint8_t Brake_Tq_H;			//刹车力矩
	uint8_t CRC_Value;			//校验值
}VCUFdBDataType;
//反馈给上装的电池数据
typedef struct BMSData
{
	uint8_t SOC;				//电量百分值
	uint8_t Bat_Vol_L;			//电池电压值
	uint8_t Bat_Vol_H;			//电池电压值
	uint8_t Bat_Discharge_Cur;	//电池放电电流
	uint8_t Reserved1;			//保留位
	uint8_t Reserved2;			//保留位
	uint8_t Reserved3;			//保留位
	uint8_t CRC_Value;			//校验值
}BMSFdBDataType;
//反馈给上装，ECU对命令请求的执行结果
typedef struct EnableState
{
	uint8_t Gear_Enable;		//挡位使能
	uint8_t Steering_Enable;	//转向使能
	uint8_t Drive_Enable;		//驱动使能
	uint8_t Brake_Enable;		//制动使能
	uint8_t Parking_Enable;		//驻车使能
	uint8_t RC_Takeover_Flag;	//接管标志位
	uint8_t Reserved;			//保留位
	uint8_t CRC_Value;			//校验值
}EnableFdBDataType;
//反馈给上装的错误码
typedef struct ErrState
{
	uint8_t Error_code;			//故障码
	uint8_t Code_Num;			//具体错误码
	uint8_t Reserved1;			//保留位
	uint8_t Reserved2;			//保留位
	uint8_t Reserved3;			//保留位
	uint8_t Reserved4;			//保留位
	uint8_t Reserved5;			//保留位
	uint8_t CRC_Value;			//校验值
}ErrCodeFdBType;

/****************************************
下发指令代码定义
*****************************************/
//下发给各个模块的控制指令
typedef struct SendControlCmd
{
	uint8_t GearPos;		//挡位
	uint8_t Value_L;		//控制变化的值
	uint8_t Value_H;		//控制变化的值
	uint8_t SpeedFlag;		//速度挡位
}ControlCmdType;

/****************************************
底盘各部分上传给ECU的状态信息代码定义
*****************************************/
//各个电机上传给ECU的状态信息
typedef struct MotorStateInformation
{
	uint8_t EnableState;	//接收到的ECU指令执行的状态
	uint8_t	ModeState;		//所处的状态（挡位信号）
	uint8_t Value_L;		//执行的值
	uint8_t Value_H;		//执行的值
	uint8_t	ErrorCodeNum;	//出现的故障
	uint8_t Reserved1;		//保留值
	uint8_t Reserved2;		//保留值
	uint8_t Reserved3;		//保留值
}MotorStateDataType;

//电池组上传给ECU的状态信息
typedef struct BMSStateInformation
{
	uint8_t SOC;				//电量百分值
	uint8_t Bat_Vol_L;			//电池电压值
	uint8_t Bat_Vol_H;			//电池电压值
	uint8_t Bat_Discharge_Cur;	//电池放电电流
	uint8_t	ErrorCodeNum;		//出现的故障
	uint8_t Reserved1;			//保留值
	uint8_t Reserved2;			//保留值
	uint8_t CRC_Value;			//校验值
}BMSStateDataType;

//共用体定义，用于将高低两字节合并
typedef union Data
{
	uint16_t Value;
	struct
	{
		uint8_t value_L;		//前面的值在低位，后面值在高位，小字节序
		uint8_t value_H;
	}val;
}ValueType;

extern ControlCmdType ControlDriveCmd;				//ECU发给驱动电机的指令
//extern ControlCmdType ControlSteeringCmd;			//ECU发给转向电机的指令
//extern ControlCmdType ControlBrakeCmd;			//ECU发给刹车电机的指令
//extern GearShiftCmdType GearCmdValue;				//上装发送给ECU的挡位状态信息
//extern SteeringControlCmdType SteeringCmdValue;	//上装发送给ECU的转向控制信息
//extern DriveControlCmdType DriveCmdValue;			//上装发送给ECU的驱动控制信息
//extern BrakeControlCmdType BrakeCmdValue;			//上装发送给ECU的刹车控制信号
//extern ParkingCmdType	ParkingCmdValue;			//上装发送给ECU的驻车控制信号

//extern VCUFdBDataType VCUFeedBackData;			//ECU给上装的反馈数据
//extern BMSFdBDataType BMSFeedBackData;			//ECU给上装的电池反馈数据
//extern EnableFdBDataType EnableFeedBackData;		//ECU给上装的使能状态反馈数据
//extern ErrCodeFdBType ErrCodeFeedBack;			//ECU给上装的错误码反馈数据

extern MotorStateDataType MotorStateFdB;			//底盘电机状态反馈

extern ValueType ValueData;

/****************************************
函数声明
*****************************************/
void Motor_GPIO_Config(void);						//配置电机控制器的控制端口
//void ReceiveCMD(uint32_t id, uint8_t *arr);			//ECU接收到的上装指令代码定义
//void FeedbackData(uint32_t slave_id ,uint8_t *arr);	//ECU反馈给上装的数据代码定义
//void FeedBackDataGenerate(uint8_t *gear_arr,uint8_t *error_num); //生成ECU反馈给上装的错误码和挡位信号
void MotorExecuteCMD(uint8_t* cmd);		//电机执行ECU发过来的命令
#endif

