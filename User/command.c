/***********************************************************************
文件名称：command.C
功    能：ECU处理各种接收到的命令和反馈的数据
实验平台：基于STM32F407VET6 开发板
库版本  ：V1.2.1 
***********************************************************************/
#include "main.h"	

ValueType ValueData;

ControlCmdType ControlDriveCmd;				//ECU发给驱动电机的指令
//ControlCmdType ControlSteeringCmd;			//ECU发给转向电机的指令
//ControlCmdType ControlBrakeCmd;				//ECU发给刹车电机的指令

//GearShiftCmdType GearCmdValue;				//上装发送给ECU的挡位状态信息
//SteeringControlCmdType SteeringCmdValue;	//上装发送给ECU的转向控制信息
//DriveControlCmdType DriveCmdValue;		//上装发送给ECU的驱动控制信息
//BrakeControlCmdType BrakeCmdValue;		//上装发送给ECU的刹车控制信号
//ParkingCmdType ParkingCmdValue;			//上装发送给ECU的驻车控制信号

//VCUFdBDataType VCUFeedBackData;				//ECU给上装的反馈数据
//BMSFdBDataType BMSFeedBackData;				//ECU给上装的电池反馈数据
//EnableFdBDataType EnableFeedBackData;		//ECU给上装的使能状态反馈数据
//ErrCodeFdBType ErrCodeFeedBack;				//ECU给上装的错误码反馈数据

//BMSStateDataType BMSSampData;				//电池部分发给ECU的数据定义

MotorStateDataType MotorStateFdB;			//底盘电机状态反馈

//成员1：Drive_Gear_Pos;成员2：Brake_Gear_Pos;
uint8_t GearPosState[2] = {0,0};	
//成员1：BMS_ErrorCodeNum;成员2：Drive_ErrorCodeNum;成员3：Brake_ErrorCodeNum;
uint8_t ErrorCodeNumber[10] = {0,0,0,0,0,0,0,0,0,0};

void Motor_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE); 	
	//高速控制端：PA12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//刹车控制端：PE10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//反转控制端：PD10  低速控制端：PD11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/*初始化完后，关闭3个LED*/ 
	BRAKE_OFF;
	BACK_OFF;
	LOW_SPEED_OFF;
	HIGH_SPEED_OFF;
}

///****************************************
//接收上装指令代码定义
//*****************************************/
//void ReceiveCMD(uint32_t id, uint8_t *arr)
//{
//	switch(id)
//	{
//		case GEAR_MODE_CMD_ID:		
//			GearCmdValue.Gear_Enb = ((GearShiftCmdType*)arr)->Gear_Enb;
//			GearCmdValue.Gear_Shift_Req = ((GearShiftCmdType*)arr)->Gear_Shift_Req;
//			GearCmdValue.IPC_ModeCtrl = ((GearShiftCmdType*)arr)->IPC_ModeCtrl;
//			GearCmdValue.IPC_Stop_Eme = ((GearShiftCmdType*)arr)->IPC_Stop_Eme;
//			break;
//		case STEER_CMD_ID:
////			SteeringCmdValue = (SteeringControlCmdType*)arr;
////			ControlSteeringCmd.SlaveID = STEERING_ID;
////			ControlSteeringCmd.DataLen = 2;
//			ControlSteeringCmd.Value_L = ((SteeringControlCmdType*)arr)->Steering_Pos_Req_L;
//			ControlSteeringCmd.Value_H = ((SteeringControlCmdType*)arr)->Steering_Pos_Req_H;
//			break;
//		case DRIVE_CMD_ID:
////			DriveCmdValue = (DriveControlCmdType)arr;
////			ControlDriveCmd.SlaveID = DRIVE_ID;
////			ControlDriveCmd.DataLen = 2;
//			ControlDriveCmd.Value_L = ((DriveControlCmdType*)arr)->Drive_Tq_Req_L;
//			ControlDriveCmd.Value_H = ((DriveControlCmdType*)arr)->Drive_Tq_Req_H;
//			break;
//		case BRAKE_CMD_ID:
////			ParkingCmdValue = (ParkingCmdType)arr;
////			ControlBrakeCmd.SlaveID = BRAKE_ID;
////			ControlBrakeCmd.DataLen = 2;
//			ControlBrakeCmd.Value_L = ((ParkingCmdType*)arr)->Parking_Cmd;
//			break;
//		default:break;
//	}
//}

///****************************************
//反馈给上装的数据代码定义
//*****************************************/
//void FeedbackData(uint32_t slave_id ,uint8_t *arr)
//{
//	switch(slave_id)
//	{
//		case BMS_FdB_ID:
//			//给上装设备的反馈数据清零
//			BMSFeedBackData.SOC = 0;
//			BMSFeedBackData.Bat_Vol_L = 0;
//			BMSFeedBackData.Bat_Vol_H = 0;
//			BMSFeedBackData.Bat_Discharge_Cur = 0;
//			BMSFeedBackData.CRC_Value = 0;
//			ErrCodeFeedBack.Code_Num = 0;	
//			//给反馈数据更新反馈数据
//			BMSFeedBackData.SOC = ((BMSStateDataType*)arr)->SOC;
//			BMSFeedBackData.Bat_Vol_L = ((BMSStateDataType*)arr)->Bat_Vol_L;
//			BMSFeedBackData.Bat_Vol_H = ((BMSStateDataType*)arr)->Bat_Vol_H;
//			BMSFeedBackData.Bat_Discharge_Cur = ((BMSStateDataType*)arr)->Bat_Discharge_Cur;
//			ErrorCodeNumber[0] = ((BMSStateDataType*)arr)->ErrorCodeNum;
//			ErrCodeFeedBack.Code_Num = ((BMSStateDataType*)arr)->ErrorCodeNum;
//			if(BMSSampData.ErrorCodeNum != 0)
//				ErrCodeFeedBack.Error_code = 1;
//			break;
//		case STEERING_FdB_ID:
//			//给上装设备的反馈数据清零
//			EnableFeedBackData.Steering_Enable = 0;
//			VCUFeedBackData.Steering_Ang_L = 0;
//			VCUFeedBackData.Steering_Ang_H = 0;
//			ErrCodeFeedBack.Code_Num = 0;
//			//给反馈数据更新反馈数据
//			EnableFeedBackData.Steering_Enable = arr[0];
//			VCUFeedBackData.Steering_Ang_L = arr[2];	
//			VCUFeedBackData.Steering_Ang_H = arr[3];	
//			ErrorCodeNumber[1] = arr[4];
//			ErrCodeFeedBack.Code_Num = arr[4];
//			if(arr[4] == 1)
//				ErrCodeFeedBack.Code_Num = 2;
//			break;
//		case DRIVE_FdB_ID:
//			//给上装设备的反馈数据清零
//			EnableFeedBackData.Drive_Enable = 0;
//			VCUFeedBackData.Gear_Pos = 0;
//			VCUFeedBackData.Speed_L = 0;
//			VCUFeedBackData.Speed_H = 0;
//			ErrCodeFeedBack.Code_Num = 0;
//			//给反馈数据更新反馈数据
//			EnableFeedBackData.Drive_Enable = arr[0];
//			GearPosState[0] = arr[1];
//			VCUFeedBackData.Gear_Pos = arr[1];
//			VCUFeedBackData.Speed_L = arr[2];
//			VCUFeedBackData.Speed_H = arr[3];
//			ErrorCodeNumber[2] = arr[4];
//			ErrCodeFeedBack.Code_Num = arr[4];
//			if(arr[4] == 2)
//				ErrCodeFeedBack.Code_Num = 2;
//			break;
//		case BRAKE_FdB_ID:
//			//给上装设备的反馈数据清零
//			EnableFeedBackData.Brake_Enable = 0;
//			VCUFeedBackData.Gear_Pos = 0;
//			VCUFeedBackData.Brake_Tq_L = 0;
//			VCUFeedBackData.Brake_Tq_H = 0;
//			//给反馈数据更新反馈数据
//			EnableFeedBackData.Brake_Enable = arr[0];
//			GearPosState[1] = arr[1];
//			VCUFeedBackData.Gear_Pos = arr[1];
//			VCUFeedBackData.Brake_Tq_L = arr[2];
//			VCUFeedBackData.Brake_Tq_H = arr[3];
//			break;
//		default:break;
//	}
//}

//void FeedBackDataGenerate(uint8_t *gear_arr,uint8_t *error_num)
//{
//	//当驱动电机挡位为：前进和后退时，整车的挡位就确定为这两种之一
//	//当驱动电机挡位为：空挡或无效时，刹车电机为驻车档时，整车挡位为驻车档
//	if(gear_arr[0]<=5 && gear_arr[1]<=5)
//	{
//		if(gear_arr[0] != 0)
//		{
//			if((gear_arr[0] == 3) || (gear_arr[0] == 4))
//			{
//				VCUFeedBackData.Gear_Pos = gear_arr[0];
//			}
//		}
//		if(gear_arr[1] != 0)
//		{
//			if((gear_arr[0] == 0) || (gear_arr[0] == 2))
//			{
//				VCUFeedBackData.Gear_Pos = gear_arr[1];
//			}
//		}
//	}
//	
//	if(error_num[1] != 0)
//	{
//		ErrCodeFeedBack.Error_code = 2;
//		ErrCodeFeedBack.Code_Num = error_num[1];
//	}
//	else if(error_num[2] != 0) 
//	{
//		ErrCodeFeedBack.Error_code = 2;
//		ErrCodeFeedBack.Code_Num = error_num[1];
//	}
//	else if(error_num[0] != 0) 
//	{
//		ErrCodeFeedBack.Error_code = 1;
//		ErrCodeFeedBack.Code_Num = error_num[0];
//	}
//}
//根据接收到的速度指令调节电机转速
void ChangeMotorSpeed(uint16_t* speed,uint16_t target_val,uint8_t gear_val)
{
	static uint32_t delay_count = 0,delay_time = 0;
	static uint8_t temp_gear_val = 0;
	
	if(temp_gear_val != gear_val)
	{
		delay_count = 0;
		delay_time = 0xFFFF;
		temp_gear_val = gear_val;
	}
	
	if(*speed != target_val)	//当前的速度不等于接收到的速度指令时，执行以下程序
	{
		if(*speed < target_val)	//当前速度小于目标速度时，执行以下程序
		{
			if(delay_count == 1)	//每次延时到后，延时变量先加加才进行判断
			{
				TIM_SetCompare3(TIM1,*speed);//速度必须为指针变量，因为还要将实际速度返回给ECU
//				if(*speed <= MOTOR_STOP_VALUE )		//PWM值小于120(1.2V)时电机就不转了，所以设为零点
//				{
//					*speed += 2;
//				}else if(*speed < MOTOR_MAX_SPEED_VALUE)		//PWM值大于300(3V)时，电机就不转了，所以需做限制
				if(*speed < MOTOR_MAX_SPEED_VALUE)		//PWM值大于300(3V)时，电机就不转了，所以需做限制
				{
					(*speed)++;			//速度递增
				}
				else
				{
					*speed = MOTOR_MAX_SPEED_VALUE;
				}	
			}
			else if(delay_count >= delay_time)	//循环查询变量值实现延时，
			{								//实现平稳调速的目的
				delay_count = 0;
				delay_time = 0x1FFF;
			}
			delay_count++;
		}
		if(*speed > target_val)		//当前速度大于目标速度时，执行以下程序
		{
			if(delay_count == 1)	//每次延时到后，延时变量先加加才进行判断
			{
				TIM_SetCompare3(TIM1,*speed);
				if(*speed <= MOTOR_STOP_VALUE)	//PWM值小于120(1.2V)时电机就不转了，所以设为零点
				{
					*speed = 0;		//为确保电机不转，直接清零
				}
				if(*speed > 0)		//速度递减
				{
					(*speed)--;
				}
				else
				{
					*speed = 0;
				}
			}
			else if(delay_count >= delay_time)	//循环查询变量值实现延时，
			{
				delay_count = 0;			//实现平稳调速的目的
				delay_time = 0x1FFF;
			}
			delay_count++;
		}
	}
}

//执行接收到电机控制指令
void MotorExecuteCMD(uint8_t* cmd)
{
	static uint8_t gear_flag = 0;			//挡位反馈值
	static uint8_t speed_flag = 0;	//速度反馈值
	static uint16_t motor_speed_record = 0;	//当前电机速度
	
	if(can1_rec_flag == 1)			//收到ECU发送的指令
	{
		ValueData.val.value_L = ControlDriveCmd.Value_L;
		ValueData.val.value_H = ControlDriveCmd.Value_H;
		CAN1_RX_data = ValueData.Value;		//获取速度值	
		can1_rec_flag = 0;
	}
	switch(ControlDriveCmd.SpeedFlag)		//读取高速、低速控制指令
	{
		case LOW_SPEED_GEAR:
			HIGH_SPEED_OFF;	
			LOW_SPEED_ON;					//开启低速模式
			speed_flag = LOW_SPEED_GEAR;
			break;
		case HIGH_SPEED_GEAR:
			LOW_SPEED_OFF;
			HIGH_SPEED_ON;					//开启高速模式
			speed_flag = HIGH_SPEED_GEAR;
			break;
		case NORMAL_SPEED_GEAR:				//正常速度模式
			LOW_SPEED_OFF;
			HIGH_SPEED_OFF;
			speed_flag = NORMAL_SPEED_GEAR;
			break;
		default:break;
	}
	switch(ControlDriveCmd.GearPos)			//读取挡位命令
	{
		case PARKING:						//驻车档
			motor_speed_record = 0;			//电机速度清零
			TIM_SetCompare3(TIM1,motor_speed_record);
			BRAKE_ON;						//刹车
			BACK_OFF;
			gear_flag = PARKING;			//挡位反馈值
			MotorStateFdB.EnableState = CMD_ENABLE;
			break;
		case NULL:							//空挡
			motor_speed_record = 0;			//电机速度清零
			TIM_SetCompare3(TIM1,motor_speed_record);
			BRAKE_ON;
			BACK_OFF;
			gear_flag = NULL;				//挡位反馈值
			MotorStateFdB.EnableState = CMD_ENABLE;
			break;
		case FORWARD:						//前进挡
			BRAKE_OFF;
			BACK_OFF;
			if(gear_flag != FORWARD)		//切换正反转时，速度值必须清零
			{								//否则切换不过来
				motor_speed_record = 0;	
			}
			gear_flag = FORWARD;
			ChangeMotorSpeed(&motor_speed_record,CAN1_RX_data,gear_flag);
			MotorStateFdB.EnableState = CMD_ENABLE;
			break;
		case BACK:
			BRAKE_OFF;
			if(gear_flag != BACK)			//后退挡
			{								//切换正反转时，速度值必须清零
				motor_speed_record = 0;		//否则切换不过来
				BACK_OFF;
			}
			else
			{
				BACK_ON;
			}	
			gear_flag = BACK;
			ChangeMotorSpeed(&motor_speed_record,CAN1_RX_data,gear_flag);
			MotorStateFdB.EnableState = CMD_ENABLE;
			break;
		default:break;
	}
	MotorStateFdB.ModeState = gear_flag;		//将挡位信号反馈给ECU
	MotorStateFdB.Value_L = motor_speed_record & 0xFF;		//将电机速度反馈给ECU
	MotorStateFdB.Value_H = (motor_speed_record>>8) & 0xFF;
	MotorStateFdB.ErrorCodeNum = 0;				//将错误码反馈给ECU
//			can1_tx(DRIVE_FdB_ID, ((uint8_t*)&MotorStateFdB));
}
