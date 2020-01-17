/***********************************************************************
�ļ����ƣ�command.C
��    �ܣ�ECU������ֽ��յ�������ͷ���������
ʵ��ƽ̨������STM32F407VET6 ������
��汾  ��V1.2.1 
***********************************************************************/
#include "main.h"	

ValueType ValueData;

ControlCmdType ControlDriveCmd;				//ECU�������������ָ��
//ControlCmdType ControlSteeringCmd;			//ECU����ת������ָ��
//ControlCmdType ControlBrakeCmd;				//ECU����ɲ�������ָ��

//GearShiftCmdType GearCmdValue;				//��װ���͸�ECU�ĵ�λ״̬��Ϣ
//SteeringControlCmdType SteeringCmdValue;	//��װ���͸�ECU��ת�������Ϣ
//DriveControlCmdType DriveCmdValue;		//��װ���͸�ECU������������Ϣ
//BrakeControlCmdType BrakeCmdValue;		//��װ���͸�ECU��ɲ�������ź�
//ParkingCmdType ParkingCmdValue;			//��װ���͸�ECU��פ�������ź�

//VCUFdBDataType VCUFeedBackData;				//ECU����װ�ķ�������
//BMSFdBDataType BMSFeedBackData;				//ECU����װ�ĵ�ط�������
//EnableFdBDataType EnableFeedBackData;		//ECU����װ��ʹ��״̬��������
//ErrCodeFdBType ErrCodeFeedBack;				//ECU����װ�Ĵ����뷴������

//BMSStateDataType BMSSampData;				//��ز��ַ���ECU�����ݶ���

MotorStateDataType MotorStateFdB;			//���̵��״̬����

//��Ա1��Drive_Gear_Pos;��Ա2��Brake_Gear_Pos;
uint8_t GearPosState[2] = {0,0};	
//��Ա1��BMS_ErrorCodeNum;��Ա2��Drive_ErrorCodeNum;��Ա3��Brake_ErrorCodeNum;
uint8_t ErrorCodeNumber[10] = {0,0,0,0,0,0,0,0,0,0};

void Motor_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE); 	
	//���ٿ��ƶˣ�PA12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//ɲ�����ƶˣ�PE10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//��ת���ƶˣ�PD10  ���ٿ��ƶˣ�PD11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/*��ʼ����󣬹ر�3��LED*/ 
	BRAKE_OFF;
	BACK_OFF;
	LOW_SPEED_OFF;
	HIGH_SPEED_OFF;
}

///****************************************
//������װָ����붨��
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
//��������װ�����ݴ��붨��
//*****************************************/
//void FeedbackData(uint32_t slave_id ,uint8_t *arr)
//{
//	switch(slave_id)
//	{
//		case BMS_FdB_ID:
//			//����װ�豸�ķ�����������
//			BMSFeedBackData.SOC = 0;
//			BMSFeedBackData.Bat_Vol_L = 0;
//			BMSFeedBackData.Bat_Vol_H = 0;
//			BMSFeedBackData.Bat_Discharge_Cur = 0;
//			BMSFeedBackData.CRC_Value = 0;
//			ErrCodeFeedBack.Code_Num = 0;	
//			//���������ݸ��·�������
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
//			//����װ�豸�ķ�����������
//			EnableFeedBackData.Steering_Enable = 0;
//			VCUFeedBackData.Steering_Ang_L = 0;
//			VCUFeedBackData.Steering_Ang_H = 0;
//			ErrCodeFeedBack.Code_Num = 0;
//			//���������ݸ��·�������
//			EnableFeedBackData.Steering_Enable = arr[0];
//			VCUFeedBackData.Steering_Ang_L = arr[2];	
//			VCUFeedBackData.Steering_Ang_H = arr[3];	
//			ErrorCodeNumber[1] = arr[4];
//			ErrCodeFeedBack.Code_Num = arr[4];
//			if(arr[4] == 1)
//				ErrCodeFeedBack.Code_Num = 2;
//			break;
//		case DRIVE_FdB_ID:
//			//����װ�豸�ķ�����������
//			EnableFeedBackData.Drive_Enable = 0;
//			VCUFeedBackData.Gear_Pos = 0;
//			VCUFeedBackData.Speed_L = 0;
//			VCUFeedBackData.Speed_H = 0;
//			ErrCodeFeedBack.Code_Num = 0;
//			//���������ݸ��·�������
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
//			//����װ�豸�ķ�����������
//			EnableFeedBackData.Brake_Enable = 0;
//			VCUFeedBackData.Gear_Pos = 0;
//			VCUFeedBackData.Brake_Tq_L = 0;
//			VCUFeedBackData.Brake_Tq_H = 0;
//			//���������ݸ��·�������
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
//	//�����������λΪ��ǰ���ͺ���ʱ�������ĵ�λ��ȷ��Ϊ������֮һ
//	//�����������λΪ���յ�����Чʱ��ɲ�����Ϊפ����ʱ��������λΪפ����
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
//���ݽ��յ����ٶ�ָ����ڵ��ת��
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
	
	if(*speed != target_val)	//��ǰ���ٶȲ����ڽ��յ����ٶ�ָ��ʱ��ִ�����³���
	{
		if(*speed < target_val)	//��ǰ�ٶ�С��Ŀ���ٶ�ʱ��ִ�����³���
		{
			if(delay_count == 1)	//ÿ����ʱ������ʱ�����ȼӼӲŽ����ж�
			{
				TIM_SetCompare3(TIM1,*speed);//�ٶȱ���Ϊָ���������Ϊ��Ҫ��ʵ���ٶȷ��ظ�ECU
//				if(*speed <= MOTOR_STOP_VALUE )		//PWMֵС��120(1.2V)ʱ����Ͳ�ת�ˣ�������Ϊ���
//				{
//					*speed += 2;
//				}else if(*speed < MOTOR_MAX_SPEED_VALUE)		//PWMֵ����300(3V)ʱ������Ͳ�ת�ˣ�������������
				if(*speed < MOTOR_MAX_SPEED_VALUE)		//PWMֵ����300(3V)ʱ������Ͳ�ת�ˣ�������������
				{
					(*speed)++;			//�ٶȵ���
				}
				else
				{
					*speed = MOTOR_MAX_SPEED_VALUE;
				}	
			}
			else if(delay_count >= delay_time)	//ѭ����ѯ����ֵʵ����ʱ��
			{								//ʵ��ƽ�ȵ��ٵ�Ŀ��
				delay_count = 0;
				delay_time = 0x1FFF;
			}
			delay_count++;
		}
		if(*speed > target_val)		//��ǰ�ٶȴ���Ŀ���ٶ�ʱ��ִ�����³���
		{
			if(delay_count == 1)	//ÿ����ʱ������ʱ�����ȼӼӲŽ����ж�
			{
				TIM_SetCompare3(TIM1,*speed);
				if(*speed <= MOTOR_STOP_VALUE)	//PWMֵС��120(1.2V)ʱ����Ͳ�ת�ˣ�������Ϊ���
				{
					*speed = 0;		//Ϊȷ�������ת��ֱ������
				}
				if(*speed > 0)		//�ٶȵݼ�
				{
					(*speed)--;
				}
				else
				{
					*speed = 0;
				}
			}
			else if(delay_count >= delay_time)	//ѭ����ѯ����ֵʵ����ʱ��
			{
				delay_count = 0;			//ʵ��ƽ�ȵ��ٵ�Ŀ��
				delay_time = 0x1FFF;
			}
			delay_count++;
		}
	}
}

//ִ�н��յ��������ָ��
void MotorExecuteCMD(uint8_t* cmd)
{
	static uint8_t gear_flag = 0;			//��λ����ֵ
	static uint8_t speed_flag = 0;	//�ٶȷ���ֵ
	static uint16_t motor_speed_record = 0;	//��ǰ����ٶ�
	
	if(can1_rec_flag == 1)			//�յ�ECU���͵�ָ��
	{
		ValueData.val.value_L = ControlDriveCmd.Value_L;
		ValueData.val.value_H = ControlDriveCmd.Value_H;
		CAN1_RX_data = ValueData.Value;		//��ȡ�ٶ�ֵ	
		can1_rec_flag = 0;
	}
	switch(ControlDriveCmd.SpeedFlag)		//��ȡ���١����ٿ���ָ��
	{
		case LOW_SPEED_GEAR:
			HIGH_SPEED_OFF;	
			LOW_SPEED_ON;					//��������ģʽ
			speed_flag = LOW_SPEED_GEAR;
			break;
		case HIGH_SPEED_GEAR:
			LOW_SPEED_OFF;
			HIGH_SPEED_ON;					//��������ģʽ
			speed_flag = HIGH_SPEED_GEAR;
			break;
		case NORMAL_SPEED_GEAR:				//�����ٶ�ģʽ
			LOW_SPEED_OFF;
			HIGH_SPEED_OFF;
			speed_flag = NORMAL_SPEED_GEAR;
			break;
		default:break;
	}
	switch(ControlDriveCmd.GearPos)			//��ȡ��λ����
	{
		case PARKING:						//פ����
			motor_speed_record = 0;			//����ٶ�����
			TIM_SetCompare3(TIM1,motor_speed_record);
			BRAKE_ON;						//ɲ��
			BACK_OFF;
			gear_flag = PARKING;			//��λ����ֵ
			MotorStateFdB.EnableState = CMD_ENABLE;
			break;
		case NULL:							//�յ�
			motor_speed_record = 0;			//����ٶ�����
			TIM_SetCompare3(TIM1,motor_speed_record);
			BRAKE_ON;
			BACK_OFF;
			gear_flag = NULL;				//��λ����ֵ
			MotorStateFdB.EnableState = CMD_ENABLE;
			break;
		case FORWARD:						//ǰ����
			BRAKE_OFF;
			BACK_OFF;
			if(gear_flag != FORWARD)		//�л�����תʱ���ٶ�ֵ��������
			{								//�����л�������
				motor_speed_record = 0;	
			}
			gear_flag = FORWARD;
			ChangeMotorSpeed(&motor_speed_record,CAN1_RX_data,gear_flag);
			MotorStateFdB.EnableState = CMD_ENABLE;
			break;
		case BACK:
			BRAKE_OFF;
			if(gear_flag != BACK)			//���˵�
			{								//�л�����תʱ���ٶ�ֵ��������
				motor_speed_record = 0;		//�����л�������
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
	MotorStateFdB.ModeState = gear_flag;		//����λ�źŷ�����ECU
	MotorStateFdB.Value_L = motor_speed_record & 0xFF;		//������ٶȷ�����ECU
	MotorStateFdB.Value_H = (motor_speed_record>>8) & 0xFF;
	MotorStateFdB.ErrorCodeNum = 0;				//�������뷴����ECU
//			can1_tx(DRIVE_FdB_ID, ((uint8_t*)&MotorStateFdB));
}
