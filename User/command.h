#ifndef _COMMAND_H_
#define _COMMAND_H_

#include "stm32f4xx.h"

/****************************************
��װָ��ID����
*****************************************/
#define GEAR_MODE_CMD_ID 	0xA1	//����״̬ģʽ�л�ģʽID
#define STEER_CMD_ID		0xA2	//ת��ָ��ID
#define DRIVE_CMD_ID		0xA3	//����ָ��ID
#define BRAKE_CMD_ID		0xA4	//ɲ��ָ��ID
#define	PARK_CMD_ID			0xA5	//פ��ָ��ID

#define	VCU_FEEDBACK_ID		0xC1	//VCU״̬��������ID
#define	BMS_FEEDBACK_ID		0xC2	//BMS״̬��������ID
#define	ENABLE_FEEDBACK_ID	0xC3	//ENABLE״̬����ID
#define	ERR_FEEDBACK_ID		0xE1	//������״̬����ID

#define BRAKE_ID			0x41	//ɲ�����ID
#define STEERING_ID			0x42	//ת����ID
#define DRIVE_ID			0x44	//�������ID
#define	BMS_ID				0x43	//���BMSID

#define BRAKE_FdB_ID		0x81	//ɲ�������������ID
#define STEERING_FdB_ID		0x82	//ת������������ID
#define DRIVE_FdB_ID		0x84	//���������������ID
#define	BMS_FdB_ID			0x83	//���BMS�����������ID

#define CMD_ENABLE 	1
#define CMD_DISABLE 0

#define PARKING		1				//פ��
#define NULL		2				//�յ�
#define	FORWARD		3				//ǰ��
#define	BACK		4				//����
#define INVALID		5				//��Ч

#define LOW_SPEED_GEAR 		1		//���ٵ�
#define NORMAL_SPEED_GEAR 	2		//�����ٶȵ�
#define HIGH_SPEED_GEAR 	3		//���ٵ�

#define MOTOR_STOP_VALUE	120		//���ֹͣʱ��PWMֵ
#define MOTOR_MAX_SPEED_VALUE	300	//�������ٶ�ʱ��PWMֵ

#define BRAKE_PIN 			GPIOE , GPIO_Pin_10		//ɲ������
#define BACK_PIN 			GPIOD , GPIO_Pin_10		//��������
#define LOW_SPEED_PIN 		GPIOD , GPIO_Pin_11		//���ٿ���
#define HIGH_SPEED_PIN 		GPIOA , GPIO_Pin_12		//���ٿ���

#define BRAKE_OFF 			GPIO_ResetBits(GPIOE , GPIO_Pin_10)	//ɲ��ʧ��
#define BACK_OFF 			GPIO_ResetBits(GPIOD , GPIO_Pin_10)	//����ʧ��
#define LOW_SPEED_OFF 		GPIO_ResetBits(GPIOD , GPIO_Pin_11)	//����ʧ��
#define HIGH_SPEED_OFF 		GPIO_ResetBits(GPIOA , GPIO_Pin_12)	//����ʧ��

#define BRAKE_ON 			GPIO_SetBits(GPIOE , GPIO_Pin_10)	//ɲ��ʹ��
#define BACK_ON 			GPIO_SetBits(GPIOD , GPIO_Pin_10)	//����ʹ��
#define LOW_SPEED_ON 		GPIO_SetBits(GPIOD , GPIO_Pin_11)	//����ʹ��
#define HIGH_SPEED_ON 		GPIO_SetBits(GPIOA , GPIO_Pin_12)	//����ʹ��

/****************************************
ECU������װָ����붨��
*****************************************/
//��װ���͵�����״̬����
typedef struct GearModeCmd
{
	uint8_t Gear_Enb;			//��λʹ���ź�
	uint8_t Gear_Shift_Req;		//��λ�л��ź�
	uint8_t IPC_ModeCtrl;		//IPC�·�ģʽ�ź�
	uint8_t IPC_Stop_Eme;		//��װ��ָͣ��
}GearShiftCmdType;
//��װ���͵�ת������
typedef struct SteeringCmd
{
	uint8_t Steering_Enb;		//ת����ʹ���ź�
	uint8_t Steering_Pos_Req_L;	//ת��Ƕ�ֵ
	uint8_t Steering_Pos_Req_H;	//ת��Ƕ�ֵ
	uint8_t Reserved;			//����λ
}SteeringControlCmdType;
//��װ���͵������������
typedef struct DriveCmd
{
	uint8_t Drive_Enb;			//�������ʹ���ź�
	uint8_t Drive_Tq_Req_L;		//�����������ֵ
	uint8_t Drive_Tq_Req_H;		//�����������ֵ
	uint8_t Reserved;			//����λ
}DriveControlCmdType;
//��װ���͵�ɲ������
typedef struct BrakeCmd
{
	uint8_t Brake_Enb;			//ɲ��ʹ���ź�
	uint8_t Brake_Tq_Req_L;		//ɲ���������ֵ
	uint8_t Brake_Tq_Req_H;		//ɲ���������ֵ
	uint8_t Reserved;			//����λ
}BrakeControlCmdType;
//��װ���͵�פ������
typedef struct ParkingCmd
{
	uint8_t Parking_Enb;		//פ��ʹ���ź�
	uint8_t Parking_Cmd;		//פ������ֵ
	uint8_t Reserved;			//����λ
}ParkingCmdType;

/****************************************
ECU��������װ�����ݴ��붨��
*****************************************/
//��������װ������״̬����
typedef struct VCUData
{
	uint8_t Speed_L;			//�ٶ�ֵ
	uint8_t Speed_H;			//�ٶ�ֵ
	uint8_t Steering_Ang_L;		//ת��Ƕ�
	uint8_t Steering_Ang_H;		//ת��Ƕ�
	uint8_t Gear_Pos;			//��λ�ź�
	uint8_t Brake_Tq_L;			//ɲ������
	uint8_t Brake_Tq_H;			//ɲ������
	uint8_t CRC_Value;			//У��ֵ
}VCUFdBDataType;
//��������װ�ĵ������
typedef struct BMSData
{
	uint8_t SOC;				//�����ٷ�ֵ
	uint8_t Bat_Vol_L;			//��ص�ѹֵ
	uint8_t Bat_Vol_H;			//��ص�ѹֵ
	uint8_t Bat_Discharge_Cur;	//��طŵ����
	uint8_t Reserved1;			//����λ
	uint8_t Reserved2;			//����λ
	uint8_t Reserved3;			//����λ
	uint8_t CRC_Value;			//У��ֵ
}BMSFdBDataType;
//��������װ��ECU�����������ִ�н��
typedef struct EnableState
{
	uint8_t Gear_Enable;		//��λʹ��
	uint8_t Steering_Enable;	//ת��ʹ��
	uint8_t Drive_Enable;		//����ʹ��
	uint8_t Brake_Enable;		//�ƶ�ʹ��
	uint8_t Parking_Enable;		//פ��ʹ��
	uint8_t RC_Takeover_Flag;	//�ӹܱ�־λ
	uint8_t Reserved;			//����λ
	uint8_t CRC_Value;			//У��ֵ
}EnableFdBDataType;
//��������װ�Ĵ�����
typedef struct ErrState
{
	uint8_t Error_code;			//������
	uint8_t Code_Num;			//���������
	uint8_t Reserved1;			//����λ
	uint8_t Reserved2;			//����λ
	uint8_t Reserved3;			//����λ
	uint8_t Reserved4;			//����λ
	uint8_t Reserved5;			//����λ
	uint8_t CRC_Value;			//У��ֵ
}ErrCodeFdBType;

/****************************************
�·�ָ����붨��
*****************************************/
//�·�������ģ��Ŀ���ָ��
typedef struct SendControlCmd
{
	uint8_t GearPos;		//��λ
	uint8_t Value_L;		//���Ʊ仯��ֵ
	uint8_t Value_H;		//���Ʊ仯��ֵ
	uint8_t SpeedFlag;		//�ٶȵ�λ
}ControlCmdType;

/****************************************
���̸������ϴ���ECU��״̬��Ϣ���붨��
*****************************************/
//��������ϴ���ECU��״̬��Ϣ
typedef struct MotorStateInformation
{
	uint8_t EnableState;	//���յ���ECUָ��ִ�е�״̬
	uint8_t	ModeState;		//������״̬����λ�źţ�
	uint8_t Value_L;		//ִ�е�ֵ
	uint8_t Value_H;		//ִ�е�ֵ
	uint8_t	ErrorCodeNum;	//���ֵĹ���
	uint8_t Reserved1;		//����ֵ
	uint8_t Reserved2;		//����ֵ
	uint8_t Reserved3;		//����ֵ
}MotorStateDataType;

//������ϴ���ECU��״̬��Ϣ
typedef struct BMSStateInformation
{
	uint8_t SOC;				//�����ٷ�ֵ
	uint8_t Bat_Vol_L;			//��ص�ѹֵ
	uint8_t Bat_Vol_H;			//��ص�ѹֵ
	uint8_t Bat_Discharge_Cur;	//��طŵ����
	uint8_t	ErrorCodeNum;		//���ֵĹ���
	uint8_t Reserved1;			//����ֵ
	uint8_t Reserved2;			//����ֵ
	uint8_t CRC_Value;			//У��ֵ
}BMSStateDataType;

//�����嶨�壬���ڽ��ߵ����ֽںϲ�
typedef union Data
{
	uint16_t Value;
	struct
	{
		uint8_t value_L;		//ǰ���ֵ�ڵ�λ������ֵ�ڸ�λ��С�ֽ���
		uint8_t value_H;
	}val;
}ValueType;

extern ControlCmdType ControlDriveCmd;				//ECU�������������ָ��
//extern ControlCmdType ControlSteeringCmd;			//ECU����ת������ָ��
//extern ControlCmdType ControlBrakeCmd;			//ECU����ɲ�������ָ��
//extern GearShiftCmdType GearCmdValue;				//��װ���͸�ECU�ĵ�λ״̬��Ϣ
//extern SteeringControlCmdType SteeringCmdValue;	//��װ���͸�ECU��ת�������Ϣ
//extern DriveControlCmdType DriveCmdValue;			//��װ���͸�ECU������������Ϣ
//extern BrakeControlCmdType BrakeCmdValue;			//��װ���͸�ECU��ɲ�������ź�
//extern ParkingCmdType	ParkingCmdValue;			//��װ���͸�ECU��פ�������ź�

//extern VCUFdBDataType VCUFeedBackData;			//ECU����װ�ķ�������
//extern BMSFdBDataType BMSFeedBackData;			//ECU����װ�ĵ�ط�������
//extern EnableFdBDataType EnableFeedBackData;		//ECU����װ��ʹ��״̬��������
//extern ErrCodeFdBType ErrCodeFeedBack;			//ECU����װ�Ĵ����뷴������

extern MotorStateDataType MotorStateFdB;			//���̵��״̬����

extern ValueType ValueData;

/****************************************
��������
*****************************************/
void Motor_GPIO_Config(void);						//���õ���������Ŀ��ƶ˿�
//void ReceiveCMD(uint32_t id, uint8_t *arr);			//ECU���յ�����װָ����붨��
//void FeedbackData(uint32_t slave_id ,uint8_t *arr);	//ECU��������װ�����ݴ��붨��
//void FeedBackDataGenerate(uint8_t *gear_arr,uint8_t *error_num); //����ECU��������װ�Ĵ�����͵�λ�ź�
void MotorExecuteCMD(uint8_t* cmd);		//���ִ��ECU������������
#endif

