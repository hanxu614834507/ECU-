#ifndef _CAN_H_
#define _CAN_H_

#include "stm32f4xx.h"

#define CAN_BAUD_NUM    18		//�������ò����ʸ���
#define CAN1_ID   0x0006		//CAN1��ID

#define CAN2_ID   0x221803		//CAN2��ID

extern uint8_t* CAN1_data;
extern uint16_t CAN1_RX_data;
extern unsigned char can1_rec_flag;
extern unsigned char CAN2_data[8];
extern uint16_t CAN2_RX_data;
extern unsigned char can2_rec_flag;

void CAN1_Configuration(void);
void CAN2_Configuration(void);
//void can1_tx(uint32_t ID, MotorStateDataType* state_vale);
void can1_tx(uint32_t ID, uint8_t* Data);
void can2_tx(uint32_t ID, uint8_t* Data);
void CAN1_WriteData(unsigned int ID);
void CAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure);
#endif