/***********************************************************************
�ļ����ƣ�main.C
��    �ܣ�KEY_LED����ɨ������жϷ�ʽ
ʵ��ƽ̨������STM32F407VET6 ������
��汾  ��V1.2.1 
***********************************************************************/
#include "main.h"	


//void Delay(vu32 nCount);
//void KeyInit(void);
//void GetKey(void);

int main(void)
{
	unsigned int i=0;
	u16  ZHI=150;
	u16  A=0;	
	/*
		ϵͳʱ��������system_stm32f4xx.c �ļ��е�SystemInit()������ʵ�֣���λ��ֱ���������ļ�������
	*/
	NVIC_Configuration(); 
	SysTick_Configuration();
	TIM1_PWM_Init();
	TIM_Configuration();
	CAN1_Configuration();
	CAN2_Configuration();
	Motor_GPIO_Config();
	LED_GPIO_Config();
	while(1)
	{
		MotorExecuteCMD((uint8_t*)&ControlDriveCmd);
	}
}
//void Delay(vu32 nCount)
//{
//	for(; nCount != 0; nCount--);
//}
///*�����ܽų�ʼ��*/
//void KeyInit(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE);
//	/********************��������***************************/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 ;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
//	GPIO_Init(GPIOE, &GPIO_InitStructure);	

//}
//uint16_t TableValue[3]={120,200,300};

///*����Ƿ��а�������*/
//void  GetKey(void)
//{
//	static unsigned int key_value = 12;
//	static unsigned int switch_flag = 0;
//	
////	if(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10))
////	{
////		Delay(1000000);//ȥ����
////		if(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10))
////		{
////			while(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10)){ ; }//�ȴ������ͷ�
////			switch(switch_flag)
////			{
////				case 0: key_value += 10;
////						if(key_value>=300)
////						{
////							key_value = 300;
////							switch_flag =1;
////						}
////						break;
////				case 1: key_value -= 10;
////						if(key_value<=150)
////						{
////							key_value = 150;
////							switch_flag =0;
////						}
////						break;
////				default:break;
////			}
////			
////			can1_tx(0x11,CAN1_ID,key_value); 
////			LED1_ON;	LED2_OFF;	LED3_OFF;
////		}
////	}

//	if(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))
//	{
//		Delay(1000000);//ȥ����//ȥ����
//		if(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))
//		{
//			while(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)){ ; }//�ȴ������ͷ�   
//			switch(switch_flag)
//			{
//				case 0: key_value += 1;
//						if(key_value>=3)
//						{
//							key_value = 3;
//							switch_flag =1;
//						}
//						break;
//				case 1: key_value -= 1;
//						if(key_value<=1)
//						{
//							key_value = 1;
//							switch_flag =0;
//						}
//						break;
//				default:break;
//			}
//			
//			ControlDriveCmd.GearPos = GearCmdValue.Gear_Shift_Req;				//��λ�ź�
//			ControlDriveCmd.Value_L = TableValue[key_value-1] & 0xFF;			//���Ʊ仯��ֵ
//			ControlDriveCmd.Value_H = (TableValue[key_value-1]>>8) & 0xFF;			//���Ʊ仯��ֵ
//			can2_tx(DRIVE_ID,((uint8_t*)&ControlDriveCmd));  
////			can2_tx(0x22,CAN2_ID,250); 	
//			LED4_ON;	
//		}
//	}

//	if(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12))
//	{
//		Delay(1000000);//ȥ����//ȥ����
//		if(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12))
//		{
//			while(Bit_RESET == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)){ ; }//�ȴ������ͷ�                        
//			
////			LED1_OFF;	LED2_OFF;	LED3_OFF;
//		}
//	}            
//}



