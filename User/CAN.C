#include "main.h"

uint8_t* CAN1_data;
uint16_t CAN1_RX_data;
unsigned char can1_rec_flag = 0;
unsigned char CAN2_data[8];
uint16_t CAN2_RX_data;
unsigned char can2_rec_flag = 0;

const unsigned int CAN_baud_table[CAN_BAUD_NUM][5] = 
{
//波特率， CAN_SJW，   CAN_BS1，    CAN_BS2，CAN_Prescaler 
	{5,   CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,450},		//未通			
	{10,  CAN_SJW_1tq,CAN_BS1_6tq,CAN_BS2_2tq, 400},		//未通			
	{15,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,150},		//15K  未通
	{20,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,200},		//20k //未通
	{25,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,112},		//25k  未通
	{40,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,100},		//40k  未通
	{50,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,56},			//50k	ok
	{62,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,36},			//62.5k
	{80,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,50},			//80k   未通
	{100, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,28},			//100K	ok
	{125, CAN_SJW_1tq,CAN_BS1_13tq, CAN_BS2_2tq,18},		//125K 未通
	{200, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,14},			//200K  ok
	{250, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,8},		    //250k  ok
	{400, CAN_SJW_1tq,CAN_BS1_15tq, CAN_BS2_5tq,5},			//400K  ok
	{500, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,4},			//500K	ok
	{666, CAN_SJW_1tq,CAN_BS1_5tq, CAN_BS2_2tq,8},			//未通
	{800, CAN_SJW_1tq,CAN_BS1_8tq, CAN_BS2_3tq,14},			//800K 未通
	{1000,CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,2},			//1000K	ok
};


//CAN1配置
void CAN1_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/* CAN GPIOs configuration **************************************************/

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 

	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

	CAN_Baud_Process(250,&CAN_InitStructure);
	CAN_Init(CAN1, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;	   //CAN1滤波器号从0到13

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //滤波屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (unsigned int)(DRIVE_ID>>13) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterIdLow = (unsigned int)(((DRIVE_ID & 0xFFFF)<<3) | CAN_ID_EXT | CAN_RTR_DATA)&0xFFFF;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;	//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;		//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;	  // /*!< Specifies the FIFO (0 or 1) which will be assigned to the filter.
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}
//CAN2配置
void CAN2_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/* CAN GPIOs configuration **************************************************/

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2); 

	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);//用can2时，can1时钟也要开启

	/* CAN register init */
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;


	CAN_Baud_Process(250,&CAN_InitStructure);
	CAN_Init(CAN2, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 14;	   //CAN2滤波器号从14到27

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //滤波屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (unsigned int)(CAN1_ID>>13) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterIdLow = (unsigned int)(((CAN1_ID & 0xFFFF)<<3) | CAN_ID_EXT | CAN_RTR_DATA)&0xFFFF;
//	CAN_FilterInitStructure.CAN_FilterIdLow = 0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;	//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;		//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;	  // /*!< Specifies the FIFO (0 or 1) which will be assigned to the filter.
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}
/***********************************************************************
函数名称：CCAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure)
功    能：计算波特率，返回
编写时间：2013.4.25
编 写 人：
注    意：CAN时钟为42M
CAN_SJW : CAN_SJW_1tq - CAN_SJW_4tq	  不能比任何一相位缓冲段长
CAN_BS1 : CAN_BS1_1tq - CAN_BS1_16tq
CAN_BS2 : CAN_BS2_1tq - CAN_BS2_8tq
CAN_Prescaler : 1 - 1024
	配置说明：
CAN_SJW + CAN_BS1 / (CAN_SJW + CAN_BS1 + CAN_BS2)
	0.75     baud > 800k
	0.80     baud > 500k
	0.875    baud <= 500k
	baud = 42M / (CAN_SJW + CAN_BS1 + CAN_BS2) / CAN_Prescaler
***********************************************************************/
void CAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure)
{
	unsigned int i = 0;
	for(i = 0;i < CAN_BAUD_NUM;i ++)
	{
		if(Baud == CAN_baud_table[i][0])
		{
			CAN_InitStructure->CAN_SJW = CAN_baud_table[i][1];
			CAN_InitStructure->CAN_BS1 = CAN_baud_table[i][2];
			CAN_InitStructure->CAN_BS2 = CAN_baud_table[i][3];
			CAN_InitStructure->CAN_Prescaler = CAN_baud_table[i][4];
			return;	
		}
	}	
}
//CAN1接收中断函数
void CAN1_RX0_IRQHandler(void)
{
	unsigned char i = 0;
	CanRxMsg RxMessage; 
	uint8_t* RxData = (uint8_t*)&ControlDriveCmd;
	
	RxMessage.StdId=0x00;
	RxMessage.ExtId=0x00;
	RxMessage.IDE=0;
	RxMessage.DLC=0;
	RxMessage.FMI=0;
	RxMessage.Data[0]=0x00;     
	
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);  /* 此函数包含释放提出报文了的,在非必要时,不需要自己释放 */
	if(RxMessage.ExtId==DRIVE_ID) 
	{ 
//		ControlDriveCmd.GearPos = RxMessage.Data[0];
//		ControlDriveCmd.Value_L = RxMessage.Data[1];
//		ControlDriveCmd.Value_H = RxMessage.Data[2];
//		ControlDriveCmd.SpeedFlag = RxMessage.Data[3];
		for(i=0;i<8;i++)
		{
			RxData[i] = RxMessage.Data[i]; 
		}
//		LED1_ON;
	}
  
	can1_rec_flag = 1;
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);  /* 清除挂起中断 */
}


//CAN1发送函数
//void can1_tx(uint32_t ID, MotorStateDataType* state_vale)
//{	
//	unsigned char i;
//	uint8_t transmit_mailbox = 0;
//	CanTxMsg TxMessage;	
//	
//	TxMessage.StdId=00;	//标准标
//	TxMessage.ExtId=ID; //扩展标识符
//	TxMessage.IDE = CAN_ID_EXT;//使用扩展标识符
//	TxMessage.RTR = CAN_RTR_DATA; /* 设置为数据帧 */
//	TxMessage.DLC = 8;            /* 数据长度, can报文规定最大的数据长度为8字节 */
//	CAN1_data = (uint8_t*)state_vale;
//	for(i = 0;i < 8; i ++)
//	{
//		TxMessage.Data[i] = CAN1_data[i];
//	}
//	transmit_mailbox = CAN_Transmit(CAN1,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
////	LED3_ON;
//	/*while((CAN_TransmitStatus(CAN1, transmit_mailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
//	{
//		i ++;
//	}*/
//}

void can1_tx(uint32_t ID, uint8_t* Data)
{	
	unsigned char i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId=00;	//标准标
	TxMessage.ExtId=ID; //扩展标识符
	TxMessage.IDE = CAN_ID_EXT;//使用扩展标识符
	TxMessage.RTR = CAN_RTR_DATA; /* 设置为数据帧 */
	TxMessage.DLC = 8;            /* 数据长度, can报文规定最大的数据长度为8字节 */
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = (uint8_t)Data[i];
	}
	transmit_mailbox = CAN_Transmit(CAN1,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
//	LED3_ON;
	/*while((CAN_TransmitStatus(CAN1, transmit_mailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
	{
		i ++;
	}*/
}

//CAN2接收中断函数
void CAN2_RX0_IRQHandler(void)
{
	unsigned char i = 0;
	CanRxMsg RxMessage;
	
	RxMessage.StdId=0x00;
	RxMessage.ExtId=0x00;
	RxMessage.IDE=0;
	RxMessage.DLC=0;
	RxMessage.FMI=0;
	RxMessage.Data[0]=0x00;     
	
	CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);  /* 此函数包含释放提出报文了的,在非必要时,不需要自己释放 */

//	FeedbackData(RxMessage.ExtId,RxMessage.Data);
	if(RxMessage.ExtId==CAN1_ID) 
	{ 
//		for(i=0;i<8;i++)
//		{
//			CAN2_RX_data[i] = RxMessage.Data[i]; 
//		}
		ValueData.val.value_L = RxMessage.Data[0];
		ValueData.val.value_H = RxMessage.Data[1];
//		LED4_ON;
	}	

	can2_rec_flag = 1;
	CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);  /* 清除挂起中断 */

}
//CAN2发送函数
void can2_tx(uint32_t ID, uint8_t* Data)
{
		
	unsigned char i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId=00;	//标准标识符为0x00
	TxMessage.ExtId=ID; //扩展标识符0x0000
	TxMessage.IDE = CAN_ID_EXT;//使用标准标识符
	TxMessage.RTR = CAN_RTR_DATA; /* 设置为数据帧 */
	TxMessage.DLC = 8;            /* 数据长度, can报文规定最大的数据长度为8字节 */
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = (uint8_t)Data[i];
	}
	transmit_mailbox = CAN_Transmit(CAN2,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
	/*while((CAN_TransmitStatus(CAN2, transmit_mailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
	{
		i ++;
	}*/
}
