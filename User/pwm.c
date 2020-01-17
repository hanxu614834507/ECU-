#include "main.h"	


void TIM1_PWM_Init(void)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); //TIM1ʱ��ʹ�� 	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��		
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //��ʼ��PE13 PE13
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); //GPIOF9����Ϊ��ʱ��14
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); //GPIOF9����Ϊ��ʱ��14
	  
	TIM_TimeBaseStructure.TIM_Prescaler=168;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=500;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
	
	//��ʼ��TIM1 Channel3 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//OC���ʹ��
  //TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//OC�������ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;//�Զ���װ��ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//����������� 
  //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;//���û����������
  //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC3
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM1 Channel4 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//OC���ʹ��
	//TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//OC�������ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;//�Զ���װ��ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//����������� 
  //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;//���û����������
  //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 OC4
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR1�ϵ�Ԥװ�ؼĴ���
 
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);	//PWM���ʹ�ܣ�TIM1��TIM8һ��Ҫ��								  
}  


