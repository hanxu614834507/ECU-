
#include  "main.h"

void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

//��ʱ��2����
//��ʱ�����ʱ����㷽��:Tout=((4999+1)*(8399+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz

void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM2ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period = 4200-1; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = 200-1;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //������ʱ��2�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
}

//��ʱ��2�жϺ���
//ÿ��	time��ms���ж�һ��
void TIM2_IRQHandler(void)
{
	static uint16_t time_count = 0;
// 	CLI();			//�ر����ж�
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
		time_count++;
		GPIO_ToggleBits(LED4);
		
		can1_tx(DRIVE_FdB_ID, ((uint8_t*)&MotorStateFdB));	
	}
// 	SEI();			//�����ж�	
}
