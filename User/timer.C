
#include  "main.h"

void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

//定时器2配置
//定时器溢出时间计算方法:Tout=((4999+1)*(8399+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz

void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM2时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = 4200-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 200-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器2更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器2
}

//定时器2中断函数
//每隔	time（ms）中断一次
void TIM2_IRQHandler(void)
{
	static uint16_t time_count = 0;
// 	CLI();			//关闭总中断
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
		time_count++;
		GPIO_ToggleBits(LED4);
		
		can1_tx(DRIVE_FdB_ID, ((uint8_t*)&MotorStateFdB));	
	}
// 	SEI();			//打开总中断	
}

