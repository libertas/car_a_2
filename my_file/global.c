#include "global.h"


int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {}

    /* e.g. write a character to the USART */
    USART_SendData(USART3, (uint8_t) ch);

    return ch;
}

int fgetc(FILE *fp)
{
	int ch = 0;
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
    {
    }

    ch = (int)USART3->DR & 0xFF;
	
    //putchar(ch); //回显
	
	return ch;
}

void uprintf(USART_TypeDef* USARTx, char *fmt, ...)
{

	char buffer[CMD_BUFFER_LEN+1];  // CMD_BUFFER_LEN长度自己定义吧
	u8 i = 0;
	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, CMD_BUFFER_LEN+1, fmt, arg_ptr);
	USART_ClearFlag(USARTx,USART_FLAG_TXE);
	while ((i < CMD_BUFFER_LEN) && buffer[i])
	{
		if(buffer[i] == '\n'){
			USART_SendData(USARTx,(u8)buffer[i++]);
			while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); 
			USART_SendData(USARTx,(u8)'\r');
		}else{
	    USART_SendData(USARTx, (u8) buffer[i++]);
		}
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); 
	}
	va_end(arg_ptr);
} 



void delay_us(uint32_t us)	   //1us的延迟，可能偏小
{
	char i = 0;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	if (us > 1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		  	 
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = us-1;
		
		TIM_TimeBaseStructure.TIM_Prescaler = 41;
		
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);				
		TIM_ClearFlag(TIM7, TIM_FLAG_Update);  		

		while(i<1)
		{
			TIM_Cmd(TIM7, ENABLE);			  //当内层嵌套的Delay结束时，会使得外层Delay卡死，故将开关写在循环里面。

			if (TIM_GetFlagStatus(TIM7, TIM_FLAG_Update) == SET)
			{
				i++;
				TIM_ClearFlag(TIM7, TIM_FLAG_Update);
			}
		}

		TIM_ClearFlag(TIM7, TIM_FLAG_Update);
		TIM_Cmd(TIM7,DISABLE);
	}
}

void delay_ms(uint16_t ms)	   //0.1ms的延迟，可能偏小
{
	char i = 0;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	if (ms > 1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		  	 
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period = ms-1;
		
		TIM_TimeBaseStructure.TIM_Prescaler = 41999;
		
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);				
		TIM_ClearFlag(TIM7, TIM_FLAG_Update);  		

		while(i<1)
		{
			TIM_Cmd(TIM7, ENABLE);			  //当内层嵌套的Delay结束时，会使得外层Delay卡死，故将开关写在循环里面。

			if (TIM_GetFlagStatus(TIM7, TIM_FLAG_Update) == SET)
			{
				i++;
				TIM_ClearFlag(TIM7, TIM_FLAG_Update);
			}
				
		}

		TIM_ClearFlag(TIM7, TIM_FLAG_Update);
		TIM_Cmd(TIM7,DISABLE);
	}
}


void tim3_pwm_set(u16 freq,u16 duty){
	TIM_OCInitTypeDef timer_oc_stru;
	u16 cmp,period;
	period = 1000000/freq - 1;
	cmp = period*duty/10000;
//	TIM_SetCounter(TIM3,period);
	TIM_SetAutoreload(TIM3,period);
	TIM_SetCompare1(TIM3,cmp);
	TIM_SetCompare2(TIM3,cmp);
	TIM_SetCompare3(TIM3,cmp);
	TIM_SetCompare4(TIM3,cmp);
	
}

void exti_color_disable(){
		NVIC_InitTypeDef NVIC_InitStructure;
	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}
void exti_color_enable(){
		NVIC_InitTypeDef NVIC_InitStructure;
	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);}
