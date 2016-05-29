#include "configuration.h"

GPIO_InitTypeDef GPIO_InitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;
USART_InitTypeDef USART_InitStructure; 
NVIC_InitTypeDef NVIC_InitStructure;
DMA_InitTypeDef DMA_Structure; 
EXTI_InitTypeDef EXTI_InitStructure;

TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
TIM_OCInitTypeDef  TIM1_OCInitStructure;//ͨ�������ʼ���ṹ
TIM_ICInitTypeDef   TIM1_ICInitStructure;

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;//ͨ�������ʼ���ṹ
TIM_ICInitTypeDef   TIM_ICInitStructure;




void system_clk_set(void)
{ 
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();
 
    RCC_HSEConfig(RCC_HSE_ON );   //���ⲿʱ��
 
    HSEStartUpStatus = RCC_WaitForHSEStartUp();  //�ȴ��ⲿʱ�Ӵ����ȶ�
 
  if(HSEStartUpStatus == SUCCESS)     
  {
    FLASH_SetLatency(FLASH_Latency_5);   
    FLASH_PrefetchBufferCmd(ENABLE);       //flashʱ�ӵ��������  
    RCC_PLLCmd(DISABLE);  //����PLL֮ǰ��Ҫ�ر�PLL
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   //HCLK��Ƶ
    RCC_PCLK2Config(RCC_HCLK_Div1);   //PCLK2��Ƶ
    RCC_PCLK1Config(RCC_HCLK_Div4);    //PCLK1��Ƶ
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);    //sysclk = 168MHZ  ,,���㹫ʽ�μ������ֲ�
    RCC_PLLCmd(ENABLE); //ʹ��PLL
 
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){      //�ȴ�����ֱ��PLLʹ�����
    
    }
 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  //ѡ��PLLʱ��Ϊϵͳʱ��
 
    while(RCC_GetSYSCLKSource() != 0x08)       //�ȴ�����PLLʱ�����ó�ϵͳʱ�����
       { 
       }
     }
    #if(__FPU_PRESENT == 1)&&(__FPU_USED == 1)
			SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  //����FPU
		#endif

}



void rcc_configuration()
{
	RCC_DeInit();			//��ʼ��Ϊȱʡֵ
	
//	SystemInit();//Դ��system_stm32f10x.c�ļ�,ֻ��Ҫ���ô˺���,������RCC������.
	system_clk_set();
	//RCC_GetClocksFreq(&RCC_ClockFreq);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
                                                            //�����¼����ƼĴ���/�ⲿ�жϿ��ƼĴ���/��ӳ��ʱ���뿪��AFIOʱ�ӣ������ܽŵ�Ĭ�����蹦�ܲ�����Ҫ��AFIOʱ�ӣ�û����ӳ�䣩
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	 //ʹ��USART1ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//ʹ��USART2ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//ʹ��USART3ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);   //ʹ��UART5ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); 		
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI,ENABLE);  //ʹ������ͷ�ӿ�DCMIʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);		
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //ʹ��DMA2ʱ��
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);   //ʹ��FMCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);   //ʹ��syscfgʱ�ӣ������ⲿ�ж�
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);   
	
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_3);   //����MCO1,��Ϊ����ͷ��ʱ��XCLK DIV3���18M

}


void GPIO_Configuration(uint16_t GPIO_Pin,
                        GPIOMode_TypeDef  GPIO_Mode,
                        GPIOSpeed_TypeDef GPIO_Speed,
                        GPIOOType_TypeDef GPIO_OType,
                        GPIOPuPd_TypeDef GPIO_PuPd,
                        GPIO_TypeDef* GPIOx)
{
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;                       //�ܽź� 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;                     //���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
    
    GPIO_InitStructure.GPIO_OType = GPIO_OType;                 //407   GPIO_OType_PP Ϊ����   GPIO_OType_OD Ϊ��©
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;										//407
    GPIO_Init(GPIOx, &GPIO_InitStructure);                        //�ܽ���� 
}

void gpio_config(void)
{
    u32 gpio_temp;


	//δʹ�� ���ô���1  PB6 TX  PB7 RX
//	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	GPIO_Configuration(GPIO_Pin_6,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);   //USART1 TX
	GPIO_Configuration(GPIO_Pin_7,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB); //USART1 RX
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);   
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);  
	
	//δʹ�� ���ô���2   PA2 TX PA3 RX
	//GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
	GPIO_Configuration(GPIO_Pin_2,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);   //USART2 TX
	GPIO_Configuration(GPIO_Pin_3,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); //USART2 RX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);   
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  
	//����3    �жϽ�����������  PB10��TX  PB11��RX
    GPIO_Configuration(GPIO_Pin_10,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);   //USART3 TX
	GPIO_Configuration(GPIO_Pin_11,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB); //USART3 RX
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);   
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  
	
		//����5
	GPIO_Configuration(GPIO_Pin_2,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);   //USART5 TX
	GPIO_Configuration(GPIO_Pin_12,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOC); //USART5 RX
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);   
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);  
	
		/************************************************************************************
	                 		��ʾ�� 
#define OLED_D0_PORT	GPIOE
#define OLED_D0_Pin		GPIO_Pin_8
#define OLED_D1_PORT	GPIOE
#define OLED_D1_Pin		GPIO_Pin_7

#define OLED_CS_PORT	GPIOG
#define OLED_CS_Pin		GPIO_Pin_0
#define OLED_DC_PORT	GPIOG
#define OLED_DC_Pin		GPIO_Pin_1
	*************************************************************************************/
//	GPIO_Configuration(GPIO_Pin_0,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOG);
//	GPIO_Configuration(GPIO_Pin_1,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOG);
//	GPIO_Configuration(GPIO_Pin_8,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOE);
//	GPIO_Configuration(GPIO_Pin_7,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOE);
	
	//����ֵ����  DO:PG5  CLK:PG4 CS:PG3
//	GPIO_Configuration(GPIO_Pin_5,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIOG);  //DO����
//	GPIO_Configuration(GPIO_Pin_4,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOG);
//	GPIO_Configuration(GPIO_Pin_3,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOG);
	
	//��ʱ�� ��������   ����TIM4 PD12 PD13   TIM5 PA1 PA0
//	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
//	GPIO_Configuration(GPIO_Pin_12|GPIO_Pin_13,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);  //TIM4��remap
//	GPIO_Configuration(GPIO_Pin_0|GPIO_Pin_1,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); 
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM4);
//	
	//��ʱ�� �������� TIM4 PD12  PD13
	GPIO_Configuration(GPIO_Pin_12 | GPIO_Pin_13,GPIO_Mode_AF,GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); 	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 	
		
	//��ʱ��TIM3���壬�������
	GPIO_Configuration(GPIO_Pin_6,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); 	
	GPIO_Configuration(GPIO_Pin_7,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); 
	GPIO_Configuration(GPIO_Pin_5,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); 
/*****************************����ͷOV7725***************************************/	
	//PA8:��Ϊ���벶�񣬽�����ͷ��PCLK,�½��زɼ�����
	GPIO_Configuration(GPIO_Pin_8,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	//PD3:�½����жϣ���Ϊ���жϣ�������ͷVSYNC
	GPIO_Configuration(GPIO_Pin_3,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD); 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource3);
	//����ͷIIC1 PB8:SCL  PB9:SDA
	GPIO_Configuration(GPIO_Pin_8|GPIO_Pin_9,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_OD,GPIO_PuPd_UP,GPIOB); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
	//	����ͷ����
	GPIO_Configuration(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_IN,GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOG); 
	GPIO_Configuration(GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_IN,GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOG);  
/***********************************************************************************/
    //������������D0 D1:PD14 PD15    D2  D3:PD0  PD1   D4 ~ D12 :PE7 ~ PE15 
    //D13 D14 D15:PD8  PD9  PD10
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);   //D0
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);    //D1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);     //D2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);     //D3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);     //D4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);     //D5
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);     //D6
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);     //D7
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);     //D8
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);     //D9
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);      //D10
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);      //D11
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);      //D12
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);      //D13
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);       //D14
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);      //D15

    gpio_temp = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|
                GPIO_Pin_14|GPIO_Pin_15;
    GPIO_Configuration(gpio_temp,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);
    gpio_temp = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|
                GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_Configuration(gpio_temp,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOE);

    //LCD_CS(FSMC_NE4):PG12 RD(FSMC_NOE):PD4 WR(FSMC_NWE):PD5   RS(FSMC_A6):PF12
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);  
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);  
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);  
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);  


  GPIO_Configuration(GPIO_Pin_12,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOG);
  GPIO_Configuration(GPIO_Pin_4|GPIO_Pin_5,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);
  GPIO_Configuration(GPIO_Pin_12,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOF);
	//LCD��������  PB15
	GPIO_Configuration(GPIO_Pin_15,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);
	

/***************************TFTLCD��������***************************************/
    
		//��ɫ���������͹�紫����
		GPIO_Configuration(GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOF); 
		GPIO_Configuration(GPIO_Pin_1,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOF); 
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource6);  //�����ⲿ�ж���
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource7);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource9);
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource1);
        GPIO_Configuration(GPIO_Pin_13|GPIO_Pin_15,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOG);
        GPIO_Configuration(GPIO_Pin_6,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD); 
        GPIO_Configuration(GPIO_Pin_10,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOC); 
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);

/********************************************************************************/
	
	//SDIO  PC8  PC9   PC10  PC11   PC12  PD2
}

void exti_config(){
    EXTI_InitTypeDef exti_init_structure;
    
    
    //����ͷ���ж�
    exti_init_structure.EXTI_Line = EXTI_Line3;
    exti_init_structure.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_structure.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init_structure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_structure); 
    //exti_init_structure.EXTI_Line = EXTI_Line6;
    //exti_init_structure.EXTI_Mode = EXTI_Mode_Interrupt;
    //exti_init_structure.EXTI_Trigger = EXTI_Trigger_Falling;
    //exti_init_structure.EXTI_LineCmd = ENABLE;
    //EXTI_Init(&exti_init_structure); 
    
    exti_init_structure.EXTI_Line = EXTI_Line9;
    exti_init_structure.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_structure.EXTI_Trigger = EXTI_Trigger_Rising;
    exti_init_structure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_structure); 

    exti_init_structure.EXTI_Line = EXTI_Line10;
    exti_init_structure.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_structure.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init_structure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_structure); 
    
    //��ɫ�������ж�
    exti_init_structure.EXTI_Line = EXTI_Line1;
    exti_init_structure.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_structure.EXTI_Trigger = EXTI_Trigger_Rising;
    exti_init_structure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_structure); 
    
    //exti_init_structure.EXTI_Line = EXTI_Line8;
    //exti_init_structure.EXTI_Mode = EXTI_Mode_Interrupt;
    //exti_init_structure.EXTI_Trigger = EXTI_Trigger_Falling;
    //exti_init_structure.EXTI_LineCmd = ENABLE;
    //EXTI_Init(&exti_init_structure); 
}


void TIM2_Configuration()        //0.005ms ��ʱ
{
	/*TIM2*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 9999;  
  TIM_TimeBaseStructure.TIM_Period = 16800-1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //������ʱ�ӷָ�
    //��ʱ����ʱ������Ϊ (4+1)*(71+1)/(72*10^6)=0.005ms
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// ����  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);	
}


void tim3_config(){
	TIM_TimeBaseInitTypeDef timer_base_stru;
	TIM_OCInitTypeDef timer_oc_stru;
	u16 period;
	u16 cmp;
	period = 1000000/100 - 1;
	cmp = period*3100/10000;
	
	
	timer_base_stru.TIM_Prescaler = 41;  //42/(41 + 1)=1M
	timer_base_stru.TIM_Period = 9999;  //freq = 1000000/(TIM_Period+1)   TIM_Period = 1000000/freq - 1
	timer_base_stru.TIM_ClockDivision = 0;
	timer_base_stru.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&timer_base_stru);
	
	
	timer_oc_stru.TIM_OCMode = TIM_OCMode_PWM1; 
	timer_oc_stru.TIM_OCPolarity = TIM_OCPolarity_High;
	timer_oc_stru.TIM_OutputState = TIM_OutputState_Enable;
	timer_oc_stru.TIM_Pulse = cmp;
	TIM_OC2Init(TIM3,&timer_oc_stru);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	

	
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}
	

void tim4_config()   //��������
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_DeInit(TIM4);
   
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = 29999;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //������ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// ����  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 6;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);

  TIM4->CNT=4000;

  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM4, ENABLE);	
}


void tim1_config(){
	TIM_TimeBaseInitTypeDef timer_base_stru;
	TIM_OCInitTypeDef timer_oc_stru;
	
	u16 cmp = 5000;
	
	timer_base_stru.TIM_Prescaler = 167;  //168/(167+1)=1M
	timer_base_stru.TIM_Period = 999;  //freq = 1000000/(TIM_Period+1)   TIM_Period = 1000000/freq - 1
	timer_base_stru.TIM_ClockDivision = 0;
	timer_base_stru.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1,&timer_base_stru);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 5;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_ICInit(TIM1,&TIM_ICInitStructure);
      TIM1->DIER |= 1<<9;   //ʹ��DMA����
    TIM_Cmd(TIM1,ENABLE);

	
}





void USART_Configuration(void)
{
	//����1
	USART_InitTypeDef USART_InitStructure; 
	USART_InitStructure.USART_BaudRate = 115200; 	   //���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //һ��֡�д��������λ�����ֳ�Ϊ8λ���ݸ�ʽ��
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���ͺͽ���ģʽ
	USART_Init(USART1, &USART_InitStructure);	 //��ʼ������	
	USART_Cmd(USART1, ENABLE);	  //ʹ�ܴ���
	//USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);
	
	//����2
	USART_InitStructure.USART_BaudRate = 115200; 	   //���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //һ��֡�д��������λ�����ֳ�Ϊ8λ���ݸ�ʽ��
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���ͺͽ���ģʽ
	USART_Init(USART2, &USART_InitStructure);	 //��ʼ������	
	//USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);//����DMA
	USART_Cmd(USART2, ENABLE);	  //ʹ�ܴ���
	//USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);
	
	//����3 ����
	USART_InitStructure.USART_BaudRate = 115200; 	   //���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //һ��֡�д��������λ�����ֳ�Ϊ8λ���ݸ�ʽ��
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���ͺͽ���ģʽ
	USART_Init(USART3, &USART_InitStructure);	 //��ʼ������	
	//USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);//����DMA
	USART_Cmd(USART3, ENABLE);	  //ʹ�ܴ���
	//USART_ITConfig(USART3,USART_IT_RXNE, ENABLE);
		//����5
	USART_InitStructure.USART_BaudRate = 115200; 	   //���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //һ��֡�д��������λ�����ֳ�Ϊ8λ���ݸ�ʽ��
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//����żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���ͺͽ���ģʽ
	USART_Init(UART5, &USART_InitStructure);	 //��ʼ������	
	USART_Cmd(UART5, ENABLE);	  //ʹ�ܴ���
	USART_ITConfig(UART5,USART_IT_RXNE, ENABLE);
	

}

void NVIC_Configuration()
{
    //USART3 �ж�
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel=UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
		
		//�����ⲿ�ж� 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;	 //ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //��ռ���ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //�����ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);				 //����NVIC_InitStructure��ָ���Ĳ�����ʼ������NVIC
	
	//TIM2�ж�
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		//����ͷ�ӿ�DCMI�ж�
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=DCMI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		//����ͷDMA2_Stream1�ж�
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		//��ɫ���������͹�紫����
		
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		//����ͷ���ж�
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //�����ж�
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}



void SPI_Configuration(void)  //�ֱ�
{
	SPI_InitTypeDef SPI_Structure;
	SPI_Structure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//��Ƶ������ΪCPUʱ��Ƶ��
	SPI_Structure.SPI_CPHA = SPI_CPHA_2Edge;							//�ڶ������ؿ�ʼ����
	SPI_Structure.SPI_CPOL = SPI_CPOL_High;								//ʱ�ӿ���ʱΪ�ߵ�ƽ
	SPI_Structure.SPI_CRCPolynomial = 7;								//CRCУ�����ʽ
	SPI_Structure.SPI_DataSize = SPI_DataSize_8b;						//8λһ����
	SPI_Structure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;		//ȫ˫��
	SPI_Structure.SPI_FirstBit = SPI_FirstBit_LSB;						//С��ģʽ
	SPI_Structure.SPI_Mode = SPI_Mode_Master;							//���豸
	SPI_Structure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2, &SPI_Structure);
	//SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
}


void iic_configuration(void){  //iic��ʼ��
	
	//I2C1 ��ʼ�� ���ڣ�������MPU6050
	I2C_InitTypeDef i2c_init_stru;
//	I2C_DeInit(I2C1);
//	i2c_init_stru.I2C_ClockSpeed = 1000;    //1kHz
//	i2c_init_stru.I2C_Mode = I2C_Mode_I2C;
//	i2c_init_stru.I2C_DutyCycle = I2C_DutyCycle_2;   //�����ʱ������
//	i2c_init_stru.I2C_Ack = I2C_Ack_Enable;   //ʹ��Ӧ��
//	i2c_init_stru.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  //Ӧ��7λ��ַ
//	I2C_Init(I2C1,&i2c_init_stru);
//	I2C_ITConfig(I2C1,I2C_IT_EVT,ENABLE);
//	I2C_Cmd(I2C1,ENABLE);
	
	//I2C2 ��ʼ��  ���ڣ�����ͷ
	I2C_DeInit(I2C2);
	i2c_init_stru.I2C_ClockSpeed = 100;    //1kHz
	i2c_init_stru.I2C_Mode = I2C_Mode_I2C;
	i2c_init_stru.I2C_DutyCycle = I2C_DutyCycle_2;   //�����ʱ������
	i2c_init_stru.I2C_Ack = I2C_Ack_Enable;   //ʹ��Ӧ��
	i2c_init_stru.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  //Ӧ��7λ��ַ
	I2C_Init(I2C2,&i2c_init_stru);
	I2C_ITConfig(I2C2,I2C_IT_EVT,ENABLE);
	I2C_Cmd(I2C2,ENABLE);
}



//void dma_config(){
//    DMA_InitTypeDef dma_init_stru;
//    
//    //��������ͷcamera DMA  
//    DMA_DeInit(DMA2_Stream1);   
//    dma_init_stru.DMA_BufferSize = 20000;  //��ʱ��ͬ����˼
//    dma_init_stru.DMA_Channel = DMA_Channel_6;  //!!!!ͨ��1���ܵ�ֻд����1�����򣬾����Է��֣���CR�Ĵ������֮��ENλ��1����������ļĴ����޷�������
//    dma_init_stru.DMA_DIR = DMA_DIR_PeripheralToMemory; //���ݷ���Ϊ���赽�ڴ�
//    dma_init_stru.DMA_FIFOMode = DMA_FIFOMode_Enable;   //DMA_FIF��OMode_Enableʹ��FIFOģʽ
//    dma_init_stru.DMA_FIFOThreshold = DMA_FIFOThreshold_Full; //����FIFO��ֵΪ��
//    dma_init_stru.DMA_Memory0BaseAddr = (uint32_t)(g_camera_image);//(g_camera_image) ((u32)(0x60000000|0x0C000000|0x00000080));  //���ô洢���ݵ��ڴ����ַ
//    dma_init_stru.DMA_MemoryBurst = DMA_MemoryBurst_Single;  //ÿ�δ���һ�����ݵ�Ԫ
//    dma_init_stru.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte ;  //���ִ���
//    dma_init_stru.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ�ÿ�ν��յ�����֮�󣬵�ַ��������
//    dma_init_stru.DMA_Mode = DMA_Mode_Circular  ;  //DMA_Mode_Circular��ͣ�ش���  DMA_Mode_Normal  
//    dma_init_stru.DMA_PeripheralBaseAddr = (u32)(&(GPIOG->IDR));  //DCMI_DR_ADDR�����ַΪDCMI�����ݼĴ�����ַ
//    dma_init_stru.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //һ�δ�һ�����ݵ�Ԫ
//    dma_init_stru.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte; //�������ݵ�Ԫ��СΪ��
//    dma_init_stru.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //��ʹ�������ַ����
//    dma_init_stru.DMA_Priority = DMA_Priority_High;  //DMA��������ȼ�Ϊ��
//    DMA_Init(DMA2_Stream1,&dma_init_stru);
//	//	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_FE,ENABLE);
//	//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
//    DMA_Cmd(DMA2_Stream1,ENABLE);
//
//}

void fsmc_config(){
    FSMC_NORSRAMInitTypeDef fsmc_init_stru;
    FSMC_NORSRAMTimingInitTypeDef readWriteTiming,writeTiming;



/**********************TFTLCD��FSMC����***********************/
   /* FMC_Bank1_NORSRAM4 configuration */
  readWriteTiming.FSMC_AddressSetupTime = 0XF;	 //��ַ����ʱ�䣨ADDSET��Ϊ16��HCLK 1/168M=6ns*16=96ns	
  readWriteTiming.FSMC_AddressHoldTime = 0x00;	 //��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�	
  readWriteTiming.FSMC_DataSetupTime = 60;			//���ݱ���ʱ��Ϊ60��HCLK	=6*60=360ns
  readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
  readWriteTiming.FSMC_CLKDivision = 0x00;
  readWriteTiming.FSMC_DataLatency = 0x00;
  readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //ģʽA 
    

	writeTiming.FSMC_AddressSetupTime =9;	      //��ַ����ʱ�䣨ADDSET��Ϊ9��HCLK =54ns 
  writeTiming.FSMC_AddressHoldTime = 0x00;	 //��ַ����ʱ�䣨A		
  writeTiming.FSMC_DataSetupTime = 8;		 //���ݱ���ʱ��Ϊ6ns*9��HCLK=54ns
  writeTiming.FSMC_BusTurnAroundDuration = 0x00;
  writeTiming.FSMC_CLKDivision = 0x00;
  writeTiming.FSMC_DataLatency = 0x00;
  writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //ģʽA 
   /* Color LCD configuration ------------------------------------
      LCD configured as follow:
         - Data/Address MUX = Disable
         - Memory Type = SRAM
         - Data Width = 16bit
         - Write Operation = Enable
         - Extended Mode = Enable
         - Asynchronous Wait = Disable */
 
  fsmc_init_stru.FSMC_Bank = FSMC_Bank1_NORSRAM4;//  ��������ʹ��NE4 ��Ҳ�Ͷ�ӦBTCR[6],[7]��
  fsmc_init_stru.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // ���������ݵ�ַ
  fsmc_init_stru.FSMC_MemoryType =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM   
  fsmc_init_stru.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//�洢�����ݿ��Ϊ16bit   
  fsmc_init_stru.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
  fsmc_init_stru.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	fsmc_init_stru.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
  fsmc_init_stru.FSMC_WrapMode = FSMC_WrapMode_Disable;   
  fsmc_init_stru.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
  fsmc_init_stru.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	//  �洢��дʹ��
  fsmc_init_stru.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
  fsmc_init_stru.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; // ��дʹ�ò�ͬ��ʱ��
  fsmc_init_stru.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
  fsmc_init_stru.FSMC_ReadWriteTimingStruct = &readWriteTiming; //��дʱ��
  fsmc_init_stru.FSMC_WriteTimingStruct = &writeTiming;  //дʱ��;
 
   FSMC_NORSRAMInit(&fsmc_init_stru);   
 
   /* Enable FMC NOR/SRAM Bank3 */
   FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);

}











