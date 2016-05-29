#include "configuration.h"

GPIO_InitTypeDef GPIO_InitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;
USART_InitTypeDef USART_InitStructure; 
NVIC_InitTypeDef NVIC_InitStructure;
DMA_InitTypeDef DMA_Structure; 
EXTI_InitTypeDef EXTI_InitStructure;

TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
TIM_OCInitTypeDef  TIM1_OCInitStructure;//通道输出初始化结构
TIM_ICInitTypeDef   TIM1_ICInitStructure;

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;//通道输出初始化结构
TIM_ICInitTypeDef   TIM_ICInitStructure;




void system_clk_set(void)
{ 
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();
 
    RCC_HSEConfig(RCC_HSE_ON );   //打开外部时钟
 
    HSEStartUpStatus = RCC_WaitForHSEStartUp();  //等待外部时钟打开至稳定
 
  if(HSEStartUpStatus == SUCCESS)     
  {
    FLASH_SetLatency(FLASH_Latency_5);   
    FLASH_PrefetchBufferCmd(ENABLE);       //flash时钟的相关配置  
    RCC_PLLCmd(DISABLE);  //配置PLL之前需要关闭PLL
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   //HCLK分频
    RCC_PCLK2Config(RCC_HCLK_Div1);   //PCLK2分频
    RCC_PCLK1Config(RCC_HCLK_Div4);    //PCLK1分频
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);    //sysclk = 168MHZ  ,,计算公式参见数据手册
    RCC_PLLCmd(ENABLE); //使能PLL
 
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){      //等待，，直到PLL使能完毕
    
    }
 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  //选择PLL时钟为系统时钟
 
    while(RCC_GetSYSCLKSource() != 0x08)       //等待，至PLL时钟设置成系统时钟完毕
       { 
       }
     }
    #if(__FPU_PRESENT == 1)&&(__FPU_USED == 1)
			SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  //开启FPU
		#endif

}



void rcc_configuration()
{
	RCC_DeInit();			//初始化为缺省值
	
//	SystemInit();//源自system_stm32f10x.c文件,只需要调用此函数,则可完成RCC的配置.
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
                                                            //配置事件控制寄存器/外部中断控制寄存器/重映射时必须开启AFIO时钟，而开管脚的默认外设功能并不需要开AFIO时钟（没有重映射）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	 //使能USART1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//使能USART2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//使能USART3时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);   //使能UART5时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); 		
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI,ENABLE);  //使能摄像头接口DCMI时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);		
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   //使能DMA2时钟
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);   //使能FMC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);   //使能syscfg时钟，用于外部中断
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);   
	
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_3);   //配置MCO1,作为摄像头的时钟XCLK DIV3大概18M

}


void GPIO_Configuration(uint16_t GPIO_Pin,
                        GPIOMode_TypeDef  GPIO_Mode,
                        GPIOSpeed_TypeDef GPIO_Speed,
                        GPIOOType_TypeDef GPIO_OType,
                        GPIOPuPd_TypeDef GPIO_PuPd,
                        GPIO_TypeDef* GPIOx)
{
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;                       //管脚号 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;                     //输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
    
    GPIO_InitStructure.GPIO_OType = GPIO_OType;                 //407   GPIO_OType_PP 为推挽   GPIO_OType_OD 为开漏
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;										//407
    GPIO_Init(GPIOx, &GPIO_InitStructure);                        //管脚组别 
}

void gpio_config(void)
{
    u32 gpio_temp;


	//未使用 复用串口1  PB6 TX  PB7 RX
//	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	GPIO_Configuration(GPIO_Pin_6,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);   //USART1 TX
	GPIO_Configuration(GPIO_Pin_7,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB); //USART1 RX
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);   
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);  
	
	//未使用 复用串口2   PA2 TX PA3 RX
	//GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
	GPIO_Configuration(GPIO_Pin_2,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);   //USART2 TX
	GPIO_Configuration(GPIO_Pin_3,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); //USART2 RX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);   
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  
	//串口3    中断接受蓝牙数据  PB10：TX  PB11：RX
    GPIO_Configuration(GPIO_Pin_10,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);   //USART3 TX
	GPIO_Configuration(GPIO_Pin_11,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB); //USART3 RX
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);   
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  
	
		//串口5
	GPIO_Configuration(GPIO_Pin_2,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);   //USART5 TX
	GPIO_Configuration(GPIO_Pin_12,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOC); //USART5 RX
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);   
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);  
	
		/************************************************************************************
	                 		显示屏 
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
	
	//绝对值码盘  DO:PG5  CLK:PG4 CS:PG3
//	GPIO_Configuration(GPIO_Pin_5,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIOG);  //DO浮空
//	GPIO_Configuration(GPIO_Pin_4,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOG);
//	GPIO_Configuration(GPIO_Pin_3,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIOG);
	
	//定时器 增量码盘   复用TIM4 PD12 PD13   TIM5 PA1 PA0
//	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
//	GPIO_Configuration(GPIO_Pin_12|GPIO_Pin_13,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);  //TIM4的remap
//	GPIO_Configuration(GPIO_Pin_0|GPIO_Pin_1,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); 
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM4);
//	
	//定时器 增量码盘 TIM4 PD12  PD13
	GPIO_Configuration(GPIO_Pin_12 | GPIO_Pin_13,GPIO_Mode_AF,GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); 	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 	
		
	//定时器TIM3定义，用作舵机
	GPIO_Configuration(GPIO_Pin_6,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); 	
	GPIO_Configuration(GPIO_Pin_7,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); 
	GPIO_Configuration(GPIO_Pin_5,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); 
/*****************************摄像头OV7725***************************************/	
	//PA8:作为输入捕获，接摄像头的PCLK,下降沿采集数据
	GPIO_Configuration(GPIO_Pin_8,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	//PD3:下降沿中断，作为场中断，接摄像头VSYNC
	GPIO_Configuration(GPIO_Pin_3,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD); 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource3);
	//摄像头IIC1 PB8:SCL  PB9:SDA
	GPIO_Configuration(GPIO_Pin_8|GPIO_Pin_9,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_OD,GPIO_PuPd_UP,GPIOB); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
	//	摄像头数据
	GPIO_Configuration(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_IN,GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOG); 
	GPIO_Configuration(GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_IN,GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOG);  
/***********************************************************************************/
    //数据引脚配置D0 D1:PD14 PD15    D2  D3:PD0  PD1   D4 ~ D12 :PE7 ~ PE15 
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
	//LCD背光设置  PB15
	GPIO_Configuration(GPIO_Pin_15,GPIO_Mode_OUT, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);
	

/***************************TFTLCD引脚配置***************************************/
    
		//颜色传感器，和光电传感器
		GPIO_Configuration(GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOF); 
		GPIO_Configuration(GPIO_Pin_1,GPIO_Mode_IN, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOF); 
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource6);  //连接外部中断线
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
    
    
    //摄像头场中断
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
    
    //颜色传感器中断
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


void TIM2_Configuration()        //0.005ms 定时
{
	/*TIM2*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 9999;  
  TIM_TimeBaseStructure.TIM_Period = 16800-1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //设置了时钟分割
    //计时器的时钟周期为 (4+1)*(71+1)/(72*10^6)=0.005ms
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上  
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
	

void tim4_config()   //增量码盘
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_DeInit(TIM4);
   
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = 29999;  
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //设置了时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上  
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
      TIM1->DIER |= 1<<9;   //使能DMA请求
    TIM_Cmd(TIM1,ENABLE);

	
}





void USART_Configuration(void)
{
	//串口1
	USART_InitTypeDef USART_InitStructure; 
	USART_InitStructure.USART_BaudRate = 115200; 	   //设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //一个帧中传输的数据位数（字长为8位数据格式）
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送和接受模式
	USART_Init(USART1, &USART_InitStructure);	 //初始化串口	
	USART_Cmd(USART1, ENABLE);	  //使能串口
	//USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);
	
	//串口2
	USART_InitStructure.USART_BaudRate = 115200; 	   //设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //一个帧中传输的数据位数（字长为8位数据格式）
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送和接受模式
	USART_Init(USART2, &USART_InitStructure);	 //初始化串口	
	//USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);//允许DMA
	USART_Cmd(USART2, ENABLE);	  //使能串口
	//USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);
	
	//串口3 蓝牙
	USART_InitStructure.USART_BaudRate = 115200; 	   //设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //一个帧中传输的数据位数（字长为8位数据格式）
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送和接受模式
	USART_Init(USART3, &USART_InitStructure);	 //初始化串口	
	//USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);//允许DMA
	USART_Cmd(USART3, ENABLE);	  //使能串口
	//USART_ITConfig(USART3,USART_IT_RXNE, ENABLE);
		//串口5
	USART_InitStructure.USART_BaudRate = 115200; 	   //设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;    //一个帧中传输的数据位数（字长为8位数据格式）
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 	   //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; 	//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送和接受模式
	USART_Init(UART5, &USART_InitStructure);	 //初始化串口	
	USART_Cmd(UART5, ENABLE);	  //使能串口
	USART_ITConfig(UART5,USART_IT_RXNE, ENABLE);
	

}

void NVIC_Configuration()
{
    //USART3 中断
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel=UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
		
		//配置外部中断 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;	 //使能按键所在的外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //抢占优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);				 //根据NVIC_InitStructure中指定的参数初始化外设NVIC
	
	//TIM2中断
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		//摄像头接口DCMI中断
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=DCMI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		//摄像头DMA2_Stream1中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		//颜色传感器，和光电传感器
		
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
		
		//摄像头场中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //按键中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
}



void SPI_Configuration(void)  //手柄
{
	SPI_InitTypeDef SPI_Structure;
	SPI_Structure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//分频，分子为CPU时钟频率
	SPI_Structure.SPI_CPHA = SPI_CPHA_2Edge;							//第二个边沿开始采样
	SPI_Structure.SPI_CPOL = SPI_CPOL_High;								//时钟空闲时为高电平
	SPI_Structure.SPI_CRCPolynomial = 7;								//CRC校验多项式
	SPI_Structure.SPI_DataSize = SPI_DataSize_8b;						//8位一个包
	SPI_Structure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;		//全双工
	SPI_Structure.SPI_FirstBit = SPI_FirstBit_LSB;						//小端模式
	SPI_Structure.SPI_Mode = SPI_Mode_Master;							//主设备
	SPI_Structure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2, &SPI_Structure);
	//SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
}


void iic_configuration(void){  //iic初始化
	
	//I2C1 初始化 用于：陀螺仪MPU6050
	I2C_InitTypeDef i2c_init_stru;
//	I2C_DeInit(I2C1);
//	i2c_init_stru.I2C_ClockSpeed = 1000;    //1kHz
//	i2c_init_stru.I2C_Mode = I2C_Mode_I2C;
//	i2c_init_stru.I2C_DutyCycle = I2C_DutyCycle_2;   //这个暂时不明白
//	i2c_init_stru.I2C_Ack = I2C_Ack_Enable;   //使能应答
//	i2c_init_stru.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  //应答7位地址
//	I2C_Init(I2C1,&i2c_init_stru);
//	I2C_ITConfig(I2C1,I2C_IT_EVT,ENABLE);
//	I2C_Cmd(I2C1,ENABLE);
	
	//I2C2 初始化  用于：摄像头
	I2C_DeInit(I2C2);
	i2c_init_stru.I2C_ClockSpeed = 100;    //1kHz
	i2c_init_stru.I2C_Mode = I2C_Mode_I2C;
	i2c_init_stru.I2C_DutyCycle = I2C_DutyCycle_2;   //这个暂时不明白
	i2c_init_stru.I2C_Ack = I2C_Ack_Enable;   //使能应答
	i2c_init_stru.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  //应答7位地址
	I2C_Init(I2C2,&i2c_init_stru);
	I2C_ITConfig(I2C2,I2C_IT_EVT,ENABLE);
	I2C_Cmd(I2C2,ENABLE);
}



//void dma_config(){
//    DMA_InitTypeDef dma_init_stru;
//    
//    //配置摄像头camera DMA  
//    DMA_DeInit(DMA2_Stream1);   
//    dma_init_stru.DMA_BufferSize = 20000;  //暂时不同其意思
//    dma_init_stru.DMA_Channel = DMA_Channel_6;  //!!!!通道1不能单只写数字1，否则，经调试发现，与CR寄存器相或之后，EN位置1，导致下面的寄存器无法被配置
//    dma_init_stru.DMA_DIR = DMA_DIR_PeripheralToMemory; //数据方向为外设到内存
//    dma_init_stru.DMA_FIFOMode = DMA_FIFOMode_Enable;   //DMA_FIF、OMode_Enable使能FIFO模式
//    dma_init_stru.DMA_FIFOThreshold = DMA_FIFOThreshold_Full; //设置FIFO阈值为满
//    dma_init_stru.DMA_Memory0BaseAddr = (uint32_t)(g_camera_image);//(g_camera_image) ((u32)(0x60000000|0x0C000000|0x00000080));  //设置存储数据的内存基地址
//    dma_init_stru.DMA_MemoryBurst = DMA_MemoryBurst_Single;  //每次传输一个数据单元
//    dma_init_stru.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte ;  //半字传输
//    dma_init_stru.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存每次接收到数据之后，地址都会自增
//    dma_init_stru.DMA_Mode = DMA_Mode_Circular  ;  //DMA_Mode_Circular不停地传送  DMA_Mode_Normal  
//    dma_init_stru.DMA_PeripheralBaseAddr = (u32)(&(GPIOG->IDR));  //DCMI_DR_ADDR外设地址为DCMI的数据寄存器地址
//    dma_init_stru.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //一次传一个数据单元
//    dma_init_stru.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte; //外设数据单元大小为字
//    dma_init_stru.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //不使用外设地址自增
//    dma_init_stru.DMA_Priority = DMA_Priority_High;  //DMA传输的优先级为高
//    DMA_Init(DMA2_Stream1,&dma_init_stru);
//	//	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_FE,ENABLE);
//	//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
//    DMA_Cmd(DMA2_Stream1,ENABLE);
//
//}

void fsmc_config(){
    FSMC_NORSRAMInitTypeDef fsmc_init_stru;
    FSMC_NORSRAMTimingInitTypeDef readWriteTiming,writeTiming;



/**********************TFTLCD的FSMC配置***********************/
   /* FMC_Bank1_NORSRAM4 configuration */
  readWriteTiming.FSMC_AddressSetupTime = 0XF;	 //地址建立时间（ADDSET）为16个HCLK 1/168M=6ns*16=96ns	
  readWriteTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（ADDHLD）模式A未用到	
  readWriteTiming.FSMC_DataSetupTime = 60;			//数据保存时间为60个HCLK	=6*60=360ns
  readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
  readWriteTiming.FSMC_CLKDivision = 0x00;
  readWriteTiming.FSMC_DataLatency = 0x00;
  readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 
    

	writeTiming.FSMC_AddressSetupTime =9;	      //地址建立时间（ADDSET）为9个HCLK =54ns 
  writeTiming.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（A		
  writeTiming.FSMC_DataSetupTime = 8;		 //数据保存时间为6ns*9个HCLK=54ns
  writeTiming.FSMC_BusTurnAroundDuration = 0x00;
  writeTiming.FSMC_CLKDivision = 0x00;
  writeTiming.FSMC_DataLatency = 0x00;
  writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 
   /* Color LCD configuration ------------------------------------
      LCD configured as follow:
         - Data/Address MUX = Disable
         - Memory Type = SRAM
         - Data Width = 16bit
         - Write Operation = Enable
         - Extended Mode = Enable
         - Asynchronous Wait = Disable */
 
  fsmc_init_stru.FSMC_Bank = FSMC_Bank1_NORSRAM4;//  这里我们使用NE4 ，也就对应BTCR[6],[7]。
  fsmc_init_stru.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // 不复用数据地址
  fsmc_init_stru.FSMC_MemoryType =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM   
  fsmc_init_stru.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//存储器数据宽度为16bit   
  fsmc_init_stru.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
  fsmc_init_stru.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	fsmc_init_stru.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
  fsmc_init_stru.FSMC_WrapMode = FSMC_WrapMode_Disable;   
  fsmc_init_stru.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
  fsmc_init_stru.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	//  存储器写使能
  fsmc_init_stru.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
  fsmc_init_stru.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; // 读写使用不同的时序
  fsmc_init_stru.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
  fsmc_init_stru.FSMC_ReadWriteTimingStruct = &readWriteTiming; //读写时序
  fsmc_init_stru.FSMC_WriteTimingStruct = &writeTiming;  //写时序;
 
   FSMC_NORSRAMInit(&fsmc_init_stru);   
 
   /* Enable FMC NOR/SRAM Bank3 */
   FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);

}











