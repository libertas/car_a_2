#include "ccd.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stdlib.h"
#include "misc.h"



static ccd_d *g_p_ccd_d;


/* 函数名：ccd_init()
 * 功能：初始化CCD
 * 参数：
 * 返回值：
 */  

int ccd_init(ccd_d *p_ccd_d){
    //初始化CCD数据结构
    g_p_ccd_d = p_ccd_d;
    p_ccd_d->size = CCD_DEFAUT_LINE_SIZE;
    p_ccd_d->data = (u16 *)malloc(CCD_DEFAUT_LINE_SIZE*sizeof(u16));
    ccd_rcc_config();
    ccd_gpio_config();
    ccd_dma_config();
    ccd_adc_config();
    ccd_tim5_config();
    ccd_nvic_config();
    return 1;
}


/* 函数名：void ccd_rcc_config()
 * 功能：
 * 参数：
 * 返回值：
 */
void ccd_rcc_config(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

/* 函数名：
 * 功能：
 * 参数：
 * 返回值：
 */
void ccd_gpio_config(){
    GPIO_InitTypeDef gpio_init_stru;
    gpio_init_stru.GPIO_Mode = GPIO_Mode_AN;
    gpio_init_stru.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_stru.GPIO_OType = GPIO_OType_PP;
    gpio_init_stru.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //使用ADC1通道6为ADC的模拟输入
    gpio_init_stru.GPIO_Pin = GPIO_Pin_6;   
    GPIO_Init(GPIOA,&gpio_init_stru);
    //CCD的CLK:PA0
    gpio_init_stru.GPIO_Mode = GPIO_Mode_AF;
    gpio_init_stru.GPIO_Pin = GPIO_Pin_0;   
    GPIO_Init(GPIOA,&gpio_init_stru);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    //CCD的SI:PA4
    gpio_init_stru.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init_stru.GPIO_Pin = GPIO_Pin_4;   
    GPIO_Init(GPIOA,&gpio_init_stru);
}

/* 函数名：
 * 功能：
 * 参数：
 * 返回值：
 */
void ccd_nvic_config(){
    NVIC_InitTypeDef nvic_init_stru; 
    /**************************下面配置NVIC****************************/
    //配置PG12外部中断
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    nvic_init_stru.NVIC_IRQChannel=DMA2_Stream0_IRQn;
    nvic_init_stru.NVIC_IRQChannelPreemptionPriority = CCD_NVIC_IRQPP;//1
    nvic_init_stru.NVIC_IRQChannelSubPriority = CCD_NVIC_IRQSP;//0
    nvic_init_stru.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_stru); 
    /**************************上面配置NVIC****************************/
}


/* 函数名：ccd_dma_confg()
 * 功能：配置CCD所需要的DMA
 * 参数：无
 * 返回值：
 */
void ccd_dma_config(){

    //ADC1使用DMA2数据流0通道0
    DMA_InitTypeDef dma_init_stru;
    DMA_DeInit(CCD_DMA);   
    dma_init_stru.DMA_BufferSize = g_p_ccd_d->size;  //
    dma_init_stru.DMA_Channel = DMA_Channel_0;  //!!!!通道1不能单只写数字1，否则，经调试发现，与CR寄存器相或之后，EN位置1，导致下面的寄存器无法被配置
    dma_init_stru.DMA_DIR = DMA_DIR_PeripheralToMemory; //数据方向为外设到内存
    dma_init_stru.DMA_FIFOMode = DMA_FIFOMode_Enable;   //DMA_FIF、OMode_Enable使能FIFO模式
    dma_init_stru.DMA_FIFOThreshold = DMA_FIFOThreshold_Full; //设置FIFO阈值为满
    dma_init_stru.DMA_Memory0BaseAddr = (uint32_t)(g_p_ccd_d->data);//(g_camera_image) ((u32)(0x60000000|0x0C000000|0x00000080));  //设置存储数据的内存基地址
    dma_init_stru.DMA_MemoryBurst = DMA_MemoryBurst_Single;  //每次传输一个数据单元
    dma_init_stru.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //半字传输
    dma_init_stru.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存每次接收到数据之后，地址都会自增
    dma_init_stru.DMA_Mode = DMA_Mode_Circular  ;  //DMA_Mode_Circular不停地传送  DMA_Mode_Normal  
    dma_init_stru.DMA_PeripheralBaseAddr = (u32)(&(ADC1->DR));  //
    dma_init_stru.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //一次传一个数据单元
    dma_init_stru.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //外设数据单元大小为字
    dma_init_stru.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //不使用外设地址自增
    dma_init_stru.DMA_Priority = DMA_Priority_Medium;  //DMA传输的优先级为中等
    DMA_Init(CCD_DMA,&dma_init_stru);
    DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
    DMA_Cmd(CCD_DMA,ENABLE);
}

/* 函数名：adc_config()
 * 功能：配置CCD的ADC
 * 参数：
 * 返回值：
 */

void ccd_adc_config(){
    
    ADC_CommonInitTypeDef adc_cominit_stru;
    ADC_InitTypeDef adc_init_stru;

    adc_cominit_stru.ADC_Mode = ADC_Mode_Independent;
    adc_cominit_stru.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
    adc_cominit_stru.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;  //多重模式DMA失能
    adc_cominit_stru.ADC_Prescaler = ADC_Prescaler_Div6;   //预分频为6分频 ADCCLK = PCLK2/4  ADCCLK最好不要超过36MHz
    ADC_CommonInit(&adc_cominit_stru);

    adc_init_stru.ADC_Resolution = ADC_Resolution_12b;  //12位模式
    adc_init_stru.ADC_ScanConvMode = DISABLE;   //因为使用单通道，所以不用扫描模式
    adc_init_stru.ADC_ContinuousConvMode = DISABLE;  //连续转换失能
    adc_init_stru.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T5_CC1;  //选择触发源
    adc_init_stru.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;  //上升沿触发
    adc_init_stru.ADC_DataAlign = ADC_DataAlign_Right; //右对齐,低字节对齐
    adc_init_stru.ADC_NbrOfConversion = 1;  //一个转换在规则序列中
    ADC_Init(ADC1,&adc_init_stru);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_480Cycles);
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  //  ADC1->CR2 &= ~(1 << 9);   //DDS置0  表示DMA最后一个数据传输完之后，就不请求传输,之后，只有把DMA位清零后置1才能请求DMA（DMA传输数目可以在DMA控制器中配置）
    ADC_DMACmd(ADC1,ENABLE);  //使能DMA请求（单一模式下）
}
  
/* 函数名：ccd_tim8_config()
 * 功能：配置CCD所需要的定时器
 * 参数：
 * 返回值：
 */
void ccd_tim5_config(){
	TIM_TimeBaseInitTypeDef timer_base_stru;
	TIM_OCInitTypeDef timer_oc_stru;
	u16 period;
	u16 cmp;
    u32 freq = 8000;
	period = 1000000/freq - 1;
	cmp = period*5000/10000;
	
	
	timer_base_stru.TIM_Prescaler = 167;  //168/(167+1)=1M
	timer_base_stru.TIM_Period = period;  //freq = 1000000/(TIM_Period+1)   TIM_Period = 1000000/freq - 1
	timer_base_stru.TIM_ClockDivision = 0;
	timer_base_stru.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5,&timer_base_stru);
	
	
	timer_oc_stru.TIM_OCMode = TIM_OCMode_PWM1; 
	timer_oc_stru.TIM_OCPolarity = TIM_OCPolarity_Low;//极性是先低后高
	timer_oc_stru.TIM_OutputState = TIM_OutputState_Enable;
	timer_oc_stru.TIM_Pulse = cmp;
	TIM_OC1Init(TIM5,&timer_oc_stru);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM5,ENABLE);
	TIM_Cmd(TIM5,ENABLE);   //SI发出高电平之后，再使能PWM
}


/* 函数名：ccd_start()
 * 功能：开始采集CCD数据
 * 参数：
 * 返回值：
 */

void ccd_start(){
    ADC_Cmd(ADC1,ENABLE);
    ccd_read_start();
}

/* 函数名：void ccd_read_start()
 * 功能：开始读取数据
 * 参数：
 * 返回值：
 */
void ccd_read_start(){
    int i,j;
    TIM5->CNT  = 0;
    GPIO_SetBits(GPIOA,GPIO_Pin_4);  //si置位
    //延时，等待CLK的上升沿后再拉低si
    for(i = 0;i < 6;i++)
        for(j = 0;j < 1000;j++);  
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);  //拉低si
    ADC_DMACmd(ADC1,DISABLE); //失能DMA请求（单一模式下）
    for(i = 0;i < 100;i++)
        for(j = 0;j < 1000;j++);  
    ADC_DMACmd(ADC1,ENABLE);  //使能DMA请求（单一模式下）
}

/* 函数名：ccd_stop()
 * 功能：停止采集CCD数据
 * 参数：
 * 返回值：
 */

void ccd_stop(){
    ADC_Cmd(ADC1,DISABLE);
}


/* 函数名：DMA2_Stream0_IRQHandler()
 * 功能：DMA2_Stream0 的中断
 * 参数：
 * 返回值：
 */
void DMA2_Stream0_IRQHandler(){
    if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0) != RESET ){  //传输完成产生的中断
        DMA2_Stream0->CR &= ~(uint32_t)DMA_SxCR_EN;  //配置寄存器之前，先关闭DMA
        while(DMA2_Stream0->CR&0X01);//等待DMA可以被配置
        DMA2_Stream0->NDTR = g_p_ccd_d->size;
        DMA2_Stream0->FCR=0X0000021;//复位FIFO
        DMA2_Stream0->M0AR=(uint32_t)(g_p_ccd_d->data);//
        DMA2_Stream0->CR |= (uint32_t)DMA_SxCR_EN;  //使能DMA
        ccd_read_start();
        DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);  //清除流0的传输完成中断标志位
    }
}
