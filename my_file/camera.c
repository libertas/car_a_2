#include "camera.h"
#include "camera_cfg.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "tftlcd.h"
#include "main.h"


static image *p_dma_image;
static char g_str_temp[128];

/* 函数名：camera_init()
 * 功能：摄像头初始化
 * 参数：图像数据结构指针
 * 返回值：
 */
int camera_init(image **pp_image){
    int i,j;
    int erro_n;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //使能DMA时钟
    DMA_InitTypeDef dma_init_stru;
#if PERIPHERAL_INIT_EN == 1
    I2C_InitTypeDef i2c_init_stru;
    GPIO_InitTypeDef gpio_init_stru;
    TIM_TimeBaseInitTypeDef timer_base_stru;
    TIM_OCInitTypeDef timer_oc_stru;
    TIM_ICInitTypeDef tim_ic_init_stru;
    EXTI_InitTypeDef exti_init_structure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  //使能I2C时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);   //使能syscfg时钟，用于外部中断
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    //以下是IIC配置
    gpio_init_stru.GPIO_Mode = GPIO_Mode_AF;
    gpio_init_stru.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init_stru.GPIO_OType = GPIO_OType_OD;
    gpio_init_stru.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init_stru.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;   //PB8:SCL  PB9:SDA
    GPIO_Init(GPIOB,&gpio_init_stru);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
    I2C_DeInit(CAMERA_I2CX);
    i2c_init_stru.I2C_ClockSpeed = 100;   //100Hz
    i2c_init_stru.I2C_Mode = I2C_Mode_I2C;
    i2c_init_stru.I2C_DutyCycle = I2C_DutyCycle_2;   //
    i2c_init_stru.I2C_Ack = I2C_Ack_Enable;   //  使能应答
    i2c_init_stru.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  // 使用7位地址
    I2C_Init(CAMERA_I2CX,&i2c_init_stru);
    I2C_Cmd(CAMERA_I2CX,ENABLE);
    //以下是数据引脚的配置
    gpio_init_stru.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_stru.GPIO_OType = GPIO_OType_PP;
    gpio_init_stru.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init_stru.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3
                                |GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;  
    GPIO_Init(GPIOG,&gpio_init_stru);
    
    //PA8:作为TIM1输入捕获，接摄像头的PCLK,下降沿采集数据
    gpio_init_stru.GPIO_Mode = GPIO_Mode_AF;
    gpio_init_stru.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init_stru.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA,&gpio_init_stru);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    
    u16 cmp = 5000;
    
    timer_base_stru.TIM_Prescaler = 168 - 1;  //42/(41+1)=1M
    timer_base_stru.TIM_Period = 10000;  //freq = 1000000/(TIM_Period+1)   TIM_Period = 1000000/freq - 1
    timer_base_stru.TIM_ClockDivision = 0;
    timer_base_stru.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1,&timer_base_stru);
    
    tim_ic_init_stru.TIM_Channel = TIM_Channel_1;
    tim_ic_init_stru.TIM_ICFilter = 5;
    tim_ic_init_stru.TIM_ICPolarity = TIM_ICPolarity_Falling;
    tim_ic_init_stru.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    tim_ic_init_stru.TIM_ICSelection = TIM_ICSelection_DirectTI;
    
    TIM_ICInit(TIM1,&tim_ic_init_stru);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //使能定时器溢出中断
    TIM1->DIER |= 1<<9;   //使能DMA请求
    TIM_Cmd(TIM1,ENABLE);
    //PD3:下降沿中断，作为场中断，接摄像头的VSYNC
    gpio_init_stru.GPIO_Mode = GPIO_Mode_IN;
    gpio_init_stru.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOD,&gpio_init_stru);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource3);
    exti_init_structure.EXTI_Line = EXTI_Line3;
    exti_init_structure.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_structure.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init_structure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_structure); 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //摄像头场中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //DMA中断
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;//0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
//下面配置摄像头的寄存器
#if CAMERA_TYPE == CAMERA_OV2640
    for(i=0;i<sizeof(ov2640_cif_init_reg_tbl)/sizeof(ov2640_cif_init_reg_tbl[0]);i++){
        if((erro_n = camera_reg_write(ov2640_cif_init_reg_tbl[i][0],ov2640_cif_init_reg_tbl[i][1])) < 0){
            return erro_n;
        }
        for(j = 0;j < 10000;j++);  //稍微等一会儿
    }
    for(i=0;i<sizeof(ov2640_yuv422_reg_tbl)/sizeof(ov2640_yuv422_reg_tbl[0]);i++){
        if((erro_n = camera_reg_write(ov2640_yuv422_reg_tbl[i][0],ov2640_yuv422_reg_tbl[i][1])) < 0){
            return erro_n;
        }
        for(j = 0;j < 10000;j++);  //稍微等一会儿
    }
    for(i=0;i<sizeof(g_ov2640_custom_config)/sizeof(g_ov2640_custom_config[0]);i++){
        if((erro_n = camera_reg_write(g_ov2640_custom_config[i][0],g_ov2640_custom_config[i][1])) < 0){
            return erro_n;
        }
        for(j = 0;j < 10000;j++);  //稍微等一会儿
    }
		
    ov2640_window_set(251,5,352,288);
    ov2640_outsize_set(IMAGE_DEFAULT_WIDTH,IMAGE_DEFAULT_HEIGHT);
#elif CAMERA_TYPE == CAMERA_OV7620
    for(i=0;i<sizeof(ov7620_init_tbl)/sizeof(ov7620_init_tbl[0]);i++){
        if((erro_n = camera_reg_write(ov7620_init_tbl[i][0],ov7620_init_tbl[i][1])) < 0){
            return erro_n;
        }
        for(j = 0;j < 10000;j++);  //稍微等一会儿
    }
#elif CAMERA_TYPE == CAMERA_OV7725
    for(i=0;i<sizeof(ov7725_init_tbl)/sizeof(ov7725_init_tbl[0]);i++){
        if((erro_n = camera_reg_write(ov7725_init_tbl[i][0],ov7725_init_tbl[i][1])) < 0){
            return erro_n;
        }
        for(j = 0;j < 100000;j++);  //稍微等一会儿,一定要多等一会儿，不然寄存器写不了数据
    }
#endif
    camera_create_image(pp_image,IMAGE_DEFAULT_WIDTH,IMAGE_DEFAULT_HEIGHT,IMAGE_DEFAULT_TYPE);  //创建图像
    p_dma_image = *pp_image;
    //下面开始配置DMA
    
    DMA_DeInit(DMA2_Stream1);   
    dma_init_stru.DMA_BufferSize = p_dma_image->size;  //一次传输的大小
    dma_init_stru.DMA_Channel = DMA_Channel_6;  //!!!!通道1不能单只写数字1，否则，经调试发现，与CR寄存器相或之后，EN位置1，导致下面的寄存器无法被配置
    dma_init_stru.DMA_DIR = DMA_DIR_PeripheralToMemory; //数据方向为外设到内存
    dma_init_stru.DMA_FIFOMode = DMA_FIFOMode_Enable;   //DMA_FIF、OMode_Enable使能FIFO模式
    dma_init_stru.DMA_FIFOThreshold = DMA_FIFOThreshold_Full; //设置FIFO阈值为满
    dma_init_stru.DMA_Memory0BaseAddr = (uint32_t)(p_dma_image->array);//(g_camera_image) ((u32)(0x60000000|0x0C000000|0x00000080));  //设置存储数据的内存基地址
    dma_init_stru.DMA_MemoryBurst = DMA_MemoryBurst_Single;  //每次传输一个数据单元
    dma_init_stru.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte ;  //半字传输
    dma_init_stru.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存每次接收到数据之后，地址都会自增
    dma_init_stru.DMA_Mode = DMA_Mode_Normal  ;  //DMA_Mode_Circular不停地传送  DMA_Mode_Normal  
    dma_init_stru.DMA_PeripheralBaseAddr = (u32)(&(GPIOG->IDR));  //GPIOG的读寄存器
    dma_init_stru.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; //一次传一个数据单元
    dma_init_stru.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte; //外设数据单元大小为字
    dma_init_stru.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //不使用外设地址自增
    dma_init_stru.DMA_Priority = DMA_Priority_High;  //DMA传输的优先级为高
    DMA_Init(DMA2_Stream1,&dma_init_stru);
//		DMA_ITConfig(DMA2_Stream1,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE|DMA_IT_FE,ENABLE);
	//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
	  DMA_Cmd(DMA2_Stream1,ENABLE);   //不采集数据那么快,调用camera_start()函数再开始

	return 0;
}

/* 函数名：camera_create_image()
 * 功能：创建图像数据结构
 * 参数：无 
 * 返回值：
 */
int camera_create_image(image **p_image,u32 width,u32 height,u8 type){
    u8 *image_array;
    (*p_image) = (image *)malloc(sizeof(image));
    (*p_image)->width = width;
    (*p_image)->height = height;
    (*p_image)->type = type;
    switch(type){
        case IMAGE_TYPE_BIN_THRESH:
            (*p_image)->size = (width*height + 7)/8;
            break;
        case IMAGE_TYPE_GRAY:
            (*p_image)->size = width*height;
            break;
        case IMAGE_TYPE_YUYV:
            (*p_image)->size = width*height*2;
            break;
    }
    (*p_image)->array = (u8 *)malloc((*p_image)->size);
    (*p_image)->ready = 0;
}

/* 函数名：camera_start()
 * 功能：开始采集摄像头数据
 * 参数：无 
 * 返回值：
 */

int camera_start(){
    camera_stop();	//尝试关闭DCMI  尝试关闭DMA
    while(DMA2_Stream1->CR&0X01);//等待DMA2_Stream1可配置
    DMA2->LIFCR|=0X3D<<6*1;	 //清空通道1上所有中断标志
    DMA2_Stream1->NDTR = p_dma_image->size;
    DMA2_Stream1->FCR=0X0000021;//设置为默认值
    DMA2_Stream1->M0AR=(uint32_t)(p_dma_image->array);//(g_camera_image)(0x60000000|0x0C000000|0x00000080);   //重新设置图像的地址
    DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;  //启动DMA
}


/* 函数名：camera_stop()
 * 功能：停止采集摄像头数据
 * 参数：无 
 * 返回值：
 */

int camera_stop(){
    DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN; //关闭DMA

}

/* 函数名：camera_reg_write()
 * 功能：给camera指定寄存器写入数据
 * 参数：u8 reg_addr 要写入数据的寄存器的地址。u8 data_write 要给寄存器写入的数据
 *       的空间中（数组）。
 * 返回值：-1:标明处于连续读取数据状态;
 *          <-1，说明写超时,根据返回值的大小，可以定位到超时的代码处;
 *          返回0则说明写成功
 */
int camera_reg_write(u8 reg_addr,u8 data_write){
    u32 timeout;
    u16 i2c_sr1_temp,i2c_sr2_temp;

    I2C_GenerateSTART(CAMERA_I2CX,ENABLE);
    timeout = 10000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -2;
        }
    }
    I2C_ClearFlag(CAMERA_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(CAMERA_I2CX,CAMERA_I2C_ADDR,0); //发送I2C地址，且设置为写
    
    timeout = 10000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_ADDR) == 0){  //等待地址发送完毕
        if(timeout-- == 0){
            return -3;
        }
    }
    timeout = 3000;
    while(timeout--);  //等一会儿
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(CAMERA_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(CAMERA_I2CX,I2C_Register_SR2);
    I2C_SendData(CAMERA_I2CX,reg_addr);  //发送寄存器地址
    
    timeout = 10000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_BTF) == 0){//等待数据写完
        if(timeout-- == 0){
            return -4;
        }
    }
    timeout = 3000;
    while(timeout--);  //让子弹飞一会
    I2C_SendData(CAMERA_I2CX,data_write);   //发送数据的同时，会清除BTF的
    timeout = 10000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_BTF) == 0){  //等待数据写完
        if(timeout-- == 0){
            return -5;
        }
    }
    I2C_GenerateSTOP(CAMERA_I2CX,ENABLE);  //发送停止位
    timeout = 1000;
    while(timeout--);  //等一会儿
    
    return 0;
}



/* 函数名：camera_reg_read()
 * 功能：读取camera寄存器的数据
 * 参数: u8 reg_addr 寄存器地址   u8 *data_read 保存读取数据的变量的指针
 * 返回值：<-1，则说明读(写)超时,根据返回值的大小，可以定位到超时的代码处;
 *         0,则说明写成功。
 * 
 */
int camera_reg_read(u8 reg_addr,u8 *data_read){
    u16 i2c_sr1_temp,i2c_sr2_temp;
    u32 timeout;
    int i,j;

	I2C_GenerateSTART(CAMERA_I2CX,ENABLE); //发出起始信号
    timeout = 1000000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -2;
        }
    }
    I2C_ClearFlag(CAMERA_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(CAMERA_I2CX,CAMERA_I2C_ADDR,0);
    
    timeout = 1000000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_ADDR) ==0){  //等待I2C地址发送完毕
        if(timeout-- == 0){
            return -3;
        }
    }
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(CAMERA_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(CAMERA_I2CX,I2C_Register_SR2);
    I2C_SendData(CAMERA_I2CX,reg_addr);  //发送寄存器地址

    timeout = 1000000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_BTF) == 0){  //等待寄存器地址发送完毕
        if(timeout-- == 0){
            return -4;
        }
    }
    //发出重复起始信号(该动作会使BTF位被清零),开始读数据
		I2C_GenerateSTOP(CAMERA_I2CX,ENABLE);
    I2C_GenerateSTART(CAMERA_I2CX,ENABLE);  

    timeout = 1000000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -5;
        }
    }
    I2C_ClearFlag(CAMERA_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(CAMERA_I2CX,CAMERA_I2C_ADDR,1);  

    timeout = 1000000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_ADDR) ==0){  //等待I2C地址发送完毕
        if(timeout-- == 0){
            return -6;
        }
    }
    //因为只读一次数据，所以在清ADDR之前需要设置NACK 和 STOP 
    //设置NACK
    I2C_AcknowledgeConfig(CAMERA_I2CX,DISABLE);
    //设置STOP位
    I2C_GenerateSTOP(CAMERA_I2CX,ENABLE);
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(CAMERA_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(CAMERA_I2CX,I2C_Register_SR2);

    //等待目标寄存器数据接收完毕
    timeout = 1000000;
    while(I2C_GetFlagStatus(CAMERA_I2CX,I2C_FLAG_RXNE) == 0){
        if(timeout-- == 0){
            return -7;
        }
    }
    *data_read = I2C_ReceiveData(CAMERA_I2CX);  //读I2C接收到的数据,同时BTF位被清零

    //接收好数据之后，别忘了使能ACK
    I2C_AcknowledgeConfig(CAMERA_I2CX,ENABLE);  //使能ACK
    return 0;
}

/* 函数名：camera_config(u8 reg_tbl[][2])
 * 功能：配置摄像头寄存器
 * 参数：摄像头寄存器配置列表
 * 返回值：0，表示函数执行成功
 *         < 0,表示执行错误
 */
int camera_config(u8 reg_tbl[][2]){
    int erro_n;
    int i,j;
    for(i=0;i<sizeof(reg_tbl)/sizeof(reg_tbl[0]);i++){
        if((erro_n = camera_reg_write(reg_tbl[i][0],reg_tbl[i][1])) < 0){
            return erro_n;
        }
        for(i = 0;i < 10;i++)
            for(j = 0;j < 1000;j++);  //稍微等一会儿
    }
    return 0;
}

/* 函数名：
 * 功能：
 * 参数：
 * 返回值：
 */
u8 ov2640_outsize_set(u16 width,u16 height){
    u16 outh;
    u16 outw;
    u8 temp; 
    if(width%4)return 1;
    if(height%4)return 2;
    outw=width/4;
    outh=height/4; 
    camera_reg_write(0XFF,0X00);	
    camera_reg_write(0XE0,0X04);			
    camera_reg_write(0X5A,outw&0XFF);		//设置OUTW的低八位
    camera_reg_write(0X5B,outh&0XFF);		//设置OUTH的低八位
    temp=(outw>>8)&0X03;
    temp|=(outh>>6)&0X04;
    camera_reg_write(0X5C,temp);				//设置OUTH/OUTW的高位 
    camera_reg_write(0XE0,0X00);	
    return 0;
}

/* 函数名：
 * 功能：
 * 参数：
 * 返回值：
 */

void ov2640_window_set(u16 sx,u16 sy,u16 width,u16 height){
    u16 endx;
    u16 endy;
    u8 temp; 
    endx=sx+width/2;	//V*2
    endy=sy+height/2;
    
    camera_reg_write(0XFF,0X01);			
    camera_reg_read(0X03,&temp);				//读取Vref之前的值
    temp&=0XF0;
    temp|=((endy&0X03)<<2)|(sy&0X03);
    camera_reg_write(0X03,temp);				//设置Vref的start和end的最低2位
    camera_reg_write(0X19,sy>>2);			//设置Vref的start高8位
    camera_reg_write(0X1A,endy>>2);			//设置Vref的end的高8位
    
    camera_reg_read(0X32,&temp);				//读取Href之前的值
    temp&=0XC0;
    temp|=((endx&0X07)<<3)|(sx&0X07);
    camera_reg_write(0X32,temp);				//设置Href的start和end的最低3位
    camera_reg_write(0X17,sx>>3);			//设置Href的start高8位
    camera_reg_write(0X18,endx>>3);			//设置Href的end的高8位
}

/* 函数名：
 * 功能：
 * 参数：
 * 返回值：
 */
u8 ov2640_imagewin_set(u16 offx,u16 offy,u16 width,u16 height){
    u16 hsize;
    u16 vsize;
    u8 temp; 
    if(width%4)return 1;
    if(height%4)return 2;
    hsize=width/4;
    vsize=height/4;
    camera_reg_write(0XFF,0X00);	
    camera_reg_write(0XE0,0X04);					
    camera_reg_write(0X51,hsize&0XFF);		
    camera_reg_write(0X52,vsize&0XFF);		
    camera_reg_write(0X53,offx&0XFF);		
    camera_reg_write(0X54,offy&0XFF);		
    temp=(vsize>>1)&0X80;
    temp|=(offy>>4)&0X70;
    temp|=(hsize>>5)&0X08;
    temp|=(offx>>8)&0X07; 
    camera_reg_write(0X55,temp);				
    camera_reg_write(0X57,(hsize>>2)&0X80);	
    camera_reg_write(0XE0,0X00);	
    return 0;
} 

/* 函数名：
 * 功能：
 * 参数：
 * 返回值：
 */
u8 ov2640_imagesize_set(u16 width,u16 height){ 
    u8 temp; 
    camera_reg_write(0XFF,0X00);			
    camera_reg_write(0XE0,0X04);			
    camera_reg_write(0XC0,(width)>>3&0XFF);		
    camera_reg_write(0XC1,(height)>>3&0XFF);		
    temp=(width&0X07)<<3;
    temp|=height&0X07;
    temp|=(width>>4)&0X80; 
    camera_reg_write(0X8C,temp);	
    camera_reg_write(0XE0,0X00);				 
    return 0;
}

//void ov2640_auto_set(){
//	camera_reg_write(0xff,0x01);
//	camera_reg_write();
//}


/* 函数名：EXTI3_IRQHandler()
 * 功能：摄像头帧中断(引脚GPIOD3外部中断)
 * 参数：无
 * 返回值：无
 */
void EXTI3_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line3) != RESET){
        int i;
        if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 0){
            DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN; //配置寄存器之前，先关闭DMA
            while(DMA2_Stream1->CR&0X01);//等待DMA可以被配置
            DMA2->LIFCR|=0X3D<<6;	 //清除所有的中断标志
            DMA2_Stream1->NDTR = p_dma_image->size;
            DMA2_Stream1->FCR=0X0000021;//
            DMA2_Stream1->M0AR=(uint32_t)(p_dma_image->array);//
            DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;  //??DMA
            p_dma_image->fps += 1;
            p_dma_image->ready = 1;
        }
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

/* 函数名：TIM1_IRQHandler()
 * 功能：为定时器TIM1的溢出中断，用来计算fps（帧率）的。
 * 参数：无
 * 返回值：无
 */
void TIM1_UP_TIM10_IRQHandler (void){
    if(TIM_GetITStatus(TIM1,TIM_IT_Update) != RESET){
        TIM_ClearFlag(TIM1,TIM_FLAG_Update);
        TIM_ClearITPendingBit(TIM1,TIM_FLAG_Update);
        sprintf(g_str_temp,"fps:%d   ",p_dma_image->fps);
        lcd_show_string(30,20,200,100,16,g_str_temp);
        p_dma_image->fps = 0;
    }
    
}

/* 函数名：
 * 功能：
 * 参数：
 * 返回值：
 */
void DMA2_Stream1_IRQHandler(){
    if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_TCIF1) != RESET ){  //传输完成产生的中断
        DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1);
        uprintf(DEBUG_USARTx,"DMA_IT_TCIF1\n");
    }
    if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_HTIF1) != RESET ){
        DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_HTIF1);
        uprintf(DEBUG_USARTx,"DMA_IT_HTIF1\n");
    }
    if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_TEIF1) != RESET ){
        DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TEIF1);
        uprintf(DEBUG_USARTx,"DMA_IT_TEIF1\n");
    }
    if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_DMEIF1) != RESET ){
        DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_DMEIF1);
        uprintf(DEBUG_USARTx,"DMA_IT_DMEIF1\n");
    }
    if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_FEIF1) != RESET ){
        DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_FEIF1);
        uprintf(DEBUG_USARTx,"DMA_IT_FEIF1\n");
    }
}
