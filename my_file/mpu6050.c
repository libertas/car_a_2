#include "mpu6050.h"
//#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
//#include "global.h"


static u8 mpu6050_orig_data[14];   //存储mpu6050的原始数据(刚从模块上读取的数据),最大14个字节
static u16 mpu6050_data_cnt = 0;  //存储数据的数量
static u8 mpu6050_regaddr_start = 0;  //读取MPU6050数据寄存器的起始地址
static u8 mpu6050_cycleread_flag = 0;   //循环连续读取数据标志
static u8 data_choose;
static u8 gyro_dimension;
static u8 gyro_adj_flag = 0;
static float gyro_adj_cnt[3] = {4000,4000,4000}; //默认的校正值
static int gyro_reset_cnt = 200;
static short gyro_centre[3] = {0};
static short gyro_v[3] = {0};
static float pre_gyro_data[3];   //量化的陀螺仪积分数据(未换算成角度或者弧度)
static float *p_accel_data;
static float *p_gyro_data;
static float *p_temp_data;
static DMA_TypeDef *dma1 = DMA1;



int mpu6050_init(mpu6050_init_struct *init_stru){
    int i,j,errno;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);		
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);   
    data_choose = init_stru->data_choose;
    p_accel_data = init_stru->accel_data;
    p_gyro_data = init_stru->gyro_data;
    p_temp_data = init_stru->temp_data;
    gyro_dimension = init_stru->gyro_dimension;
    mpu6050_data_cnt = 0;
    if(((data_choose >> 0) & 1) == 1){  //角速度数据选择
        mpu6050_data_cnt += 6;
        mpu6050_regaddr_start = GYRO_XOUT_H;
    }
    if(((data_choose >> 1) & 1) == 1){  //温度数据选择
        mpu6050_data_cnt += 2;
        mpu6050_regaddr_start = TEMP_OUT_H;
    }
    if(((data_choose >> 2) & 1) == 1){  //加速度数据选择
        //如果只需要加速度数据和角速度数据，则报错返回(原因是地址不连续)
        if(mpu6050_data_cnt == 6){
            return -1;
        }
        mpu6050_data_cnt += 6;
        mpu6050_regaddr_start = ACCEL_XOUT_H;
    }
    //以下配置DMA,配置方式是寄存器配置，DMA相关寄存器信息，查看《STM32F4XX中文参考手册》P220
    if(MPU6050_I2CX == I2C1){  //如果使用I2C1,则使用DMA1_STREAM5 通道1
        MPU6050_DMA->CR &= ~(1 << 0);   //要配置CR寄存器，首先要把EN位置零
        while((MPU6050_DMA->CR & 1) == 1);  //等待CR寄存器可配置
        MPU6050_DMA->CR &= ~(0x07 << 25);
        MPU6050_DMA->CR |= (0x01 << 25);  //选择通道1

    }else if(MPU6050_I2CX == I2C2){   //如果使用I2C2,则使用DMA1_STREAM2 通道7
        MPU6050_DMA->CR &= ~(1 << 0);   //要配置CR寄存器，首先要把EN位置零
        while((MPU6050_DMA->CR & 1) == 1);  //等待CR寄存器可配置
        MPU6050_DMA->CR &= ~(0x07 << 25);
        MPU6050_DMA->CR |= (0x07 << 25);  //选择通道7

    }else{  //亦或者使用I2C3,则使用DMA1_STREAM2通道3
        MPU6050_DMA->CR &= ~(1 << 0);   //要配置CR寄存器，首先要把EN位置零
        while((MPU6050_DMA->CR & 1) == 1);  //等待CR寄存器可配置
        MPU6050_DMA->CR &= ~(0x07 << 25);
        MPU6050_DMA->CR |= (0x03 << 25);  //选择通道3
    }
    MPU6050_DMA->CR &= ~(0x0F << 21);  //配置为存储器突发单次传输，外设突发单次传输
    MPU6050_DMA->CR &= ~(0x03 << 18);   //使用DMA_SxM0AR指针寻址,不使用双缓冲区模式
    MPU6050_DMA->CR |= (0x03 << 16);   //优先级最高
    MPU6050_DMA->CR &= ~(0x03 << 13);   //存储器数据大小为8位 
    MPU6050_DMA->CR &= ~(0x03 << 11);   //外设数据大小为8位,每次从IIC的DR寄存器读取的数据大小就是8位的
    MPU6050_DMA->CR |= (0x01 << 10);   //使能存储器递增模式
    MPU6050_DMA->CR &= ~(0x01 << 9);   //不使用外设地址递增模式  因为我们要一直读取IIC的DR寄存器的数据，其地址没有改变
    MPU6050_DMA->CR &= ~(0x01 << 8);    //禁止循环模式，因为IIC读取最后一个字节的之后，需要
    //禁止DMA，给IIC总线发STOP信号,才能够进行下一次的数据读取
    //如果使用循环模式，则来不及发STOP信号
    MPU6050_DMA->CR &= ~(0x03 << 6);   //DMA传输方向为：从外设到存储器(即把IIC的DR寄存器数据传到我们指定的内存空间，该内存空间是我们定义的一个数组)
    MPU6050_DMA->CR &= ~(0x01 << 5);  //设置DMA为流控制器,即要传输数据的数目由DMA_SxNDTR决定
    MPU6050_DMA->CR &= ~(0x0f << 1);   //先禁止全部中断，在下面使能所需要的中断
    MPU6050_DMA->CR |= (0x08 << 1);   //使能传输完成中断
    MPU6050_DMA->NDTR = mpu6050_data_cnt;   //设置传输数据数量
    MPU6050_DMA->PAR = (u32)(&(MPU6050_I2CX->DR));   //DMA外设地址为I2C的数据寄存器DR的地址
    MPU6050_DMA->M0AR = (u32)mpu6050_orig_data;   //DMA存储器地址为 数组mpu6050_orig_data的首地址
    MPU6050_DMA->FCR &= ~(0x01 << 7);  //禁止FIFO中断
    MPU6050_DMA->FCR |= (0x01 << 2);   //禁止直接模式(使能FIFO模式)
    MPU6050_DMA->FCR |= (0x03 << 0);   //使用FIFO完整容量
    //配置完DMA之后，暂时不使能,DMA在函数mpu6050_read_start()里被使能

    //以下配置IIC，配置方式为库函数配置
#if PERIPHERAL_INIT_EN == 1
    I2C_InitTypeDef i2c_init_stru;
    GPIO_InitTypeDef gpio_init_stru;
    NVIC_InitTypeDef nvic_init_stru; 

    gpio_init_stru.GPIO_Mode = GPIO_Mode_AF;
    gpio_init_stru.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init_stru.GPIO_OType = GPIO_OType_OD;
    gpio_init_stru.GPIO_PuPd = GPIO_PuPd_UP;


#if I2Cn == 1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  //使能I2C时钟
    gpio_init_stru.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;   //PB8:SCL  PB9:SDA
    GPIO_Init(GPIOB,&gpio_init_stru);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
    nvic_init_stru.NVIC_IRQChannel=DMA1_Stream5_IRQn;

#elif I2Cn == 2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);  //使能I2C时钟
    gpio_init_stru.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  //PF0:SCL  PF1:SDA
    GPIO_Init(GPIOF,&gpio_init_stru);
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource0,GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOF,GPIO_PinSource1,GPIO_AF_I2C2);
    nvic_init_stru.NVIC_IRQChannel=DMA1_Stream2_IRQn;
#else
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,ENABLE);  //使能I2C时钟
    gpio_init_stru.GPIO_Pin = GPIO_Pin_8;  //PA8:SCL   PC9:SDA
    GPIO_Init(GPIOA,&gpio_init_stru);
    gpio_init_stru.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC,&gpio_init_stru);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_I2C3);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_I2C3);
    nvic_init_stru.NVIC_IRQChannel=DMA1_Stream2_IRQn;
#endif
    I2C_DeInit(MPU6050_I2CX);
    i2c_init_stru.I2C_ClockSpeed = 100;   //100Hz
	  i2c_init_stru.I2C_Mode = I2C_Mode_I2C;
	  i2c_init_stru.I2C_DutyCycle = I2C_DutyCycle_2;   //
  	i2c_init_stru.I2C_Ack = I2C_Ack_Enable;   //  使能应答
	  i2c_init_stru.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  // 使用7位地址
  	I2C_Init(MPU6050_I2CX,&i2c_init_stru);
  	I2C_ITConfig(MPU6050_I2CX,I2C_IT_EVT,ENABLE);
	  I2C_Cmd(MPU6050_I2CX,ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
    nvic_init_stru.NVIC_IRQChannelPreemptionPriority = 1;//1
    nvic_init_stru.NVIC_IRQChannelSubPriority =0;//0
    nvic_init_stru.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_stru);

#endif
    MPU6050_I2CX->CR2 |= (0x01 << 12);  //LAST置1，可对最后接收的数据生成NACK
    MPU6050_I2CX->CR2 |= (0x01 << 11);  //使能DMA传输数据
    for(i = 0;i < 1000;i++)
        for(j =0;j < 10;j++);   //等待I2C配置完毕
    //下面开始配置MPU6050的寄存器
    mpu6050_reg_write(PWR_MSMT_1,0x00);  //启动MPU6050
    mpu6050_reg_write(SMPLRT_DIV,0x0);  //采样率不分频
    mpu6050_reg_write(CONFIG,0x06);
    mpu6050_reg_write(GYRO_CONFIG,0x18);   //配置陀螺仪(不自检 2000deg/s)
    errno = mpu6050_reg_write(ACCEL_CONFIG,0x01);  //配置加速度计(不自检，2G，5HZ)
    if(errno < 0){
        return errno;
    }

    return 0;
}

int mpu6050_fast_init(float *gyro_data){
		mpu6050_init_struct mpu6050_ist;
		mpu6050_ist.gyro_data = gyro_data;
		mpu6050_ist.data_choose = CHOOSE_GYRO;
		mpu6050_ist.gyro_dimension = GYRO_DIMEN_ANGLE;
		mpu6050_init(&mpu6050_ist);
}


/* 函数名：mpu6050_reg_write()
 * 功能：给MPU6050指定寄存器写入数据
 * 参数：u8 reg_addr 要写入数据的寄存器的地址。u8 data_write 要给寄存器写入的数据
 *       的空间中（数组）。
 * 返回值：-1:标明处于连续读取数据状态;
 *          <-1，说明写超时,根据返回值的大小，可以定位到超时的代码处;
 *          返回0则说明写成功
 */
int mpu6050_reg_write(u8 reg_addr,u8 data_write){
    u32 timeout;
    u16 i2c_sr1_temp,i2c_sr2_temp;
    if(mpu6050_cycleread_flag == 1){   //如果处于连续读取MPU6050数据的状态，则终止函数的执行，返回错误码
        return -1;
    }

    I2C_GenerateSTART(MPU6050_I2CX,ENABLE);

    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -2;
        }
    }
    I2C_ClearFlag(MPU6050_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(MPU6050_I2CX,MPU6050_ADDR,0); //发送I2C地址，且设置为写
    
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_ADDR) == 0){  //等待地址发送完毕
        if(timeout-- == 0){
            return -3;
        }
    }
    timeout = 1000;
    while(timeout--);  //等一会儿
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR2);
    I2C_SendData(MPU6050_I2CX,reg_addr);  //发送寄存器地址
    
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_BTF) == 0){//等待数据写完
        if(timeout-- == 0){
            return -4;
        }
    }
    timeout = 1000;
    while(timeout--);  //让子弹飞一会
    I2C_SendData(MPU6050_I2CX,data_write);   //发送数据的同时，会清除BTF的
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_BTF) == 0){  //等待数据写完
        if(timeout-- == 0){
            return -5;
        }
    }
    I2C_GenerateSTOP(MPU6050_I2CX,ENABLE);  //发送停止位
    timeout = 1000;
    while(timeout--);  //等一会儿
    
    return 0;
}

/* 函数名：mpu6050_reg_read()
 * 功能：读取MPU6050寄存器的数据
 * 参数: u8 reg_addr 寄存器地址   u8 *data_read 保存读取数据的变量的指针
 * 返回值：-1，说明正在连续读取MPU6050数据;
 *         <-1，则说明读(写)超时,根据返回值的大小，可以定位到超时的代码处;
 *         0,则说明写成功。
 * 
 */
int mpu6050_reg_read(u8 reg_addr,u8 *data_read){
    u16 i2c_sr1_temp,i2c_sr2_temp;
    u32 timeout;
    int i,j;

    if(mpu6050_cycleread_flag == 1){
        return -1;
    }
	
		I2C_GenerateSTART(MPU6050_I2CX,ENABLE); //发出起始信号
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -2;
        }
    }
    I2C_ClearFlag(MPU6050_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(MPU6050_I2CX,MPU6050_ADDR,0);
    
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_ADDR) ==0){  //等待I2C地址发送完毕
        if(timeout-- == 0){
            return -3;
        }
    }
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR2);
    I2C_SendData(MPU6050_I2CX,mpu6050_regaddr_start);  //发送寄存器地址

    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_BTF) == 0){  //等待寄存器地址发送完毕
        if(timeout-- == 0){
            return -4;
        }
    }
    //发出重复起始信号(该动作会使BTF位被清零),开始读数据
		I2C_GenerateSTOP(MPU6050_I2CX,ENABLE);
    I2C_GenerateSTART(MPU6050_I2CX,ENABLE);  

    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -5;
        }
    }
    I2C_ClearFlag(MPU6050_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(MPU6050_I2CX,MPU6050_ADDR,1);  

    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_ADDR) ==0){  //等待I2C地址发送完毕
        if(timeout-- == 0){
            return -6;
        }
    }
    //因为只读一次数据，所以在清ADDR之前需要设置NACK 和 STOP 
    //设置NACK
    I2C_AcknowledgeConfig(MPU6050_I2CX,DISABLE);
    //设置STOP位
    I2C_GenerateSTOP(MPU6050_I2CX,ENABLE);
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR2);

    //等待目标寄存器数据接收完毕
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_RXNE) == 0){
        if(timeout-- == 0){
            return -7;
        }
    }
    *data_read = I2C_ReceiveData(MPU6050_I2CX);  //读I2C接收到的数据,同时BTF位被清零

    //接收好数据之后，别忘了使能ACK
    I2C_AcknowledgeConfig(MPU6050_I2CX,ENABLE);  //使能ACK
    return 0;

}




/* 函数名：mpu6050_read_start()
 * 功能：开始连续从MPU6050读取所需要的数据，最终所读取到的数据（处理好的），保存在p_accel_data、p_gyro_data、p_temp_data所指向
 *       的空间中（数组）。
 *       注意：在DMA空闲时才能调用此函数，而且，调用此函数之前，应配置好DMA
 * 参数:无
 * 返回值：-1，DMA忙
 *         <-1,IIC超时，根据返回值大小，可定位对应超时的代码
 *         0,函数执行成功
 */

int mpu6050_read_start(){
    u16 i2c_sr1_temp,i2c_sr2_temp;
    u32 timeout,i,j;


    if((MPU6050_DMA->CR & 1) == 1){  //如果DMA处于使能状态，则返回
        return -1;
    }
    MPU6050_DMA->NDTR = mpu6050_data_cnt;
    MPU6050_DMA->M0AR = (u32)mpu6050_orig_data;
    I2C_GenerateSTART(MPU6050_I2CX,ENABLE); //发出IIC起始信号
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -2;
        }
    }
    I2C_ClearFlag(MPU6050_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(MPU6050_I2CX,MPU6050_ADDR,0);
    
    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_ADDR) ==0){  //等待I2C地址发送完毕
        if(timeout-- == 0){
            return -3;
        }
    }
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR2);
    I2C_SendData(MPU6050_I2CX,mpu6050_regaddr_start);  //发送寄存器地址

    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_BTF) == 0){  //等待寄存器地址发送完毕
        if(timeout-- == 0){
            return -4;
        }
    }
    //发出重复起始信号(该动作会使BTF位被清零),开始读数据
  	I2C_GenerateSTOP(MPU6050_I2CX,ENABLE);
    for(i = 0;i < 10;i++)
        for(j = 0;j < 100;j++);
    I2C_GenerateSTART(MPU6050_I2CX,ENABLE);  

    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_SB) == 0){ //等待起始信号发送完毕
        if(timeout-- == 0){
            return -5;
        }
    }
    I2C_ClearFlag(MPU6050_I2CX,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(MPU6050_I2CX,MPU6050_ADDR,1);  

    timeout = 1000000;
    while(I2C_GetFlagStatus(MPU6050_I2CX,I2C_FLAG_ADDR) ==0){  //等待I2C地址发送完毕
        if(timeout-- == 0){
            return -6;
        }
    }
    //在SCL被释放之前使能DMA
    MPU6050_DMA->CR |= (0x01 << 0);  //使能DMA
    while((MPU6050_DMA->CR & 1) == 0);
    //清ADDR(通过读取状态寄存器SR1 SR2 ),一定要清除ADDR，不然SCL是不会被释放的
    i2c_sr1_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(MPU6050_I2CX,I2C_Register_SR2);
    return 0;
}

/* 函数名：mpu6050_cycleread_start()
 * 功能：循环读取MPU6050的数据
 * 参数:无
 * 返回值：
 * 
 */
int mpu6050_cycleread_start(){
    int errno;
    if((errno = mpu6050_read_start()) < 0){
        return errno;
    }
    mpu6050_cycleread_flag = 1;  //设置循环读模式标志
    return 0;
}

/* 函数名：mpu6050_cycleread_stop()
 * 功能：停止循环读取数据
 * 参数:无
 * 返回值：
 * 
 *
 */
int mpu6050_cycleread_stop(){
    mpu6050_cycleread_flag = 0;
    while((MPU6050_DMA->CR & 1) == 1);   //等待DMA停止
    return 0;
}


/* 函数名 mpu6050_gyro_reset()
 * 功能：复位陀螺仪某个轴，或者全部的数据
 * 参数: u8 gyro_reset_sel:MPU6050_GYRO_X，复位X轴数据
 *                         MPU6050_GYRO_Y，复位Y轴的数据
 *                         MPU6050_GYRO_Z，复位Z轴的数据
 *                         MPU6050_GYRO_ALL,复位全部数据
 * 返回值：
 * 
 *
 */

int mpu6050_gyro_reset(u8 gyro_reset_sel){
    int i;
    if(gyro_reset_sel != MPU6050_GYRO_ALL){
        gyro_centre[gyro_reset_sel] = gyro_v[gyro_reset_sel];
        pre_gyro_data[gyro_reset_sel] = 0;
    }else{
        gyro_reset_cnt = 100;
    }
    return 0; 
}

/* 函数名 mpu6050_set_gyro_adj()
 * 功能：直接设置陀螺仪校正值
 * 参数: u8 gyro_reset_sel:MPU6050_GYRO_X，设置X轴的校正值
 *                         MPU6050_GYRO_Y，设置Y轴的校正值
 *                         MPU6050_GYRO_Z，设置Z轴的校正值
 * 返回值：无
 * 
 *
 */
void mpu6050_set_gyro_adj(u8 gyro_adj_sel,float gyro_adj){
    gyro_adj_cnt[gyro_adj_sel] = gyro_adj;
}

/* 函数名 mpu6050_get_gyro_adj()
 * 功能：获得当前陀螺仪的校正值
 * 参数: u8 gyro_reset_sel:MPU6050_GYRO_X，获取X轴的校正值
 *                         MPU6050_GYRO_Y，获取Y轴的校正值
 *                         MPU6050_GYRO_Z，获取Z轴的校正值
 * 返回值：无
 * 
 *
 */
float mpu6050_get_gyro_adj(u8 gyro_adj_sel){
    return gyro_adj_cnt[gyro_adj_sel];
}

/* 函数名 mpu6050_gyro_adj_status()
 * 功能：获得陀螺仪的校正状态
 * 参数: 无
 * 返回值：0，陀螺仪不处于校正状态
 *         1，陀螺仪处于校正状态
 * 
 */
u8 mpu6050_gyro_adj_status(){
    return gyro_adj_flag;
}

/* 函数名：mpu6050_gyro_adj()
 * 功能：校正陀螺仪数据，注意：该函数要成对使用，而且每一对的入口参数必须相同
 * 参数： u8 gyro_adj_sel:MPU6050_GYRO_X，校正X轴数据
 *                        MPU6050_GYRO_Y，校正Y轴的数据：
 *                        MPU6050_GYRO_Z，校正Z轴的数据
 * 返回值：
 */


int mpu6050_gyro_adj(u8 gyro_adj_sel){
    if(gyro_adj_flag == 0){  //如果是第一次调用调整函数
        mpu6050_gyro_reset(gyro_adj_sel);   //复位相应的角度值
        gyro_adj_flag = 1;
    }else if(gyro_adj_flag == 1){  //如果是第二次调用调整函数
        gyro_adj_cnt[gyro_adj_sel] = pre_gyro_data[gyro_adj_sel];  //得出调整值(最终目的)
        gyro_adj_flag = 0;
        mpu6050_gyro_reset(gyro_adj_sel);  //再次复位角度值
    }
    return 0;
}

/* 函数名：DMA1_CHANNELx_IRQHANDLER() 
 * 功能：DMA中断，当一批数据读取快完毕的时候，产生中断，及时设置IIC的 STOP 信号，以结束一次IIC的数据传输
 * 参数:无
 * 返回值：无
 */
#if I2Cn == 1
void DMA1_Stream5_IRQHandler(){
#else
void DMA1_Stream2_IRQHandler(){
#endif
    static int tim_cnt;
    static int pre_tim_cnt;
    static float d_t;
    if(DMA_GetITStatus(MPU6050_DMA,DMA_IT_TCIF5) != RESET){   //一批数据传输完成所产生的中断
        I2C_GenerateSTOP(MPU6050_I2CX,ENABLE);   //发出IIC的STOP信号
        tim_cnt = MPU6050_TIM->CNT - tim_cnt;
        if(tim_cnt < 0){
            tim_cnt = pre_tim_cnt;
        }
        pre_tim_cnt = tim_cnt;
        d_t = tim_cnt * TIM_STEP_TIME;
        //下面处理原数据
        if(((data_choose >> 2) & 1) == 1){  //处理加速度数据
            p_accel_data[0] = (mpu6050_orig_data[0] << 8) | mpu6050_orig_data[1];
            p_accel_data[1] = (mpu6050_orig_data[2] << 8) | mpu6050_orig_data[3];
            p_accel_data[2] = (mpu6050_orig_data[4] << 8) | mpu6050_orig_data[5];
            if(((data_choose >> 1) & 1) == 1){
                *p_temp_data = (mpu6050_orig_data[6] << 8) | mpu6050_orig_data[7];
                if(((data_choose >> 0) & 1) == 1){
                    gyro_v[0] = (mpu6050_orig_data[8] << 8) | mpu6050_orig_data[9];
                    gyro_v[1] = (mpu6050_orig_data[10] << 8) | mpu6050_orig_data[10];
                    gyro_v[2] = (mpu6050_orig_data[12] << 8) | mpu6050_orig_data[11];
                }
            }
        }else if(((data_choose >> 1) & 1) == 1){
            *p_temp_data = (mpu6050_orig_data[0] << 8) | mpu6050_orig_data[1];
            if(((data_choose >> 0) & 1) == 1){
                gyro_v[0] = (mpu6050_orig_data[2] << 8) | mpu6050_orig_data[3];
                gyro_v[1] = (mpu6050_orig_data[4] << 8) | mpu6050_orig_data[5];
                gyro_v[2] = (mpu6050_orig_data[6] << 8) | mpu6050_orig_data[7];
            }
        }else{
            gyro_v[0] = (mpu6050_orig_data[0] << 8) | mpu6050_orig_data[1];
            gyro_v[1] = (mpu6050_orig_data[2] << 8) | mpu6050_orig_data[3];
            gyro_v[2] = (mpu6050_orig_data[4] << 8) | mpu6050_orig_data[5];
        }
        //计算角度
        if(((data_choose >> 0) & 1) == 1){
            if(gyro_reset_cnt != 0){
                gyro_reset_cnt--;
                gyro_centre[0] = gyro_v[0];
                gyro_centre[1] = gyro_v[1];
                gyro_centre[2] = gyro_v[2];
                pre_gyro_data[0] = 0;
                pre_gyro_data[1] = 0;
                pre_gyro_data[2] = 0;
            }
            pre_gyro_data[0] += (gyro_v[0] - gyro_centre[0])*d_t;
            pre_gyro_data[1] += (gyro_v[1] - gyro_centre[1])*d_t;
            pre_gyro_data[2] += (gyro_v[2] - gyro_centre[2])*d_t;
            switch(gyro_dimension){
                case GYRO_DIMEN_ANGLE:
                    p_gyro_data[0] = pre_gyro_data[0]*(GYRO_ADJ_ANGLE/gyro_adj_cnt[0]);
                    p_gyro_data[1] = pre_gyro_data[1]*(GYRO_ADJ_ANGLE/gyro_adj_cnt[1]);
                    p_gyro_data[2] = pre_gyro_data[2]*(GYRO_ADJ_ANGLE/gyro_adj_cnt[2]);
                    break;
                case GYRO_DIMEN_RADIAN:
                    p_gyro_data[0] = ((pre_gyro_data[0]*(GYRO_ADJ_ANGLE/gyro_adj_cnt[0]))/180.f)*PI;
                    p_gyro_data[1] = ((pre_gyro_data[1]*(GYRO_ADJ_ANGLE/gyro_adj_cnt[1]))/180.f)*PI;
                    p_gyro_data[2] = ((pre_gyro_data[2]*(GYRO_ADJ_ANGLE/gyro_adj_cnt[2]))/180.f)*PI;
                    break;
                case GYRO_DIMEN_ORIGINAL:
                    p_gyro_data[0] = pre_gyro_data[0];
                    p_gyro_data[1] = pre_gyro_data[1];
                    p_gyro_data[2] = pre_gyro_data[2];
                    break;
            }
        }
        MPU6050_DMA->CR &= ~(0x01 << 0);   //关闭DMA（为了配置DMA寄存器）
        while(MPU6050_DMA->CR&0x01);  //等待关闭完成
        DMA_ClearITPendingBit(MPU6050_DMA,DMA_IT_TCIF5);		
        if(mpu6050_cycleread_flag == 0){   //判断是否要继续读取MPU6050的数据
            return;
        }
        MPU6050_DMA->NDTR = mpu6050_data_cnt;
        MPU6050_DMA->M0AR = (u32)mpu6050_orig_data;
        mpu6050_read_start();
        tim_cnt = MPU6050_TIM->CNT;
    }
}

