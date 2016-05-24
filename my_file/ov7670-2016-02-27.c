#include "ov7670.h"
#include "ov7670_cfg.h"
#include "tftlcd.h"


static u16 g_scl_pin_mask;
static u32 g_scl_shift_count;
u32 g_ov7670_irq_flag;
u32 g_ov7670_line_cnt;
u16 g_ov7670_cols_cnt;
char g_ov7670_image[40000];   //一万个点，大概占用 10KB
u8 g_ov2640_custom_config[100][2] = {   
	0xFF, 0x00, 
	0xDA, 0x00,
	0xD7, 0x03,
	0xDF, 0x00,
	0x33, 0x80,
	0x3C, 0x40,
	0xe1, 0x77,
	0x00, 0x00,
	0xff, 0x01,
	0x00, 0x92,    //AGC
	//自动曝光设置
//	0xff, 0x01,    
//	0x13, 0xE0,  //com8[7]:1 快速调整 0 关闭快速调整  com8[2] com8[0] 使能自动增益和自动曝光
//	0x24, 0x90,  //AEW
//	0x25, 0x78,   //AEB
//	0x26, 0xF4   //VV  快速调整的阈值
	0xFF,0x01,
	0xFF,0x01      //以连续两次0xff地址 为结束标志
	
};

int ov7670_init(){
	int i,j;
	
    //由16位引脚掩码转换成32位的引脚掩码
    g_scl_pin_mask = OV7670_SCL_PIN;
    g_scl_shift_count = 0;
    while(g_scl_pin_mask != 1){
        g_scl_shift_count += 2;
        g_scl_pin_mask = g_scl_pin_mask >> 1;
    }
		
		//PWDN = 0:  POWER DOWN模式为：工作模式
	GPIO_ResetBits(OV7670_PWDN_PORT,OV7670_PWDN_PIN);
		
		//使用摄像头之前，最好先reset一下摄像头
    ov7670_reset();
    //检查摄像头是否工作正常
//    if(ov7670_check() == -1){   
//        return -1;
//    }
//		
		
	//配置OV7670寄存器
//	ov7670_reg_write(OV7670_COM7,0x80);  //复位SCCB
	for(i = 0;i < 10;i++)
			for(j = 0;j < 1000;j++);
	//ov7670_reg_write(OV7670_COM7,0x00); 
	for(i = 0;i < 10;i++)
			for(j = 0;j < 1000;j++);
//	ov7670_reg_write(OV7670_CLKRC,0x9F);
//	ov7670_reg_write(OV7670_MANU,0x80);
//	ov7670_reg_write(OV7670_MANV,0x80);
//	ov7670_reg_write(OV7670_TSLB,0x05);   //bit[4]:0：使用通用的UV输出，1：使用固定的UV输出
//	ov7670_reg_write(OV7670_COM13,0x88);
//	ov7670_reg_write(OV7670_COM7,0x14);  //输出QVGA   输出YUV
//	ov7670_reg_write(OV7670_COM14,0x10); //bit[2:0] PCLK (说准确一点，就是一行的像素点的设置)分频 000  001   010   011   100
//	ov7670_reg_write(OV7670_SCALING_PC,0x00);
//	ov7670_reg_write(OV7670_SCALING_DCWCTR,0xF1);
	//ov7670_reg_write(OV7670_DBLV,0x82);
	for(i=0;i<sizeof(ov2640_cif_init_reg_tbl)/sizeof(ov2640_cif_init_reg_tbl[0]);i++){
	   	ov7670_reg_write(ov2640_cif_init_reg_tbl[i][0],ov2640_cif_init_reg_tbl[i][1]);
  }
	for(i=0;i<(sizeof(ov2640_yuv422_reg_tbl)/2);i++)
	{
			ov7670_reg_write(ov2640_yuv422_reg_tbl[i][0],ov2640_yuv422_reg_tbl[i][1]); 
	}
	ov2640_get_data(g_ov2640_custom_config);   //把参数从FLASH读到内存里
//	ov2640_config(g_ov2640_custom_config);
//	for(i=0;i<(sizeof(ov2640_cif_reg_tbl)/2);i++)
//	{
//			ov7670_reg_write(ov2640_cif_reg_tbl[i][0],ov2640_cif_reg_tbl[i][1]); 
//	} 
	//ov2640_imagesize_set(88,72);
//	ov2640_imagewin_set(251,5,88,72);
//	for(i = 0;i < sizeof(g_ov2640_custom_config)/sizeof(g_ov2640_custom_config[0]);i++){
//			ov7670_reg_write(g_ov2640_custom_config[i][0],g_ov2640_custom_config[i][1]);
//	}
	ov2640_window_set(251,5,352,288); //352,288
	ov2640_outsize_set(180,100);
	
		
	return 0;
}

void ov7670_reset(){
	int i,j;
	GPIO_ResetBits(OV7670_RESET_PORT,OV7670_RESET_PIN);
	for(i = 0;i < 1000;i++)
		for(j = 0;j <10000;j++);
	GPIO_SetBits(OV7670_RESET_PORT,OV7670_RESET_PIN);
	for(i = 0;i < 100;i++)
		for(j = 0;j <1000;j++);
		
}


//该函数是从FLASH读取数据到内存中的数组（只能是摄像头寄存器配置二维数组）里，读取的结束标志是，连续两次寄存器地址都是FF
void ov2640_get_data(u8 read_array[][2]){
	u32 data_read;
	u32 flash_addr = OV2640_FLASH_ADDR_START;
	int i = 0;
	u8 exit_flag = 0;
	while(1){
		data_read = *(u32 *)flash_addr;
		read_array[i][0] = data_read;  //低8位为地址
		read_array[i][1] = data_read >> 8;   //高8位为数据
		if(read_array[i][0] == 0xff && exit_flag == 1){
			return;   //连续两次0xff为退出标志
		}else if(read_array[i][0] == 0xff){
			exit_flag = 1;
		}else{
			exit_flag = 0;
		}
		i++;
		flash_addr += 4;
	}
}


//该函数是由一个二维配置数组对摄像头进行配置，结束标志是二维数组的最后两组元素的地址都是0xff
void ov2640_config(u8 reg_tb[][2]){
	int i = 0;
	u8 exit_flag = 0;
	
	while(1){
		if(reg_tb[i][0] == 0xff && exit_flag == 1){
			return;
		}else if(reg_tb[i][0] == 0xff){
			exit_flag = 1;			
		}else{
			exit_flag = 0;
		}
		ov7670_reg_write(reg_tb[i][0],reg_tb[i][1]);
		i++; 
	}
}

// 因为数据量不大，而且懒癌发作，用遍历的方式对数据进行存储
int ov2640_save_data_toram(u8 dst_array[][2],u8 reg_addr,u8 reg_data,u8 reg_type){
	int i = 0;
	u8 exit_flag = 0;
	u8 reg_type_now = 0;
	
	while(1){
		if(dst_array[i][0] == reg_addr && reg_type_now == reg_type){  //判断是否为所要存的摄像头寄存器的数据
			dst_array[i][1] = reg_data;
			return 1;
		}else{
			if(dst_array[i][0] == 0xff && exit_flag == 1){
				return -1;
			}else if(dst_array[i][0] == 0xff){
				exit_flag = 1;
				if(dst_array[i][1] == 0){
					reg_type_now = 0;
				}else{
					reg_type_now = 1;
				}
			}else{
				exit_flag = 0;
			}
			i++;
		}
		
	}
	
}


int ov7670_scl_set(u32 status){
    static u32 last_status = 1;
    u32 reg_tmp;


    if(last_status == 1 && status == 0){
        /*先把通用输出模式的输出设置成低电平，然后再从复用模式切换到通用输出
         * 模式，不然，在切换模式之后，SCL可能会被置高电平
         */

        //设置引脚为开漏输出
        reg_tmp = OV7670_SCL_PORT->OTYPER;
        reg_tmp |= OV7670_SCL_PIN;
        OV7670_SCL_PORT->OTYPER = reg_tmp;
        //SCL引脚复位，即软件拉低SCL
        OV7670_SCL_PORT->BSRRH |= OV7670_SCL_PIN;
        //设置引脚为通用输出模式
        reg_tmp = OV7670_SCL_PORT->MODER;
        reg_tmp &= ~(3<<g_scl_shift_count);
        reg_tmp |= 1<<g_scl_shift_count;
        OV7670_SCL_PORT->MODER = reg_tmp;

        last_status = status;
    }
    else if(last_status == 0 && status ==1){
        //设置引脚为通用输出模式
        reg_tmp = OV7670_SCL_PORT->MODER;
        reg_tmp &= ~(3<<g_scl_shift_count);
        reg_tmp |= 2<<g_scl_shift_count;
        OV7670_SCL_PORT->MODER = reg_tmp;
        
        last_status = status;
    }
    else{
        return -1;
    }
    return 0;
}

//由于对配置摄像头的时间要求不高，所以不用中断的方式进行I2C通信
int ov7670_reg_write(u8 reg_addr,u8 data_write){
		int i,j;
    u16 i2c_sr1_temp,i2c_sr2_temp;

    I2C_GenerateSTART(OV7670_I2Cx,ENABLE);

    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_SB) == 0);  //等待起始信号发送完毕
    I2C_ClearFlag(OV7670_I2Cx,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(OV7670_I2Cx,OV7670_I2C_ADDR,0); //发送I2C地址，且设置为写
    
    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_ADDR) == 0);  //等待地址发送完毕
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(OV7670_I2Cx,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(OV7670_I2Cx,I2C_Register_SR2);
    I2C_SendData(OV7670_I2Cx,reg_addr);  //发送寄存器地址
    
    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_BTF) == 0);//等待数据写完
    I2C_SendData(OV7670_I2Cx,data_write);   //发送数据的同时，会清除BTF的

    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_BTF) == 0);  //等待数据写完
    I2C_GenerateSTOP(OV7670_I2Cx,ENABLE);  //发送停止位
    
    return 0;
}


/* 由于对配置摄像头的时间要求不高，所以不用中断的方式进行I2C通信
 * 下面是通过硬件的I2C读取目标芯片的寄存器的一个字节:
 * 开始，发送读地址，器件应答，清ADDR前软件下拉SCL，写完NACK、STOP和
 * DR后软件再释放SCL。RxNE时读DR。
 */
int ov7670_reg_read(u8 reg_addr,u8 *data_read){
		int erro;
    u16 i2c_sr1_temp,i2c_sr2_temp;
    
    
	
		I2C_GenerateSTART(OV7670_I2Cx,ENABLE); //发出起始信号
    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_SB) == 0); //等待起始信号发送完毕
    I2C_ClearFlag(OV7670_I2Cx,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(OV7670_I2Cx,OV7670_I2C_ADDR,0);
    
    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_ADDR) ==0);  //等待I2C地址发送完毕
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(OV7670_I2Cx,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(OV7670_I2Cx,I2C_Register_SR2);
    I2C_SendData(OV7670_I2Cx,reg_addr);  //发送寄存器地址

    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_BTF) == 0);  //等待寄存器地址发送完毕
    //发出重复起始信号(该动作会使BTF位被清零),开始读数据
		I2C_GenerateSTOP(OV7670_I2Cx,ENABLE);
    I2C_GenerateSTART(OV7670_I2Cx,ENABLE);  

    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_SB) == 0); //等待起始信号发送完毕
    I2C_ClearFlag(OV7670_I2Cx,I2C_FLAG_SB);  //清除起始信号状态位
    I2C_Send7bitAddress(OV7670_I2Cx,OV7670_I2C_ADDR,1);  

    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_ADDR) ==0);  //等待I2C地址发送完毕
    erro = ov7670_scl_set(0);//软件拉低SCL(清除ADDR之前，先进行软件拉低SCL)
		if(erro < 0) return erro;
    //清ADDR(通过读取状态寄存器SR1 SR2 )
    i2c_sr1_temp = I2C_ReadRegister(OV7670_I2Cx,I2C_Register_SR1);
    i2c_sr2_temp = I2C_ReadRegister(OV7670_I2Cx,I2C_Register_SR2);
    //设置NACK
    I2C_AcknowledgeConfig(OV7670_I2Cx,DISABLE);
    //设置STOP位
    I2C_GenerateSTOP(OV7670_I2Cx,ENABLE);
    //设置好NACK位和STOP位之后，软件释放SCL
    erro = ov7670_scl_set(1);
		if(erro < 0) return erro;

    //等待目标寄存器数据接收完毕
    while(I2C_GetFlagStatus(OV7670_I2Cx,I2C_FLAG_RXNE) == 0); 
    *data_read = I2C_ReceiveData(OV7670_I2Cx);  //读I2C接收到的数据,同时BTF位被清零

    //接收好数据之后，别忘了使能ACK
    I2C_AcknowledgeConfig(OV7670_I2Cx,ENABLE);  //使能ACK
    

}


int ov7670_check(){
    u8 identify_h = 0,identify_l = 0;
    //读取产品识别号
    ov7670_reg_read(OV7670_PID,&identify_h);
    ov7670_reg_read(OV7670_VER,&identify_l);
    if(identify_h == 0x76 && identify_l == 0x73){
        return 0;  //如果产品识别号正确，返回0，否则返回-1
    }
    else{
        return -1;
    }
}


u8 ov2640_outsize_set(u16 width,u16 height)
{
	u16 outh;
	u16 outw;
	u8 temp; 
	if(width%4)return 1;
	if(height%4)return 2;
	outw=width/4;
	outh=height/4; 
	ov7670_reg_write(0XFF,0X00);	
	ov7670_reg_write(0XE0,0X04);			
	ov7670_reg_write(0X5A,outw&0XFF);		//设置OUTW的低八位
	ov7670_reg_write(0X5B,outh&0XFF);		//设置OUTH的低八位
	temp=(outw>>8)&0X03;
	temp|=(outh>>6)&0X04;
	ov7670_reg_write(0X5C,temp);				//设置OUTH/OUTW的高位 
	ov7670_reg_write(0XE0,0X00);	
	return 0;
}


void ov2640_window_set(u16 sx,u16 sy,u16 width,u16 height)
{
	u16 endx;
	u16 endy;
	u8 temp; 
	endx=sx+width/2;	//V*2
 	endy=sy+height/2;
	
	ov7670_reg_write(0XFF,0X01);			
	ov7670_reg_read(0X03,&temp);				//读取Vref之前的值
	temp&=0XF0;
	temp|=((endy&0X03)<<2)|(sy&0X03);
	ov7670_reg_write(0X03,temp);				//设置Vref的start和end的最低2位
	ov7670_reg_write(0X19,sy>>2);			//设置Vref的start高8位
	ov7670_reg_write(0X1A,endy>>2);			//设置Vref的end的高8位
	
	ov7670_reg_read(0X32,&temp);				//读取Href之前的值
	temp&=0XC0;
	temp|=((endx&0X07)<<3)|(sx&0X07);
	ov7670_reg_write(0X32,temp);				//设置Href的start和end的最低3位
	ov7670_reg_write(0X17,sx>>3);			//设置Href的start高8位
	ov7670_reg_write(0X18,endx>>3);			//设置Href的end的高8位
}


u8 ov2640_imagewin_set(u16 offx,u16 offy,u16 width,u16 height){
	u16 hsize;
	u16 vsize;
	u8 temp; 
	if(width%4)return 1;
	if(height%4)return 2;
	hsize=width/4;
	vsize=height/4;
	ov7670_reg_write(0XFF,0X00);	
	ov7670_reg_write(0XE0,0X04);					
	ov7670_reg_write(0X51,hsize&0XFF);		
	ov7670_reg_write(0X52,vsize&0XFF);		
	ov7670_reg_write(0X53,offx&0XFF);		
	ov7670_reg_write(0X54,offy&0XFF);		
	temp=(vsize>>1)&0X80;
	temp|=(offy>>4)&0X70;
	temp|=(hsize>>5)&0X08;
	temp|=(offx>>8)&0X07; 
	ov7670_reg_write(0X55,temp);				
	ov7670_reg_write(0X57,(hsize>>2)&0X80);	
	ov7670_reg_write(0XE0,0X00);	
	return 0;
} 

u8 ov2640_imagesize_set(u16 width,u16 height)
{ 
	u8 temp; 
	ov7670_reg_write(0XFF,0X00);			
	ov7670_reg_write(0XE0,0X04);			
	ov7670_reg_write(0XC0,(width)>>3&0XFF);		
	ov7670_reg_write(0XC1,(height)>>3&0XFF);		
	temp=(width&0X07)<<3;
	temp|=height&0X07;
	temp|=(width>>4)&0X80; 
	ov7670_reg_write(0X8C,temp);	
	ov7670_reg_write(0XE0,0X00);				 
	return 0;
}

//void ov2640_auto_set(){
//	ov7670_reg_write(0xff,0x01);
//	ov7670_reg_write();
//}

void ov2640_stop(){
		DCMI_Cmd(DISABLE);   //关闭DCMI
		DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN; //关闭DMA
}

void ov2640_start(){
		ov2640_stop();	//尝试关闭DCMI  尝试关闭DMA
		while(DMA2_Stream1->CR&0X01);//等待DMA2_Stream1可配置
		DMA2->LIFCR|=0X3D<<6*1;	 //清空通道1上所有中断标志
		DMA2_Stream1->NDTR = 40000;
		DMA2_Stream1->FCR=0X0000021;//设置为默认值
		DMA2_Stream1->M0AR=(uint32_t)(g_ov7670_image);//(g_ov7670_image)(0x60000000|0x0C000000|0x00000080);   //重新设置图像的地址
		g_ov7670_line_cnt = 0;
		DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;  //启动DMA
		DCMI_Cmd(ENABLE);   //启动DCMI
}

void DCMI_IRQHandler(){
		if(DCMI_GetITStatus(DCMI_IT_ERR) != RESET){
				DCMI_ClearITPendingBit(DCMI_IT_ERR);
				myprintf(USART3,"DCMI_IT_ERR\n");
		}
		if(DCMI_GetITStatus(DCMI_IT_LINE) != RESET){
				DCMI_ClearITPendingBit(DCMI_IT_LINE);
				g_ov7670_line_cnt++;
				lcd_set_cursor(g_ov7670_line_cnt,320);
				lcd_ram_write_prepare();
		}
		if(DCMI_GetITStatus(DCMI_IT_FRAME) != RESET){
				DCMI_ClearITPendingBit(DCMI_IT_FRAME);
			//	g_ov7670_cols_cnt = (20000 - DMA2_Stream1->NDTR);
				//g_ov7670_line_cnt = 0;
				DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN; //关闭DMA
				while(DMA2_Stream1->CR&0X01);//等待DMA2_Stream1可配置
				DMA2->LIFCR|=0X3D<<6*1;	 //清空通道1上所有中断标志
				DMA2_Stream1->NDTR = 40000;
				DMA2_Stream1->FCR=0X0000021;//设置为默认值
				DMA2_Stream1->M0AR=(uint32_t)(g_ov7670_image);//(g_ov7670_image)(0x60000000|0x0C000000|0x00000080);   //重新设置图像的地址
			//	lcd_set_cursor(0,320);
				g_ov7670_line_cnt = 0;
			//	lcd_ram_write_prepare();
				DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;  //启动DMA
				g_ov7670_irq_flag = 1;
				g_ov7670_cols_cnt++;
				
		}
		if(DCMI_GetITStatus(DCMI_IT_OVF) != RESET){
				DCMI_ClearITPendingBit(DCMI_IT_OVF);
				myprintf(USART3,"DCMI_IT_OVF\n");
		}
}

void DMA2_Stream1_IRQHandler(){
		if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_TCIF1) != RESET ){  //传输完成产生的中断
				DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1);
		}
		if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_HTIF1) != RESET ){
				DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_HTIF1);
		}
		if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_TEIF1) != RESET ){
				DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TEIF1);
				myprintf(USART3,"DMA_IT_TEIF1\n");
		}
		if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_DMEIF1) != RESET ){
				DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_DMEIF1);
				myprintf(USART3,"DMA_IT_DMEIF1\n");
		}
		if(DMA_GetITStatus(DMA2_Stream1,DMA_IT_FEIF1) != RESET ){
				DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_FEIF1);
				myprintf(USART3,"DMA_IT_FEIF1\n");
		}
}
