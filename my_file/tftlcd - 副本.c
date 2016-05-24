/****************************************
2016/2/27  增加了ILI9341显示屏的代码
****************************************/




#include "tftlcd.h"
#include "FONT.H"


u16 g_lcd_height,g_lcd_width;
u16 g_lcd_dir;
//LCD寄存器值读函数
u16 lcd_reg_read(u16 lcd_reg_addr){
    LCD->LCD_REG = lcd_reg_addr;

    return (LCD->LCD_RAM);
}
//LCD寄存机值写函数
void lcd_reg_write(u16 lcd_reg_addr,u16 lcd_reg_data){
    LCD->LCD_REG = lcd_reg_addr;
    LCD->LCD_RAM = lcd_reg_data;

}

void lcd_init(){
    u16 lcd_id;

    lcd_id = lcd_reg_read(0x00);  //获取LCD的ID号，来做相应的初始化
	
		 if(lcd_id == 0X9341||lcd_id == 0X5310||lcd_id == 0X5510||lcd_id == 0X1963){//如果是这几个IC,则设置WR时序为最快
		
				//重新配置写时序控制寄存器的时序   	 							    
				FSMC_Bank1E->BWTR[6]&=~(0XF<<0);//地址建立时间(ADDSET)清零 	 
				FSMC_Bank1E->BWTR[6]&=~(0XF<<8);//数据保存时间清零
				FSMC_Bank1E->BWTR[6]|=3<<0;		//地址建立时间(ADDSET)为3个HCLK =18ns  	 
				FSMC_Bank1E->BWTR[6]|=2<<8; 	//数据保存时间(DATAST)为6ns*3个HCLK=18ns
		}
		else if(lcd_id == 0X6804||lcd_id == 0XC505){	//6804/C505速度上不去,得降低
		
				//重新配置写时序控制寄存器的时序   	 							    
				FSMC_Bank1E->BWTR[6]&=~(0XF<<0);//地址建立时间(ADDSET)清零 	 
				FSMC_Bank1E->BWTR[6]&=~(0XF<<8);//数据保存时间清零
				FSMC_Bank1E->BWTR[6]|=10<<0;	//地址建立时间(ADDSET)为10个HCLK =60ns  	 
				FSMC_Bank1E->BWTR[6]|=12<<8; 	//数据保存时间(DATAST)为6ns*13个HCLK=78ns
		}

    if(lcd_id == 0x9325){
//          lcd_reg_write(LCD_REG_0, 0x0001); /* Start internal OSC. */
//          lcd_reg_write(LCD_REG_1, 0x0100); /* Set SS and SM bit */
//          lcd_reg_write(LCD_REG_2, 0x0700); /* Set 1 line inversion */
//          lcd_reg_write(LCD_REG_3, 0x1018); /* Set GRAM write direction and BGR=1. */
//          lcd_reg_write(LCD_REG_4, 0x0000); /* Resize register */
//          lcd_reg_write(LCD_REG_8, 0x0202); /* Set the back porch and front porch */
//          lcd_reg_write(LCD_REG_9, 0x0000); /* Set non-display area refresh cycle ISC[3:0] */
//          lcd_reg_write(LCD_REG_10, 0x0000); /* FMARK function */
//          lcd_reg_write(LCD_REG_12, 0x0000); /* RGB interface setting */
//          lcd_reg_write(LCD_REG_13, 0x0000); /* Frame marker Position */
//          lcd_reg_write(LCD_REG_15, 0x0000); /* RGB interface polarity */
// 
//          /* Power On sequence -----------------------------------------------------*/
//          lcd_reg_write(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
//          lcd_reg_write(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
//          lcd_reg_write(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
//          lcd_reg_write(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude */
//          delay_ms(2000);                      /* Dis-charge capacitor power voltage (200ms) */
//          lcd_reg_write(LCD_REG_16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
//          lcd_reg_write(LCD_REG_17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
//          delay_ms(500);                       /* Delay 50 ms */
//          lcd_reg_write(LCD_REG_18, 0x0139); /* VREG1OUT voltage */
//          delay_ms(500);                       /* Delay 50 ms */
//          lcd_reg_write(LCD_REG_19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
//          lcd_reg_write(LCD_REG_41, 0x0013); /* VCM[4:0] for VCOMH */
//          delay_ms(500);                       /* Delay 50 ms */
//          lcd_reg_write(LCD_REG_32, 0x0000); /* GRAM horizontal Address */
//          lcd_reg_write(LCD_REG_33, 0x0000); /* GRAM Vertical Address */
// 
//          /* Adjust the Gamma Curve (ILI9325)---------------------------------------*/
//          lcd_reg_write(LCD_REG_48, 0x0007);
//          lcd_reg_write(LCD_REG_49, 0x0302);
//          lcd_reg_write(LCD_REG_50, 0x0105);
//          lcd_reg_write(LCD_REG_53, 0x0206);
//          lcd_reg_write(LCD_REG_54, 0x0808);
//          lcd_reg_write(LCD_REG_55, 0x0206);
//          lcd_reg_write(LCD_REG_56, 0x0504);
//          lcd_reg_write(LCD_REG_57, 0x0007);
//          lcd_reg_write(LCD_REG_60, 0x0105);
//          lcd_reg_write(LCD_REG_61, 0x0808);
// 
//          /* Set GRAM area ---------------------------------------------------------*/
//          lcd_reg_write(LCD_REG_80, 0x0000); /* Horizontal GRAM Start Address */
//          lcd_reg_write(LCD_REG_81, 0x00EF); /* Horizontal GRAM End Address */
//          lcd_reg_write(LCD_REG_82, 0x0000); /* Vertical GRAM Start Address */
//          lcd_reg_write(LCD_REG_83, 0x013F); /* Vertical GRAM End Address */
// 
//          lcd_reg_write(LCD_REG_96,  0xA700); /* Gate Scan Line(GS=1, scan direction is G320~G1) */
//          lcd_reg_write(LCD_REG_97,  0x0001); /* NDL,VLE, REV */
//          lcd_reg_write(LCD_REG_106, 0x0000); /* set scrolling line */
// 
//          /* Partial Display Control -----------------------------------------------*/
//          lcd_reg_write(LCD_REG_128, 0x0000);
//          lcd_reg_write(LCD_REG_129, 0x0000);
//          lcd_reg_write(LCD_REG_130, 0x0000);
//          lcd_reg_write(LCD_REG_131, 0x0000);
//          lcd_reg_write(LCD_REG_132, 0x0000);
//          lcd_reg_write(LCD_REG_133, 0x0000);
// 
//          /* Panel Control ---------------------------------------------------------*/
//          lcd_reg_write(LCD_REG_144, 0x0010);
//          lcd_reg_write(LCD_REG_146, 0x0000);
//          lcd_reg_write(LCD_REG_147, 0x0003);
//          lcd_reg_write(LCD_REG_149, 0x0110);
//          lcd_reg_write(LCD_REG_151, 0x0000);
//          lcd_reg_write(LCD_REG_152, 0x0000);
// 
//          /* set GRAM write direction and BGR = 1 */
//          /* I/D=00 (Horizontal : increment, Vertical : decrement) */
//          /* AM=1 (address is updated in vertical writing direction) */
//          lcd_reg_write(LCD_REG_3, 0x1018);
// 
//          lcd_reg_write(LCD_REG_7, 0x0133); /* 262K color and display ON */ 
						lcd_reg_write(0x00E5,0x78F0); 
						lcd_reg_write(0x0001,0x0100); 
						lcd_reg_write(0x0002,0x0700); 
						lcd_reg_write(0x0003,0x1018); 
						lcd_reg_write(0x0004,0x0000); 
						lcd_reg_write(0x0008,0x0202);  
						lcd_reg_write(0x0009,0x0000);
						lcd_reg_write(0x000A,0x0000); 
						lcd_reg_write(0x000C,0x0000); 
						lcd_reg_write(0x000D,0x0000);
						lcd_reg_write(0x000F,0x0000);
						//power on sequence VGHVGL
						lcd_reg_write(0x0010,0x0000);   
						lcd_reg_write(0x0011,0x0007);  
						lcd_reg_write(0x0012,0x0000);  
						lcd_reg_write(0x0013,0x0000); 
						lcd_reg_write(0x0007,0x0000); 
						//vgh 
						lcd_reg_write(0x0010,0x1690);   
						lcd_reg_write(0x0011,0x0227);
						//delayms(100);
						//vregiout 
						lcd_reg_write(0x0012,0x009D); //0x001b
						//delayms(100); 
						//vom amplitude
						lcd_reg_write(0x0013,0x1900);
						//delayms(100); 
						//vom H
						lcd_reg_write(0x0029,0x0025); 
						lcd_reg_write(0x002B,0x000D); 
						//gamma
						lcd_reg_write(0x0030,0x0007);
						lcd_reg_write(0x0031,0x0303);
						lcd_reg_write(0x0032,0x0003);// 0006
						lcd_reg_write(0x0035,0x0206);
						lcd_reg_write(0x0036,0x0008);
						lcd_reg_write(0x0037,0x0406); 
						lcd_reg_write(0x0038,0x0304);//0200
						lcd_reg_write(0x0039,0x0007); 
						lcd_reg_write(0x003C,0x0602);// 0504
						lcd_reg_write(0x003D,0x0008); 
						//ram
						lcd_reg_write(0x0050,0x0000); 
						lcd_reg_write(0x0051,0x00EF);
						lcd_reg_write(0x0052,0x0000); 
						lcd_reg_write(0x0053,0x013F);  
						lcd_reg_write(0x0060,0xA700); 
						lcd_reg_write(0x0061,0x0001); 
						lcd_reg_write(0x006A,0x0000); 
						//
						lcd_reg_write(0x0080,0x0000); 
						lcd_reg_write(0x0081,0x0000); 
						lcd_reg_write(0x0082,0x0000); 
						lcd_reg_write(0x0083,0x0000); 
						lcd_reg_write(0x0084,0x0000); 
						lcd_reg_write(0x0085,0x0000); 
						//
						lcd_reg_write(0x0090,0x0010); 
						lcd_reg_write(0x0092,0x0600); 
						
						lcd_reg_write(0x0007,0x0133);
						lcd_reg_write(0x00,0x0022);//
    }
	else if(lcd_id == 0x9320){
	     /* Start Initial Sequence ------------------------------------------------*/
          lcd_reg_write(LCD_REG_229,0x8000); /* Set the internal vcore voltage */
          lcd_reg_write(LCD_REG_0,  0x0001); /* Start internal OSC. */
          lcd_reg_write(LCD_REG_1,  0x0100); /* set SS and SM bit */
          lcd_reg_write(LCD_REG_2,  0x0700); /* set 1 line inversion */
          lcd_reg_write(LCD_REG_3,  0x1030); /* set GRAM write direction and BGR=1. */
          lcd_reg_write(LCD_REG_4,  0x0000); /* Resize register */
          lcd_reg_write(LCD_REG_8,  0x0202); /* set the back porch and front porch */
          lcd_reg_write(LCD_REG_9,  0x0000); /* set non-display area refresh cycle ISC[3:0] */
          lcd_reg_write(LCD_REG_10, 0x0000); /* FMARK function */
          lcd_reg_write(LCD_REG_12, 0x0000); /* RGB interface setting */
          lcd_reg_write(LCD_REG_13, 0x0000); /* Frame marker Position */
          lcd_reg_write(LCD_REG_15, 0x0000); /* RGB interface polarity */

          /* Power On sequence -----------------------------------------------------*/
          lcd_reg_write(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
          lcd_reg_write(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
          lcd_reg_write(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
          lcd_reg_write(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude */
          delay_ms(2000);                 /* Dis-charge capacitor power voltage (200ms) */
          lcd_reg_write(LCD_REG_16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
          lcd_reg_write(LCD_REG_17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
          delay_ms(500);                  /* Delay 50 ms */
          lcd_reg_write(LCD_REG_18, 0x0139); /* VREG1OUT voltage */
          delay_ms(500);                  /* Delay 50 ms */
          lcd_reg_write(LCD_REG_19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
          lcd_reg_write(LCD_REG_41, 0x0013); /* VCM[4:0] for VCOMH */
          delay_ms(500);                  /* Delay 50 ms */
          lcd_reg_write(LCD_REG_32, 0x0000); /* GRAM horizontal Address */
          lcd_reg_write(LCD_REG_33, 0x0000); /* GRAM Vertical Address */

          /* Adjust the Gamma Curve ------------------------------------------------*/
          lcd_reg_write(LCD_REG_48, 0x0007);
          lcd_reg_write(LCD_REG_49, 0x0007);
          lcd_reg_write(LCD_REG_50, 0x0007);
          lcd_reg_write(LCD_REG_53, 0x0007);
          lcd_reg_write(LCD_REG_54, 0x0007);
          lcd_reg_write(LCD_REG_55, 0x0700);
          lcd_reg_write(LCD_REG_56, 0x0700);
          lcd_reg_write(LCD_REG_57, 0x0700);
          lcd_reg_write(LCD_REG_60, 0x0700);
          lcd_reg_write(LCD_REG_61, 0x1F00);
        
          /* Set GRAM area ---------------------------------------------------------*/
          lcd_reg_write(LCD_REG_80, 0x0000); /* Horizontal GRAM Start Address */
          lcd_reg_write(LCD_REG_81, 0x00EF); /* Horizontal GRAM End Address */
          lcd_reg_write(LCD_REG_82, 0x0000); /* Vertical GRAM Start Address */
          lcd_reg_write(LCD_REG_83, 0x013F); /* Vertical GRAM End Address */
          lcd_reg_write(LCD_REG_96,  0x2700); /* Gate Scan Line */
          lcd_reg_write(LCD_REG_97,  0x0001); /* NDL,VLE, REV */
          lcd_reg_write(LCD_REG_106, 0x0000); /* set scrolling line */

          /* Partial Display Control -----------------------------------------------*/
          lcd_reg_write(LCD_REG_128, 0x0000);
          lcd_reg_write(LCD_REG_129, 0x0000);
          lcd_reg_write(LCD_REG_130, 0x0000);
          lcd_reg_write(LCD_REG_131, 0x0000);
          lcd_reg_write(LCD_REG_132, 0x0000);
          lcd_reg_write(LCD_REG_133, 0x0000);

          /* Panel Control ---------------------------------------------------------*/
          lcd_reg_write(LCD_REG_144, 0x0010);
          lcd_reg_write(LCD_REG_146, 0x0000);
          lcd_reg_write(LCD_REG_147, 0x0003);
          lcd_reg_write(LCD_REG_149, 0x0110);
          lcd_reg_write(LCD_REG_151, 0x0000);
          lcd_reg_write(LCD_REG_152, 0x0000);

          /* Set GRAM write direction and BGR = 1 */
          /* I/D=01 (Horizontal : increment, Vertical : decrement) */
          /* AM=1 (address is updated in vertical writing direction) */
          lcd_reg_write(LCD_REG_3, 0x1018);

          lcd_reg_write(LCD_REG_7, 0x0173); /* 262K color and display ON */ 	
		}
		else if(lcd_id == 0x9341){
					
		}
		GPIO_SetBits(GPIOB,GPIO_Pin_15);  //背光亮起
		g_lcd_dir = LCD_DIR;
		lcd_clear(BACK_COLOR);  //设置背景颜色
		if(g_lcd_dir == 1){  //竖屏
			g_lcd_height = 320;
			g_lcd_width = 240;
			
		}
}

//给GRAM写入数据
void lcd_ram_write(u16 ram_data){
    LCD->LCD_RAM = ram_data;
}

//设置画笔坐标
void lcd_set_cursor(u16 x_pos,u16 y_pos){
    lcd_reg_write(LCD_REG_32,x_pos);
    lcd_reg_write(LCD_REG_33,y_pos);

}
//GRAM地址自增设置
void lcd_ram_write_prepare(){
    LCD->LCD_REG = LCD_REG_34;
}
//画点
void lcd_draw_point(u16 x,u16 y,u16 color){

		lcd_set_cursor(x,y);
		lcd_ram_write_prepare();
		lcd_ram_write(color);
}

//画线
void lcd_draw_line(u16 x,u16 y,u16 lenth,u8 dir){
    u32 i = 0;

    lcd_set_cursor(x,y);
    if(dir == 1){
       lcd_ram_write_prepare();  //设置成GRAM自增
			 for(i = 0;i < lenth;i++){
					lcd_ram_write(0x0000);
				}
	}
}

//清屏，，color为清屏颜色
void lcd_clear(u16 color){
    u32 i;
    lcd_set_cursor(0,0);
    lcd_ram_write_prepare();
    for(i = 0;i < 76800;i++){
        lcd_ram_write(color);
    }

}



//void lcd_show_char(u16 x,u16 y,u8 num,u8 size,u8 mode)
//{  							  
//  u8 temp,t1,t;
//	u16 y0 = y;
//	u8 csize=(size/8+((size%8)?1:0))*(size/2);		
// 	num=num-' ';
//	for(t=0;t<csize;t++)
//	{   
//		if(size==12)temp=asc2_1206[num][t]; 	 	
//		else if(size==16)temp=asc2_1608[num][t];	
//		else if(size==24)temp=asc2_2412[num][t];	
//		else return;								
//		for(t1=0;t1<8;t1++)
//		{			    
//			if(temp&0x80)lcd_draw_point(x,y,POINT_COLOR);
//			else if(mode==0)lcd_draw_point(x,y,BACK_COLOR);
//			temp<<=1;
//			y++;
//			if(y > 320)return;		//
//			if((y0-y)==size)
//			{
//				y=y0;
//				x++;
//				if(x >= 240)return;	//
//				break;
//			}
//		}  	 
//	}  	    	   	 	  
//}   

void lcd_show_char(u16 x,u16 y,char num,u8 size,u8 mode)
{  							  
    u8 temp,t1,t;
	u16 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//得到字体一个字符对应点阵集所占的字节数	
 	num=num-' ';//得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//调用1206字体
		else if(size==16)temp=asc2_1608[num][t];	//调用1608字体
		else if(size==24)temp=asc2_2412[num][t];	//调用2412字体
		else return;								//没有的字库
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)lcd_draw_point(x,y,POINT_COLOR);
			else if(mode==0)lcd_draw_point(x,y,BACK_COLOR);
			temp<<=1;
			y++;
			if(y>=320)return;		//超区域了
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=240)return;	//超区域了
				break;
			}
		}  	 
	}  	    	   	 	  
}   
	   	 	  



void lcd_show_string(u16 x,u16 y,u16 width,u16 height,u8 size,char *p)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        lcd_show_char(x,y,*p,size,0);
        x+=size/2;
        p++;
		}
}
