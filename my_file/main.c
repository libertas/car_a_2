#include "stm32f4xx_it.h"
#include "main.h"
#include "configuration.h"
#include "camera.h"
#include "global.h"
#include "tftlcd.h"
#include "tracking.h"
#include "cmd.h"
#include "mpu6050.h"
#include "ccd.h"
#include "parameter.h"
#include "vega.h"
#include "can.h"


u8 g_servo_lock = 0;
u8 g_death_turn_flag = 0;    //打死标志
u8 g_key1_flag = 0;
u16 g_abase_gpiof;
float g_pre_centroid_x[10] = {0};  //保存十个周期的质心
param_struct *g_param;



void can_rcv_cb(CanRxMsg *can_rx_msg){
    if(can_rx_msg->StdId == 0x05){
        ((u8 *)&g_abase_gpiof)[0] = can_rx_msg->Data[0];
        ((u8 *)&g_abase_gpiof)[1] = can_rx_msg->Data[1];
    }

}

int main(){
    //数据定义区
    int i,j,n;
    int signal_fturn_right = 0,signal_fturn_left = 0;  //强制右转和强制左转
    float centroid_x;
    float ccd_centroid_x;
    param_struct *param;
    u8 return_flag = 0;
    int tim2_cnt = 0,tim2_cnt_last = 0;   //用作计算控制周期
    int control_cnt = 0;    //控制次数
    float tim_ms;
    u16 lcd_id;
    char str_temp[100];
    ccd_d ccd;
    u8 point_pre;
    u8 up_flag = 0;
    u8 ccd_lose_flag = 1;
    int line_width_cnt = 0;
    int step_integral;   //跳变积分
    int downstep_integral;
    int upstep_array[STEP_LENGTH];
    int downstep_array[STEP_LENGTH];
    int vega_pos_x,vega_pos_y;
    float pos_x,pos_y,vega_angle;
    data_convert data_temp;
    u8 can_send_array[8];
    u16 can_rcv_temp;
/*********这是数据定义区的分割线**********************/	

/*************以下是各种初始化********************/
    rcc_configuration();
    gpio_config();
    USART_Configuration();
/**********************下面打印开机欢迎信息********************/
    uprintf(DEBUG_USARTx,"\n\n\n********************************************************\n");
    uprintf(DEBUG_USARTx,"               Welcome to BUPT ROBOCON!\n");
    uprintf(DEBUG_USARTx,"********************************************************\n");
    //exti_config();  //外部中断设置
    //NVIC_Configuration();
    cmd_init();   //初始化命令程序
    fsmc_config();
    lcd_init();
    delay_ms(500);   //配置好LCD之后要等一会儿再给显示器显示数据
    lcd_id = lcd_reg_read(LCD_REG_0);
    lcd_clear(0xFFFF);
    lcd_show_string(5,100,240,100,16,"hello! world!");
    
    TIM2_Configuration();
    param_init(&param);
    g_param = param;
    param_switch(0);   //小车刚开始的参数组是0
    ccd_init(&ccd);   //配置CCD
    ccd_start();    //开机采集CCD的数据
    can_init();
    vega_init(&vega_pos_x,&vega_pos_y,&vega_angle);
    can_add_callback(0x05,can_rcv_cb);
    can_send_msg(CAN_ID_RESET,can_send_array,1);  //给大车发送RESET指令
    //GPIO_SetBits(GPIOG,GPIO_Pin_13);  //刹车
    //GPIO_SetBits(GPIOG,GPIO_Pin_15);  //刹车
/************************以上是各种初始化****************/
    
/********************以下是控制周期的循环体*********************/
    while(1){
        tim2_cnt = TIM2->CNT;   //计算程序周期用
        pos_x = vega_pos_x*0.0001;
        pos_y = vega_pos_y*0.0001;
        sprintf(str_temp,"x:%.3f,y:%.3fangle:%.3f",pos_x,pos_y,vega_angle);   //显示旋转角度
        lcd_show_string(5,100,240,100,16,str_temp);

/**********************以下是测试各种代码的*********************/ 


/**********************以上是测试各种代码的*********************/ 


/*************************以下按键的代码**************************/ 
        //g_key1_flag 反应的是pc10按键被按下的情况，由中断函数里赋值
        if(g_key1_flag == 1){
            can_send_array[0] = 0;
            can_send_msg(CAN_ID_STOP,can_send_array,1);  //给大车发送开始运动指令
            g_key1_flag = 0;
        }
/*************************以上按键的代码**************************/ 
		
/*********************以下是小车左右光电的部分代码*************************/
        if(GPIO_ReadInputDataBit(GPIOF,SENSOR_LEFT_LIGHT) == 1 ){	   //左光电
        //    if(param_group_now() == 1){  //如果现在处于第一组参数，即过河流的时候
                signal_fturn_right = 1;    //设置强制右拐信号
          //  }
        }else if(GPIO_ReadInputDataBit(GPIOF,SENSOR_RIGHT_LIGHT) == 1){   //右光电
            //if(param_group_now() == 1){
                signal_fturn_left = 1;    //设置强制左拐信号
            //}
        }
/*********************以上是小车左右光电的部分代码*************************/
        for(i = 0;i < 120;i++){
            lcd_set_cursor(0,320 - i);  //
            lcd_ram_write_prepare();
            n = ccd.size - 29;
            for(j = 0;j < n;j++){
                if((ccd.data[j] >> 4) > (1 - (i + 1)/120.f)*255 && (ccd.data[j] >> 4) <= (1 - i/120.f)*255){
                    lcd_ram_write(0x0);
                }else{
                    lcd_ram_write(0xffff);
                }
            }
        }
        //以下是跳变沿
        for(i = 0;i < 15;i++){
            lcd_set_cursor(0,320 - i - 120);  //在图像之下显示
            lcd_ram_write_prepare();
            step_integral = 0;
            downstep_integral = 0;
            point_pre = ccd.data[0];
            up_flag = 0;
            ccd_lose_flag = 1;
            for(j = 0;j < STEP_LENGTH;j++){
                upstep_array[j] = 0;
                downstep_array[j] = 0;
            }
            n = ccd.size - 29;
            for(j = 1;j < n;j++){
                step_integral -= upstep_array[j%STEP_LENGTH];
                downstep_integral -= downstep_array[j%STEP_LENGTH];
                upstep_array[j%STEP_LENGTH] = ccd.data[j] - ccd.data[j - 1];
                downstep_array[j%STEP_LENGTH] = ccd.data[j - 1] - ccd.data[j];
                downstep_integral += downstep_array[j%STEP_LENGTH];
                step_integral += upstep_array[j%STEP_LENGTH];
                if(step_integral > param->servo_p_base){   //暂时用servo_p_base表示跳变积分值
                    up_flag = 1;
                    line_width_cnt = WHITE_LINE_WIDTH;
                   // centroid_x = j;
                }
                if(up_flag == 1){
                    //识别上升沿之后，如果在WHITE_LINE_WIDTH点之内没有下降沿，则不是白线
                    if(line_width_cnt-- == 0){   
                        up_flag = 0;
                    }
                }
                if((downstep_integral > param->servo_p_base) && (up_flag == 1)){//暂时用servo_p_base表示跳变积分值
                    up_flag = 0;
                    ccd_lose_flag = 0;
                    ccd_centroid_x = j - STEP_LENGTH;
                }
            }
            n = ccd.size - 29;
            for(j = 0;j < n;j++){
                if(abs(j - ccd_centroid_x) < 5 && ccd_lose_flag == 0){
                    lcd_ram_write(0xffff);
                }else{
                    lcd_ram_write(0x0);
                }
            }
        }
        centroid_x = ccd_centroid_x;
        data_temp.float_form = ccd_centroid_x;
        for(i = 0;i < 4;i++){
            can_send_array[i] = data_temp.u8_form[i];
        }
        for(i = 0;i < 4;i++){
            can_send_array[i + 4] = 0;
        }
        can_send_msg(CAN_ID_CCDANDCAMERA,can_send_array,8);

        sprintf(str_temp,"x:%3.2f",centroid_x);
        lcd_show_string(5,40,240,100,16,str_temp);
		sprintf(str_temp,"gpiof:0x%x",GPIOC->IDR);
        lcd_show_string(5,60,200,100,16,str_temp);
        sprintf(str_temp,"a_gpiof:0x%x  ",g_abase_gpiof);
        lcd_show_string(5,160,240,100,16,str_temp);
        tim2_cnt = TIM2->CNT - tim2_cnt;  //计算程序周期
        if(tim2_cnt < 0){
            tim2_cnt = tim2_cnt_last;   //有可能发生计时器重载，此时算得的时间是负的，舍弃
        }
        tim2_cnt_last = tim2_cnt;
        tim_ms = tim2_cnt * 0.0595;  //计时器的周期是16800，1S完成一个周期，所以，计时器的每个计时状态是1000/16800=0.0595 ms
        sprintf(str_temp,"dt:%3.2fms  ",tim_ms);
        lcd_show_string(5,140,240,100,16,str_temp);
        if(return_flag != 1){  //如果在该控制周期内，摄像头丢失了白线，处于回调方向状态中，则不增加控制次数
            control_cnt++;  //控制次数加一
        }
    }
/*******************以上的代码都属于控制周期的循环体里面*************************/
}

//设置舵机锁  1 是上锁  0是开锁
void servo_lock(u8 lock){
    g_servo_lock = lock;
}
