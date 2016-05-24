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

u8 g_exp_lock = 0;     //曝光度锁定
u8 g_servo_lock = 0;
u8 g_death_turn_flag = 0;    //打死标志
float g_pre_centroid_x[10] = {0};  //保存十个周期的质心
param_struct *g_param;

int main(){
    //数据定义区
    int i,j,k,scanline,timeout,erro_n,n;
    int sensor_color_status;
    int hummock_cnt = 0;    //山岗计数
    int sensor_color_antishake = 0;  //颜色传感器防抖
    int signal_turn_right = 0;   //向右转的信号
    int signal_fturn_right = 0,signal_fturn_left = 0;  //强制右转和强制左转
    float centroid_x;
    float ccd_centroid_x;
    float dx,dx_last = 0,d_dx = 0;
    float p_gain,d_gain;
    float coder_turns = 0;   //编码器转的圈数
    param_struct *param;
    u32 servo_duty;
    u32 gray_sum;
    u8 return_flag = 0;
    u32 sum_x,sum_area; 
    int tim2_cnt = 0,tim2_cnt_last = 0;   //用作计算控制周期
    int control_cnt = 0;    //控制次数
    float tim_ms;
    u16 lcd_id;
    char str_temp[100];
    image *ov7725_image;
    u32 brake_ready_cnt,brake_cnt;    //刹车计数
    u8 brake_flag = 0;   //刹车标志  
    ccd_d ccd;
    u8 point_pre;
    u8 up_flag = 0;
    u8 ccd_lose_flag = 1;
    u8 white_flag = 0,black_flag;
    u8 ccd_tmp_flag = 0;    //ccd临时使用标志
    u8 dir_adj_flag = 0;    //河流方向调整标志
    u32 up_point,down_point;  //上升沿点，下降沿点
    int line_width_cnt = 0;
    int step_integral;   //跳变积分
    int downstep_integral;
    int upstep_array[STEP_LENGTH];
    int downstep_array[STEP_LENGTH];
    int pos_x,pos_y;
    float angle;
/*********这是数据定义区的分割线**********************/	

/*************以下是各种初始化********************/
    rcc_configuration();
    gpio_config();
    USART_Configuration();
/**********************下面打印开机欢迎信息********************/
    uprintf(DEBUG_USARTx,"\n\n\n********************************************************\n");
    uprintf(DEBUG_USARTx,"               Welcome to BUPT ROBOCON!\n");
    uprintf(DEBUG_USARTx,"********************************************************\n");
    exti_config();  //外部中断设置
    NVIC_Configuration();
    cmd_init();   //初始化命令程序
    exti_color_disable(); 
    fsmc_config();
    tim3_config();
    tim3_pwm_set(100,3100);  // 初始化舵机
    lcd_init();
    delay_ms(2000);   //配置好LCD之后要等一会儿再给显示器显示数据
    lcd_id = lcd_reg_read(LCD_REG_0);
    lcd_clear(0xFFFF);
    lcd_show_string(30,100,200,100,16,"hello! world!");
//	lcd_draw_line(0,320,10000,1);
    
    TIM2_Configuration();
    tim4_config();
    param_init(&param);
    g_param = param;
    param_switch(0);   //小车刚开始的参数组是0
    ccd_init(&ccd);   //配置CCD
    ccd_start();    //开机采集CCD的数据
    erro_n = camera_init(&ov7725_image);   //初始化摄像头
    if(erro_n < 0){
        sprintf(str_temp,"camera init error:%d\n",erro_n);
        lcd_show_string(30,220,200,100,16,str_temp);
        uprintf(DEBUG_USARTx,str_temp);
        while(1);
    }
    camera_start();   //摄像头开始采集数据
    camera_reg_write(0x9c,param->threshold);
    //vega_init(&pos_x,&pos_y,&angle);
    //vega_reset();

    //GPIO_SetBits(GPIOG,GPIO_Pin_13);  //刹车
    //GPIO_SetBits(GPIOG,GPIO_Pin_15);  //刹车
/************************以上是各种初始化****************/
    
/********************以下是控制周期的循环体*********************/
    while(1){
        tim2_cnt = TIM2->CNT;   //计算程序周期用
        coder_turns += -(TIM4->CNT - 4000.f)/CODER_PERIOD;   //计算正交编码器的圈数
        TIM4->CNT = 4000;
        sprintf(str_temp,"coder:%3.2f",coder_turns);   //显示码盘所转的圈数
        lcd_show_string(30,80,200,100,16,str_temp);
        sprintf(str_temp,"angle:%3.2f",angle);   //显示旋转角度
        lcd_show_string(30,100,200,100,16,str_temp);
/***********************以下是数山岗的部分代码************************/	
        if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 1 && sensor_color_status == 0){
            if(sensor_color_antishake == 3){   //经过了3个控制周期后，颜色传感器读取的值依旧不变，则判定为稳定，此时判定为到达山岗
                hummock_cnt++;  //山岗数加一
                sensor_color_status = 1;
                sprintf(str_temp,"h_cnt:%d",hummock_cnt);
                lcd_show_string(150,20,200,100,16,str_temp);
                if(hummock_cnt == 3){
                    param_switch(1);  //切换参数组(过河流参数组)
                    camera_reg_write(0x9C,param->threshold);
                    ccd_tmp_flag = 0;   //停止CCD的临时使用
                }
                if(hummock_cnt == 4){
                    param_switch(2);   //切换到第二组参数
                }
            }
            sensor_color_antishake++;  //防抖
        }else if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 0 && sensor_color_status == 0){
            sensor_color_antishake = 0;
        }
        if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 0 && sensor_color_status == 1){
            if(sensor_color_antishake == 3){   //经过了3个控制周期后，颜色传感器读取的值依旧不变，则判定为稳定，此时判定为离开山岗
                if(param_group_now() == 1){
                    //tim3_pwm_set(100,param->servo_p_gain);
                    //delay_ms((param->servo_d_gain) * 10);
                    dir_adj_flag = 1;
                }
                sensor_color_status = 0;
                if(hummock_cnt == 2){
                    //exti_color_enable();   //开启中断(第三个山岗打死弯)
                   // ccd_tmp_flag = 1;  //临时使用CCD
                }
                if(hummock_cnt == 3){   //此时小车走完山岗三恰好进入河流段
                    signal_turn_right = 1;  //设置向右转弯的信号
                }
                if(hummock_cnt == 4){   //开始下坡
                    camera_reg_write(0x9C,param->threshold);
                    coder_turns = 0;    //编码器清零
                    brake_flag = 1;
                    brake_ready_cnt = 0;
                    brake_cnt = 0;
                }
            }
            sensor_color_antishake++;  //防抖
        }else if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 1 && sensor_color_status == 1){
        		sensor_color_antishake = 0;
        }
/***********************以上是数山岗的部分代码************************/	

/**************************以下是刹车的代码***************************/	
        if(brake_flag == 1){
            brake_ready_cnt++;
            brake_cnt++;
        }
        if(coder_turns >= param->brake_ready_cnt){
            GPIO_SetBits(GPIOG,GPIO_Pin_13);  //刹车
            GPIO_SetBits(GPIOG,GPIO_Pin_15);  //刹车
        }
        if(coder_turns >= param->brake_cnt){
            GPIO_ResetBits(GPIOG,GPIO_Pin_13);  //停止刹车
            GPIO_ResetBits(GPIOG,GPIO_Pin_15);  //停止刹车
        }
/**************************以上是刹车的代码***************************/	
		
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
        timeout = 50000;
        while(ov7725_image->ready == 0){ //等待帧传输完成,为了同步摄像头
            if(timeout-- == 0)break;
        };
        ov7725_image->ready = 1;
        gray_sum = 0;
/**********************显示CCD和摄像头数值，而且算白线的位置******************/

        //for(i = 0;i < ov7725_image->height;i++){
        //scanline = i*(ov7725_image->width);
        //lcd_set_cursor(0,320 - i);
        //lcd_ram_write_prepare();
        //    for(j = 0;j < ov7725_image->width;j++){
        //        if((((ov7725_image->array)[(scanline + j)/8] << ((scanline + j)%8)) & 0x80) == 0x80){
        //            lcd_ram_write(0x0);  //渲染二值化图像
        //        }else{ 
        //            lcd_ram_write(0xffff);   //渲染二值化图像
        //        }
        //    }
        //}
        
        sum_area = 0;
        sum_x = 0;
        for(i = 0;i < ov7725_image->height;i++){
            scanline = i*(ov7725_image->width);
            up_flag = 0;
            white_flag = 0;
            black_flag = 0;
            lcd_set_cursor(ov7725_image->width,320 - i);
            lcd_ram_write_prepare();
            for(j = 0;j < ov7725_image->width;j++){
                if((((ov7725_image->array)[(scanline + j)/8] << ((scanline + j)%8)) & 0x80) == 0x80){ //黑色的
                    black_flag = 1;
                    if(white_flag == 1){   //判断为下降升沿
                        down_point = j;   //记录下降沿点
                        if(up_flag == 1){   //如果下降沿之前，有上升沿，则两个沿之间可能是白线
                            if((down_point - up_point > WHITE_LINE_WIDTH_MIN) && (down_point - up_point < WHITE_LINE_WIDTH_MAX)){
                                //此刻确定是白线
                                lcd_set_cursor(ov7725_image->width - up_point,320 - i);
                                lcd_ram_write_prepare();
                                for(k = up_point;k < down_point;k++){
                                    sum_area++;
                                    lcd_ram_write(0xffff);   //渲染二值化图像
                                    sum_x += k;
                                }
                                lcd_set_cursor(ov7725_image->width - j,320 - i);
                                lcd_ram_write_prepare();
                            }else{
                                lcd_ram_write(0x0);  //渲染二值化图像
                            }
                        }else{
                            lcd_ram_write(0x0);  //渲染二值化图像
                        }
                    }else{
                        lcd_ram_write(0x0);  //渲染二值化图像
                    }
                    white_flag = 0;
                }else{ //白色的
                    white_flag = 1;
                    if(black_flag == 1){   //判断为上升沿
                        up_flag = 1;
                        up_point = j;  //记录上升沿点
                    }
                    black_flag = 0;
                    lcd_ram_write(0x0);  //渲染二值化图像
                }
            }
        }
        for(i = 0;i < 120;i++){
            lcd_set_cursor(0,320 - i - ov7725_image->height);  //
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
            lcd_set_cursor(0,320 - i - 120 - ov7725_image->height);  //在图像之下显示
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
        //计算质心偏离中心程度
        dx = centroid_x - 50.0f;
        if(param_group_now() != 2){   //如果不是下坡的话
            centroid_x = sum_x/sum_area;
            dx = centroid_x - 90.0f;
        }
        g_pre_centroid_x[control_cnt%10] = centroid_x;   //保存当前的质心
        //差分
        d_dx = dx - dx_last;
        //PD算法
        p_gain = 1.0f ;
        d_gain = 1.0f ;
        servo_duty = param->servo_centroid - p_gain*param->servo_p * dx - d_gain*param->servo_d*d_dx;
/********************丢失白线后的回调策略********************/
        //return_flag == 1 表示已丢失了白线，需要回调方向 
//        if(sum_area < 60 ){   //如果丢失，则回调方向，其中sum_area为白色像素数
//            return_flag = 1;
//        }
        if((ccd_lose_flag == 1 && param_group_now() == 2) || (ccd_lose_flag == 1 && ccd_tmp_flag == 1) || (dir_adj_flag == 1 && ccd_lose_flag == 1)){   //CCD的丢失回调策略
            if(signal_turn_right == 1){  //signal_turn_right为向右转信号，该信号会在“数山岗的部分代码”中被设置
                for(i = 0;i < 10;i++){
                    g_pre_centroid_x[i] = 100;   //如果signal_turn_right 被设置，则在丢失白线之后，往右拐
                }
                signal_turn_right = 0;   //只拐一次
            }
             //如果在第前三个控制周期中，白色区域质心落在右半部分，则往右回调方向,否则往左回调方向
            if(g_pre_centroid_x[(control_cnt - 3)%10] > 50){ 
                servo_duty = param->return_right;
                if(ccd_tmp_flag == 1){
                    servo_duty = 550;
                }
            }else{
                servo_duty = param->return_left;
                if(ccd_tmp_flag == 1){
                    servo_duty = 1500;
                }
            }
            if(dir_adj_flag == 1){   //河流方向打直策略,ccd丢失白线就打直,而且只打直一次
                dir_adj_flag = 0;
                tim3_pwm_set(100,param->servo_p_gain);
                delay_ms((param->servo_d_gain) * 10);
            }
            return_flag = 1;
        }else if(param_group_now() != 2 && sum_area < WHITE_AREA_SIZE_MIN){
            if(g_pre_centroid_x[(control_cnt - 3)%10] > 90){ 
                servo_duty = param->return_right;
            }else{
                servo_duty = param->return_left;
            }
            
            return_flag = 1;
        }else{
            return_flag = 0;
        }
/********************丢失白线后的回调策略********************/

/**************************下坡防止掉落策略*************************/
        //强制右转或强制左转（优先级最高）
        //强制拐弯的信号在“小车左右光电的部分代码”中设置的
//        if(signal_fturn_right == 1){
//         //   servo_duty = param->fturn_right;
//            servo_duty = 2050;
//            signal_fturn_right = 0;   //熄灭强制右拐信号
//        }else if(signal_fturn_left == 1){
//          //  servo_duty = param->fturn_left;
//            servo_duty = 1550;
//            signal_fturn_left = 0;   //熄灭强制左拐信号
//        }
/**************************下坡防止掉落策略************************/
        if(g_death_turn_flag == 1 && angle < g_param->servo_i){
            servo_duty = 2100; 
        }else{
            g_death_turn_flag = 0;
        }
        if(servo_duty > 4000){  //限位
            servo_duty = 4000;
        }else if(servo_duty < 2100){ 
            servo_duty = 2100; 
        }
        if(g_servo_lock != 1){
            tim3_pwm_set(100,servo_duty);   //舵机转向
        }
        sprintf(str_temp,"x:%3.2f",centroid_x);
        lcd_show_string(30,40,200,100,16,str_temp);
		sprintf(str_temp,"gpiof:%x",GPIOF->IDR);
        lcd_show_string(30,60,200,100,16,str_temp);
        tim2_cnt = TIM2->CNT - tim2_cnt;  //计算程序周期
        if(tim2_cnt < 0){
            tim2_cnt = tim2_cnt_last;   //有可能发生计时器重载，此时算得的时间是负的，舍弃
        }
        tim2_cnt_last = tim2_cnt;
        tim_ms = tim2_cnt * 0.0595;  //计时器的周期是16800，1S完成一个周期，所以，计时器的每个计时状态是1000/16800=0.0595 ms
        sprintf(str_temp,"dt:%3.2fms  ",tim_ms);
        lcd_show_string(30,120,200,100,16,str_temp);
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
