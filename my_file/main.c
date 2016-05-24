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

u8 g_exp_lock = 0;     //�ع������
u8 g_servo_lock = 0;
u8 g_death_turn_flag = 0;    //������־
float g_pre_centroid_x[10] = {0};  //����ʮ�����ڵ�����
param_struct *g_param;

int main(){
    //���ݶ�����
    int i,j,k,scanline,timeout,erro_n,n;
    int sensor_color_status;
    int hummock_cnt = 0;    //ɽ�ڼ���
    int sensor_color_antishake = 0;  //��ɫ����������
    int signal_turn_right = 0;   //����ת���ź�
    int signal_fturn_right = 0,signal_fturn_left = 0;  //ǿ����ת��ǿ����ת
    float centroid_x;
    float ccd_centroid_x;
    float dx,dx_last = 0,d_dx = 0;
    float p_gain,d_gain;
    float coder_turns = 0;   //������ת��Ȧ��
    param_struct *param;
    u32 servo_duty;
    u32 gray_sum;
    u8 return_flag = 0;
    u32 sum_x,sum_area; 
    int tim2_cnt = 0,tim2_cnt_last = 0;   //���������������
    int control_cnt = 0;    //���ƴ���
    float tim_ms;
    u16 lcd_id;
    char str_temp[100];
    image *ov7725_image;
    u32 brake_ready_cnt,brake_cnt;    //ɲ������
    u8 brake_flag = 0;   //ɲ����־  
    ccd_d ccd;
    u8 point_pre;
    u8 up_flag = 0;
    u8 ccd_lose_flag = 1;
    u8 white_flag = 0,black_flag;
    u8 ccd_tmp_flag = 0;    //ccd��ʱʹ�ñ�־
    u8 dir_adj_flag = 0;    //�������������־
    u32 up_point,down_point;  //�����ص㣬�½��ص�
    int line_width_cnt = 0;
    int step_integral;   //�������
    int downstep_integral;
    int upstep_array[STEP_LENGTH];
    int downstep_array[STEP_LENGTH];
    int pos_x,pos_y;
    float angle;
/*********�������ݶ������ķָ���**********************/	

/*************�����Ǹ��ֳ�ʼ��********************/
    rcc_configuration();
    gpio_config();
    USART_Configuration();
/**********************�����ӡ������ӭ��Ϣ********************/
    uprintf(DEBUG_USARTx,"\n\n\n********************************************************\n");
    uprintf(DEBUG_USARTx,"               Welcome to BUPT ROBOCON!\n");
    uprintf(DEBUG_USARTx,"********************************************************\n");
    exti_config();  //�ⲿ�ж�����
    NVIC_Configuration();
    cmd_init();   //��ʼ���������
    exti_color_disable(); 
    fsmc_config();
    tim3_config();
    tim3_pwm_set(100,3100);  // ��ʼ�����
    lcd_init();
    delay_ms(2000);   //���ú�LCD֮��Ҫ��һ����ٸ���ʾ����ʾ����
    lcd_id = lcd_reg_read(LCD_REG_0);
    lcd_clear(0xFFFF);
    lcd_show_string(30,100,200,100,16,"hello! world!");
//	lcd_draw_line(0,320,10000,1);
    
    TIM2_Configuration();
    tim4_config();
    param_init(&param);
    g_param = param;
    param_switch(0);   //С���տ�ʼ�Ĳ�������0
    ccd_init(&ccd);   //����CCD
    ccd_start();    //�����ɼ�CCD������
    erro_n = camera_init(&ov7725_image);   //��ʼ������ͷ
    if(erro_n < 0){
        sprintf(str_temp,"camera init error:%d\n",erro_n);
        lcd_show_string(30,220,200,100,16,str_temp);
        uprintf(DEBUG_USARTx,str_temp);
        while(1);
    }
    camera_start();   //����ͷ��ʼ�ɼ�����
    camera_reg_write(0x9c,param->threshold);
    //vega_init(&pos_x,&pos_y,&angle);
    //vega_reset();

    //GPIO_SetBits(GPIOG,GPIO_Pin_13);  //ɲ��
    //GPIO_SetBits(GPIOG,GPIO_Pin_15);  //ɲ��
/************************�����Ǹ��ֳ�ʼ��****************/
    
/********************�����ǿ������ڵ�ѭ����*********************/
    while(1){
        tim2_cnt = TIM2->CNT;   //�������������
        coder_turns += -(TIM4->CNT - 4000.f)/CODER_PERIOD;   //����������������Ȧ��
        TIM4->CNT = 4000;
        sprintf(str_temp,"coder:%3.2f",coder_turns);   //��ʾ������ת��Ȧ��
        lcd_show_string(30,80,200,100,16,str_temp);
        sprintf(str_temp,"angle:%3.2f",angle);   //��ʾ��ת�Ƕ�
        lcd_show_string(30,100,200,100,16,str_temp);
/***********************��������ɽ�ڵĲ��ִ���************************/	
        if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 1 && sensor_color_status == 0){
            if(sensor_color_antishake == 3){   //������3���������ں���ɫ��������ȡ��ֵ���ɲ��䣬���ж�Ϊ�ȶ�����ʱ�ж�Ϊ����ɽ��
                hummock_cnt++;  //ɽ������һ
                sensor_color_status = 1;
                sprintf(str_temp,"h_cnt:%d",hummock_cnt);
                lcd_show_string(150,20,200,100,16,str_temp);
                if(hummock_cnt == 3){
                    param_switch(1);  //�л�������(������������)
                    camera_reg_write(0x9C,param->threshold);
                    ccd_tmp_flag = 0;   //ֹͣCCD����ʱʹ��
                }
                if(hummock_cnt == 4){
                    param_switch(2);   //�л����ڶ������
                }
            }
            sensor_color_antishake++;  //����
        }else if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 0 && sensor_color_status == 0){
            sensor_color_antishake = 0;
        }
        if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 0 && sensor_color_status == 1){
            if(sensor_color_antishake == 3){   //������3���������ں���ɫ��������ȡ��ֵ���ɲ��䣬���ж�Ϊ�ȶ�����ʱ�ж�Ϊ�뿪ɽ��
                if(param_group_now() == 1){
                    //tim3_pwm_set(100,param->servo_p_gain);
                    //delay_ms((param->servo_d_gain) * 10);
                    dir_adj_flag = 1;
                }
                sensor_color_status = 0;
                if(hummock_cnt == 2){
                    //exti_color_enable();   //�����ж�(������ɽ�ڴ�����)
                   // ccd_tmp_flag = 1;  //��ʱʹ��CCD
                }
                if(hummock_cnt == 3){   //��ʱС������ɽ����ǡ�ý��������
                    signal_turn_right = 1;  //��������ת����ź�
                }
                if(hummock_cnt == 4){   //��ʼ����
                    camera_reg_write(0x9C,param->threshold);
                    coder_turns = 0;    //����������
                    brake_flag = 1;
                    brake_ready_cnt = 0;
                    brake_cnt = 0;
                }
            }
            sensor_color_antishake++;  //����
        }else if(GPIO_ReadInputDataBit(GPIOF,SENSOR_COLOR) == 1 && sensor_color_status == 1){
        		sensor_color_antishake = 0;
        }
/***********************��������ɽ�ڵĲ��ִ���************************/	

/**************************������ɲ���Ĵ���***************************/	
        if(brake_flag == 1){
            brake_ready_cnt++;
            brake_cnt++;
        }
        if(coder_turns >= param->brake_ready_cnt){
            GPIO_SetBits(GPIOG,GPIO_Pin_13);  //ɲ��
            GPIO_SetBits(GPIOG,GPIO_Pin_15);  //ɲ��
        }
        if(coder_turns >= param->brake_cnt){
            GPIO_ResetBits(GPIOG,GPIO_Pin_13);  //ֹͣɲ��
            GPIO_ResetBits(GPIOG,GPIO_Pin_15);  //ֹͣɲ��
        }
/**************************������ɲ���Ĵ���***************************/	
		
/*********************������С�����ҹ��Ĳ��ִ���*************************/
        if(GPIO_ReadInputDataBit(GPIOF,SENSOR_LEFT_LIGHT) == 1 ){	   //����
        //    if(param_group_now() == 1){  //������ڴ��ڵ�һ�����������������ʱ��
                signal_fturn_right = 1;    //����ǿ���ҹ��ź�
          //  }
        }else if(GPIO_ReadInputDataBit(GPIOF,SENSOR_RIGHT_LIGHT) == 1){   //�ҹ��
            //if(param_group_now() == 1){
                signal_fturn_left = 1;    //����ǿ������ź�
            //}
        }
/*********************������С�����ҹ��Ĳ��ִ���*************************/
        timeout = 50000;
        while(ov7725_image->ready == 0){ //�ȴ�֡�������,Ϊ��ͬ������ͷ
            if(timeout-- == 0)break;
        };
        ov7725_image->ready = 1;
        gray_sum = 0;
/**********************��ʾCCD������ͷ��ֵ����������ߵ�λ��******************/

        //for(i = 0;i < ov7725_image->height;i++){
        //scanline = i*(ov7725_image->width);
        //lcd_set_cursor(0,320 - i);
        //lcd_ram_write_prepare();
        //    for(j = 0;j < ov7725_image->width;j++){
        //        if((((ov7725_image->array)[(scanline + j)/8] << ((scanline + j)%8)) & 0x80) == 0x80){
        //            lcd_ram_write(0x0);  //��Ⱦ��ֵ��ͼ��
        //        }else{ 
        //            lcd_ram_write(0xffff);   //��Ⱦ��ֵ��ͼ��
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
                if((((ov7725_image->array)[(scanline + j)/8] << ((scanline + j)%8)) & 0x80) == 0x80){ //��ɫ��
                    black_flag = 1;
                    if(white_flag == 1){   //�ж�Ϊ�½�����
                        down_point = j;   //��¼�½��ص�
                        if(up_flag == 1){   //����½���֮ǰ���������أ���������֮������ǰ���
                            if((down_point - up_point > WHITE_LINE_WIDTH_MIN) && (down_point - up_point < WHITE_LINE_WIDTH_MAX)){
                                //�˿�ȷ���ǰ���
                                lcd_set_cursor(ov7725_image->width - up_point,320 - i);
                                lcd_ram_write_prepare();
                                for(k = up_point;k < down_point;k++){
                                    sum_area++;
                                    lcd_ram_write(0xffff);   //��Ⱦ��ֵ��ͼ��
                                    sum_x += k;
                                }
                                lcd_set_cursor(ov7725_image->width - j,320 - i);
                                lcd_ram_write_prepare();
                            }else{
                                lcd_ram_write(0x0);  //��Ⱦ��ֵ��ͼ��
                            }
                        }else{
                            lcd_ram_write(0x0);  //��Ⱦ��ֵ��ͼ��
                        }
                    }else{
                        lcd_ram_write(0x0);  //��Ⱦ��ֵ��ͼ��
                    }
                    white_flag = 0;
                }else{ //��ɫ��
                    white_flag = 1;
                    if(black_flag == 1){   //�ж�Ϊ������
                        up_flag = 1;
                        up_point = j;  //��¼�����ص�
                    }
                    black_flag = 0;
                    lcd_ram_write(0x0);  //��Ⱦ��ֵ��ͼ��
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
        //������������
        for(i = 0;i < 15;i++){
            lcd_set_cursor(0,320 - i - 120 - ov7725_image->height);  //��ͼ��֮����ʾ
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
                if(step_integral > param->servo_p_base){   //��ʱ��servo_p_base��ʾ�������ֵ
                    up_flag = 1;
                    line_width_cnt = WHITE_LINE_WIDTH;
                   // centroid_x = j;
                }
                if(up_flag == 1){
                    //ʶ��������֮�������WHITE_LINE_WIDTH��֮��û���½��أ����ǰ���
                    if(line_width_cnt-- == 0){   
                        up_flag = 0;
                    }
                }
                if((downstep_integral > param->servo_p_base) && (up_flag == 1)){//��ʱ��servo_p_base��ʾ�������ֵ
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
        //��������ƫ�����ĳ̶�
        dx = centroid_x - 50.0f;
        if(param_group_now() != 2){   //����������µĻ�
            centroid_x = sum_x/sum_area;
            dx = centroid_x - 90.0f;
        }
        g_pre_centroid_x[control_cnt%10] = centroid_x;   //���浱ǰ������
        //���
        d_dx = dx - dx_last;
        //PD�㷨
        p_gain = 1.0f ;
        d_gain = 1.0f ;
        servo_duty = param->servo_centroid - p_gain*param->servo_p * dx - d_gain*param->servo_d*d_dx;
/********************��ʧ���ߺ�Ļص�����********************/
        //return_flag == 1 ��ʾ�Ѷ�ʧ�˰��ߣ���Ҫ�ص����� 
//        if(sum_area < 60 ){   //�����ʧ����ص���������sum_areaΪ��ɫ������
//            return_flag = 1;
//        }
        if((ccd_lose_flag == 1 && param_group_now() == 2) || (ccd_lose_flag == 1 && ccd_tmp_flag == 1) || (dir_adj_flag == 1 && ccd_lose_flag == 1)){   //CCD�Ķ�ʧ�ص�����
            if(signal_turn_right == 1){  //signal_turn_rightΪ����ת�źţ����źŻ��ڡ���ɽ�ڵĲ��ִ��롱�б�����
                for(i = 0;i < 10;i++){
                    g_pre_centroid_x[i] = 100;   //���signal_turn_right �����ã����ڶ�ʧ����֮�����ҹ�
                }
                signal_turn_right = 0;   //ֻ��һ��
            }
             //����ڵ�ǰ�������������У���ɫ�������������Ұ벿�֣������һص�����,��������ص�����
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
            if(dir_adj_flag == 1){   //���������ֱ����,ccd��ʧ���߾ʹ�ֱ,����ֻ��ֱһ��
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
/********************��ʧ���ߺ�Ļص�����********************/

/**************************���·�ֹ�������*************************/
        //ǿ����ת��ǿ����ת�����ȼ���ߣ�
        //ǿ�ƹ�����ź��ڡ�С�����ҹ��Ĳ��ִ��롱�����õ�
//        if(signal_fturn_right == 1){
//         //   servo_duty = param->fturn_right;
//            servo_duty = 2050;
//            signal_fturn_right = 0;   //Ϩ��ǿ���ҹ��ź�
//        }else if(signal_fturn_left == 1){
//          //  servo_duty = param->fturn_left;
//            servo_duty = 1550;
//            signal_fturn_left = 0;   //Ϩ��ǿ������ź�
//        }
/**************************���·�ֹ�������************************/
        if(g_death_turn_flag == 1 && angle < g_param->servo_i){
            servo_duty = 2100; 
        }else{
            g_death_turn_flag = 0;
        }
        if(servo_duty > 4000){  //��λ
            servo_duty = 4000;
        }else if(servo_duty < 2100){ 
            servo_duty = 2100; 
        }
        if(g_servo_lock != 1){
            tim3_pwm_set(100,servo_duty);   //���ת��
        }
        sprintf(str_temp,"x:%3.2f",centroid_x);
        lcd_show_string(30,40,200,100,16,str_temp);
		sprintf(str_temp,"gpiof:%x",GPIOF->IDR);
        lcd_show_string(30,60,200,100,16,str_temp);
        tim2_cnt = TIM2->CNT - tim2_cnt;  //�����������
        if(tim2_cnt < 0){
            tim2_cnt = tim2_cnt_last;   //�п��ܷ�����ʱ�����أ���ʱ��õ�ʱ���Ǹ��ģ�����
        }
        tim2_cnt_last = tim2_cnt;
        tim_ms = tim2_cnt * 0.0595;  //��ʱ����������16800��1S���һ�����ڣ����ԣ���ʱ����ÿ����ʱ״̬��1000/16800=0.0595 ms
        sprintf(str_temp,"dt:%3.2fms  ",tim_ms);
        lcd_show_string(30,120,200,100,16,str_temp);
        if(return_flag != 1){  //����ڸÿ��������ڣ�����ͷ��ʧ�˰��ߣ����ڻص�����״̬�У������ӿ��ƴ���
            control_cnt++;  //���ƴ�����һ
        }
    }
/*******************���ϵĴ��붼���ڿ������ڵ�ѭ��������*************************/
}

//���ö����  1 ������  0�ǿ���
void servo_lock(u8 lock){
    g_servo_lock = lock;
}
