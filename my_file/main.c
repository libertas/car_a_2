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
u8 g_death_turn_flag = 0;    //������־
u8 g_key1_flag = 0;
u16 g_abase_gpiof;
float g_pre_centroid_x[10] = {0};  //����ʮ�����ڵ�����
param_struct *g_param;



void can_rcv_cb(CanRxMsg *can_rx_msg){
    if(can_rx_msg->StdId == 0x05){
        ((u8 *)&g_abase_gpiof)[0] = can_rx_msg->Data[0];
        ((u8 *)&g_abase_gpiof)[1] = can_rx_msg->Data[1];
    }

}

int main(){
    //���ݶ�����
    int i,j,n;
    int signal_fturn_right = 0,signal_fturn_left = 0;  //ǿ����ת��ǿ����ת
    float centroid_x;
    float ccd_centroid_x;
    param_struct *param;
    u8 return_flag = 0;
    int tim2_cnt = 0,tim2_cnt_last = 0;   //���������������
    int control_cnt = 0;    //���ƴ���
    float tim_ms;
    u16 lcd_id;
    char str_temp[100];
    ccd_d ccd;
    u8 point_pre;
    u8 up_flag = 0;
    u8 ccd_lose_flag = 1;
    int line_width_cnt = 0;
    int step_integral;   //�������
    int downstep_integral;
    int upstep_array[STEP_LENGTH];
    int downstep_array[STEP_LENGTH];
    int vega_pos_x,vega_pos_y;
    float pos_x,pos_y,vega_angle;
    data_convert data_temp;
    u8 can_send_array[8];
    u16 can_rcv_temp;
/*********�������ݶ������ķָ���**********************/	

/*************�����Ǹ��ֳ�ʼ��********************/
    rcc_configuration();
    gpio_config();
    USART_Configuration();
/**********************�����ӡ������ӭ��Ϣ********************/
    uprintf(DEBUG_USARTx,"\n\n\n********************************************************\n");
    uprintf(DEBUG_USARTx,"               Welcome to BUPT ROBOCON!\n");
    uprintf(DEBUG_USARTx,"********************************************************\n");
    //exti_config();  //�ⲿ�ж�����
    //NVIC_Configuration();
    cmd_init();   //��ʼ���������
    fsmc_config();
    lcd_init();
    delay_ms(500);   //���ú�LCD֮��Ҫ��һ����ٸ���ʾ����ʾ����
    lcd_id = lcd_reg_read(LCD_REG_0);
    lcd_clear(0xFFFF);
    lcd_show_string(5,100,240,100,16,"hello! world!");
    
    TIM2_Configuration();
    param_init(&param);
    g_param = param;
    param_switch(0);   //С���տ�ʼ�Ĳ�������0
    ccd_init(&ccd);   //����CCD
    ccd_start();    //�����ɼ�CCD������
    can_init();
    vega_init(&vega_pos_x,&vega_pos_y,&vega_angle);
    can_add_callback(0x05,can_rcv_cb);
    can_send_msg(CAN_ID_RESET,can_send_array,1);  //���󳵷���RESETָ��
    //GPIO_SetBits(GPIOG,GPIO_Pin_13);  //ɲ��
    //GPIO_SetBits(GPIOG,GPIO_Pin_15);  //ɲ��
/************************�����Ǹ��ֳ�ʼ��****************/
    
/********************�����ǿ������ڵ�ѭ����*********************/
    while(1){
        tim2_cnt = TIM2->CNT;   //�������������
        pos_x = vega_pos_x*0.0001;
        pos_y = vega_pos_y*0.0001;
        sprintf(str_temp,"x:%.3f,y:%.3fangle:%.3f",pos_x,pos_y,vega_angle);   //��ʾ��ת�Ƕ�
        lcd_show_string(5,100,240,100,16,str_temp);

/**********************�����ǲ��Ը��ִ����*********************/ 


/**********************�����ǲ��Ը��ִ����*********************/ 


/*************************���°����Ĵ���**************************/ 
        //g_key1_flag ��Ӧ����pc10���������µ���������жϺ����︳ֵ
        if(g_key1_flag == 1){
            can_send_array[0] = 0;
            can_send_msg(CAN_ID_STOP,can_send_array,1);  //���󳵷��Ϳ�ʼ�˶�ָ��
            g_key1_flag = 0;
        }
/*************************���ϰ����Ĵ���**************************/ 
		
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
        //������������
        for(i = 0;i < 15;i++){
            lcd_set_cursor(0,320 - i - 120);  //��ͼ��֮����ʾ
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
        tim2_cnt = TIM2->CNT - tim2_cnt;  //�����������
        if(tim2_cnt < 0){
            tim2_cnt = tim2_cnt_last;   //�п��ܷ�����ʱ�����أ���ʱ��õ�ʱ���Ǹ��ģ�����
        }
        tim2_cnt_last = tim2_cnt;
        tim_ms = tim2_cnt * 0.0595;  //��ʱ����������16800��1S���һ�����ڣ����ԣ���ʱ����ÿ����ʱ״̬��1000/16800=0.0595 ms
        sprintf(str_temp,"dt:%3.2fms  ",tim_ms);
        lcd_show_string(5,140,240,100,16,str_temp);
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
