#include "cmd_func.h"
#include "cmd.h"
#include "parameter.h"  //工程关联的，如果需要移植本代码，则去掉该头文件
#include "stm32f4xx.h"
#include "global.h"   //工程关联的，如果需要移植本代码，则去掉该头文件
#include "stdlib.h"
#include "string.h"
#include "tftlcd.h"
#include "mpu6050.h"
#include "camera.h"
#include "main.h"



void cmd_param_func(int argc,char *argv[]){
    float param_value;
    char param_name[PARAM_NAME_LENGTH];
    list_node *p;
    int group;
    if(argc < 2){
        uprintf(CMD_USARTx,"param <cmd>");
    }
    if(strcmp(argv[1],"print") == 0){
        param_print(param_group_now());
    }else if(strcmp(argv[1],"set") == 0){
        param_value = atof((char *)argv[3]);
        if(param_set(argv[2],param_value) < 0){
            uprintf(CMD_USARTx,"not found param name:%s",argv[2]);
        }
        if(strcmp(argv[2],"threshold") == 0){  //如果修改的是摄像头的参数，则马上给摄像头寄存器写数据
            camera_reg_write(0x9c,atoi(argv[3]));
        }
    }else if(strcmp(argv[1],"save") == 0){   //保存参数到芯片上
        param_save_to_flash();
    }else if(strcmp(argv[1],"switch") == 0){   //切换参数组
        param_switch(atoi(argv[2]));
        camera_reg_write(0x9c,(get_param_struct())->threshold);
    }else if(strcmp(argv[1],"mobile-print") == 0){
        group = atoi(argv[2]);
        p = get_param_list();
        p = p->link;
        while(p != NULL){
            uprintf(CMD_USARTx,"[P#%s#%4f]",p->data->param_name,p->data->param_value[group]);
            p = p->link;
        }
    }else{
        uprintf(CMD_USARTx,"arg not found");
        return;
    }
}

void cmd_reboot_func(int argc,char *argv[]){
    NVIC_SystemReset();
}


void cmd_sensor_func(int argc,char *argv[]){
    if(strcmp(argv[1],"read") == 0){
        if(strcmp(argv[2],"gpiof") == 0){
            uprintf(CMD_USARTx,"%x\n",GPIOG->IDR);
        }
    }
}

void cmd_pwm_func(int argc,char *argv[]){
    float f_duty;
    u32 u32_duty;
    if(strcmp(argv[1],"print") == 0){
        
    }else if(strcmp(argv[1],"set") == 0){
        if(argc == 3){
        }else if(argc == 4){
        }
    }else if(strcmp(argv[1],"setduty") == 0){ f_duty = atof(argv[2]);
        servo_lock(1);
        u32_duty = f_duty * 100;
        tim3_pwm_set(100,u32_duty);
    }
}


void cmd_gyro_func(int argc,char *argv[]){
    if(strcmp(argv[1],"adj") == 0){
        mpu6050_gyro_adj(MPU6050_GYRO_X);
    }else if(strcmp(argv[1],"start") == 0){
        mpu6050_cycleread_start();
        
    }else if(strcmp(argv[1],"stop") == 0){
        mpu6050_cycleread_stop();
    }
}

void cmd_hello_func(int argc,char *argv[]){
    lcd_show_string(30,120,200,100,16,"just test hahahah");
}
