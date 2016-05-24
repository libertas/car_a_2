/**本代码使用DMA来读取MPU6050的加速度值、角速度值和温度值**/

/* 代码简要说明：
 *     使用之前，请先定义宏I2Cn为自己所使用的I2C序号。因为需要对角速度进行积分，所以还需要定时器，
 * 本代码不包含定时器的配置，所以定时器还需要使用者自己配置，配置好之后，再在这定义宏MPU6050_TIM为
 * 自己所配置的定时器(如#define MPU6050_TIM TIM1)，然后定义宏TIM_STEP_TIME为定时器的步进时间(单位是秒)。
 * 本代码可以选择IIC和中断控制器为自动配置或工程统一配置，定义宏PERIPHERAL_INIT_EN 为1即为自动配
 * 置(调用mpu6050_init()函数即可配置好)，为0则需要使用者在其它地方配置。
 *     相关宏定义好了之后，在读取MPU6050之前，需要调用函数mpu6050_init()进行初始化。该函数的入口参
 * 数类型是结构体mpu6050_init_struct，其具体成员和说明，请看结构体声明前的注释。初始化完了之后，即可
 * 调用函数mpu6050_cycleread_start(),开始不断地读取MPU6050的数据,若想停止循环读取，调用mpu6050_cycleread_stop()函数即可。
 *     定时器的配置是必须的，但是准确性不是严格要求的，准确地配置定时器可以不用校正即可输出准确的
 * 角度值。而当角度值输出不准确的时候，可以两次调用函数mpu6050_gyro_adj()来校正角度：第一次调用会使
 * 希望被校正的轴的角度归零，然后把MPU6050沿着轴旋转宏GYRO_ADJ_ANGLE所定义的角度(顺逆都行)，最后再调用该函
 * 数即可。(注意：函数mpu6050_gyro_adj一定要成对使用)
 * 
 */





#include "stm32f4xx.h"

//#include "global.h"
//#include "stm32f4xx_i2c.h"


#define PWR_MSMT_1 0x6B   //睡眠唤醒
#define SMPLRT_DIV 0x19   //采样分频寄存器
#define CONFIG 0x1A    //配置寄存器
#define GYRO_CONFIG 0x1B   //陀螺仪配置寄存器
#define ACCEL_CONFIG 0x1C   //加速度配置寄存器
#define ACCEL_XOUT_H 0x3B   //加速度高位输出
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define PI 3.1415926535898


//以下是MPU6050数据选择的宏定义
#define CHOOSE_ACCEL (1 << 2)    //数据选择
#define CHOOSE_TEMP  (1 << 1)
#define CHOOSE_GYRO  (1 << 0)
//以下是陀螺仪量纲选择的宏宏定义
#define GYRO_DIMEN_ANGLE 0   //角度
#define GYRO_DIMEN_RADIAN 1   //弧度
#define GYRO_DIMEN_ORIGINAL 2  //原始数据
//以下是陀螺仪角度校正的宏定义
#define GYRO_ADJ_ANGLE 720.f
//以下是陀螺仪轴选择的宏定义
#define MPU6050_GYRO_X 0
#define MPU6050_GYRO_Y 1
#define MPU6050_GYRO_Z 2
#define MPU6050_GYRO_ALL 3


#define I2Cn 1              //选择I2C  I2C1 PB8:SCL PB9:SDA   I2C2 PF0:SCL PF1:SDA   I2C3 PA8:SCL PC9:SDA
#define MPU6050_TIM TIM2
#define TIM_STEP_TIME 0.0000595
#define MPU6050_ADDR 0xD0
//外设初始化使能（IIC和中断控制器初始化的使能，若需要工程统一管理外设的配置，即不必自动初始化，那么可以定义为0）
//注意：DMA配置不参与工程统一管理,若想参与统一管理，请修改代码
#define PERIPHERAL_INIT_EN 1

/*
    晕死了，本来是想直接进行#define MPU6050_I2CX xxx 然后在需要选择编译的代码前使用#if MPU6050_I2CX == xxx ,
    可这样的方式预编译，编译器打死都不肯给我通过。。。所以换成了这么麻烦的预编译了。。。
*/
#if I2Cn == 1
    #define MPU6050_I2CX I2C1
    #define MPU6050_DMA DMA1_Stream5
#elif I2Cn == 2
    #define MPU6050_I2CX I2C2
    #define MPU6050_DMA DMA1_Stream2
#else
    #define MPU6050_I2CX I2C3
    #define MPU6050_DMA DMA1_Stream2
#endif


typedef struct {
    u8  data_choose;   //数据选择
    u8  gyro_dimension;  //陀螺仪量纲
    float *accel_data;
    float *gyro_data;
    float *temp_data;
}mpu6050_init_struct;


  



int mpu6050_init(mpu6050_init_struct *init_stru);
int mpu6050_fast_init(float *gyro_data);  //快速初始化,仅仅使用陀螺仪的数据,量纲是角度..
int mpu6050_reg_write(u8 reg_addr,u8 data_write);
int mpu6050_reg_read(u8 reg_addr,u8 *data_read);
int mpu6050_read_start();  //开始读数据
int mpu6050_gyro_reset(u8 gyro_reset_sel);   //复位陀螺仪角度
int mpu6050_gyro_adj(u8 gyro_adj_sel);  //陀螺仪角度调整
void mpu6050_set_gyro_adj(u8 gyro_adj_sel,float gyro_adj);  //手动设置陀螺仪校正值
float mpu6050_get_gyro_adj(u8 gyro_adj_sel); //获取当前陀螺仪校正值
u8 mpu6050_gyro_adj_status();  //获取陀螺仪校正状态（判断是否处于校正状态）
int mpu6050_cycleread_start();   //循环读取数据
int mpu6050_cycleread_stop();  //结束循环读数据

