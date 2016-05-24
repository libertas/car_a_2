/********************本代码功能是采集摄像头数据********************************/
/*
 */

#include "stm32f4xx.h"

#define CAMERA_OV2640 1
#define CAMERA_OV7620 2
#define CAMERA_OV7725 3
#define CAMERA_I2CX I2C1   //摄像头使用的I2C
#define CAMERA_TYPE CAMERA_OV7725   //摄像头的类型,摄像头的类型有ov7620 ov7670 ov2640
#define CAMERA_DMA DMA2_Stream1   //DMA
#define CAMERA_DMA_CHANNEL DMA_Channel_6   //DMA通道
#define PERIPHERAL_INIT_EN 1

//以下是图像类型
#define IMAGE_TYPE_BIN_THRESH 0         //二值化
#define IMAGE_TYPE_GRAY 1              //灰度
#define IMAGE_TYPE_YUYV 2               //YUYV格式

#if CAMERA_TYPE == CAMERA_OV2640
    #define CAMERA_I2C_ADDR 0X60   //I2C地址
    #define IMAGE_DEFAULT_WIDTH 180
    #define IMAGE_DEFAULT_HEIGHT 100
    #define IMAGE_DEFAULT_TYPE IMAGE_TYPE_YUYV
#elif CAMERA_TYPE == CAMERA_OV7620
    #define CAMERA_I2C_ADDR 0x43   //I2C地址
    #define IMAGE_DEFAULT_WIDTH 80
    #define IMAGE_DEFAULT_HEIGHT 60
    #define IMAGE_DEFAULT_TYPE IMAGE_TYPE_YUYV
#elif CAMERA_TYPE == CAMERA_OV7725
    #define CAMERA_I2C_ADDR 0x42   //I2C地址
    #define IMAGE_DEFAULT_WIDTH 160
    #define IMAGE_DEFAULT_HEIGHT 120
    #define IMAGE_DEFAULT_TYPE IMAGE_TYPE_BIN_THRESH
#endif

//表示图像的数据结构
//该数据结构是不断地被DMA改写，而使用者也要不断地要读该数据结构的图像数据
//即此为经典的作者和读者问题。
//在此我并不使用锁机制（因为DMA不能错过每一帧图像的读取），而设置一个标志
//变量ready，让使用者知道DMA改写数据的空档期，以在空档期的时候读取图像数据
typedef struct {
    u32 width;   //图像的宽度
    u32 height;   //图像的高度
    u32 size;    //图像的大小
    u8 *array;   //存储图像数据的数组
    u8 type;     //图像类型
    u8 ready;   //就绪标志，刚被置1时表示DMA刚好传输完数据，距离下次读数据
                 //还有一段时间（空档期），此时最适合读取图像数据
    u32 fps;     //如果image数据结构为采集图像的缓冲区的话，则fps表示帧率
}image;

//typedef struct{


//}camera_init_struct;

//以下是摄像头通用的函数
int camera_reg_write(u8 reg_addr,u8 data_write);
int camera_reg_read(u8 reg_addr,u8 *data_read);
int camera_init(image **pp_image);
int camera_create_image(image **pp_image,u32 width,u32 height,u8 type);  //创建图像
int camera_start();   //开始采集摄像头数据
int camera_stop();    //停止采集摄像头数据
int camera_config(u8 reg_tbl[][2]);

//ov2640用到的函数
u8 ov2640_outsize_set(u16 width,u16 height);
void ov2640_window_set(u16 sx,u16 sy,u16 width,u16 height);
u8 ov2640_imagewin_set(u16 offx,u16 offy,u16 width,u16 height);
u8 ov2640_imagesize_set(u16 width,u16 height);
void ov2640_stop();
void ov2640_start();





