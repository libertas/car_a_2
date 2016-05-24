#include "global.h"

//ov7670 port set
#define OV7670_I2Cx I2C1

#define OV7670_SCL_PORT GPIOB
#define OV7670_SCL_PIN GPIO_Pin_8
#define OV7670_SDA_PORT GPIOB
#define OV7670_SDA_PIN GPIO_Pin_9
#define OV7670_PCLK_PORT
#define OV7670_PCLK_PIN
#define OV7670_VSYNC_PORT
#define OV7670_VSYNC_PIN
#define OV7670_HREF_PORT
#define OV7670_HREF_PIN
#define OV7670_XCLK_PORT
#define OV7670_XCLK_PIN
#define OV7670_RESET_PORT GPIOG
#define OV7670_RESET_PIN GPIO_Pin_15
#define OV7670_PWDN_PORT GPIOG
#define OV7670_PWDN_PIN GPIO_Pin_9
#define OV7670_D0_PORT
#define OV7670_D0_PIN
#define OV7670_D1_PORT
#define OV7670_D1_PIN
#define OV7670_D2_PORT
#define OV7670_D2_PIN
#define OV7670_D3_PORT
#define OV7670_D3_PIN
#define OV7670_D4_PORT
#define OV7670_D4_PIN
#define OV7670_D5_PORT
#define OV7670_D5_PIN
#define OV7670_D6_PORT
#define OV7670_D6_PIN
#define OV7670_D7_PORT
#define OV7670_D7_PIN


#define OV7670_I2C_ADDR 0X60
//ov7670 reg addr
#define OV7670_PID 0x0A
#define OV7670_VER 0x0B



/*内部时钟  CLKRC
 * bit[7]:保留
 * bit[6]:直接使用外部时钟(没有预分频)
 * bit[5:0]:内部时钟分频
 *          F(内部时钟) = F(输入时钟)/(位[5:0]+1)
 *          范围:[0000]~[1111]
 */
#define OV7670_CLKRC 0x11
/*行缓冲测试选项  TSLB 默认值0xOD
 *位[7：6]：保留
 *位[5]：负片使能
 *  0：正常
 *  1：负片
 *位[4]：UV输出数据
 *  0：使用通用的UV 输出
 *  1：使用固定的UV输出，通过设定MANU和MANV做为输出代
 *     替片内输出
 *位[3]：输出顺序(由寄存器COM13[0](0x3D)一起决定)
 *TSLB[3] COM13[0]
 *  0 0:YUYV
 *  0 1:YVYU
 *  1 0:UYVY
 *  1 1:VYUY
 *位[2：1]：保留
 *位[0]：自动输出窗口
 *  0：当分辨率改变时，传感器不会自动设置窗口，后端处
 *     理器能立即调整窗口
 *  1：当分辨率改变时，传感器立即自动设置窗口，后端处
 *     理器必须在下一个Vsync后调整窗口
 */
#define OV7670_TSLB 0x3A
#define OV7670_SCALING_DCWCTR 0x72
#define OV7670_DBLV 0x6B
#define OV7670_SCALING_PC 0X73


#define OV7670_COM1 0x04 
#define OV7670_COM2 0x09
#define OV7670_COM3 0x0C
#define OV7670_COM4 0x0D
#define OV7670_COM5 0x0E
#define OV7670_COM6 0x0F
/*通用控制7  COM7
 * bit[7]:SCCB寄存器复位
 *          0:不复位
 *          1:复位
 * bit[6]:保留
 * bit[5]:输出格式-CIF
 * bit[4]:输出格式-QVGA
 * bit[3]:输出格式-QCIF
 * bit[2]:输出格式-RGB(见下面)
 * bit[1]:彩色条
 *          0:非使能
 *          1:使能
 * bit[0]:输出格式-Raw RGB(见以下)
 *                      COM7[2]         COM7[0]
 * YUY                  0               0
 * RGB                  0               1
 * Bayer RAW            1               0
 * Processed Bayer RAW  1               1
 */
#define OV7670_COM7 0x12
#define OV7670_COM8 0x13
#define OV7670_COM9 0x14
#define OV7670_COM10 0x15
#define OV7670_COM11 0x3B
#define OV7670_COM12 0x3C
/*通用控制13  COM13 bit[0]:UV交换位置(和寄存器TSLB[3](0x3A))一起作用
 * TSLB[3] COM13[0]
 * 0 0:YUYV
 * 0 1:YVYU
 * 1 0:UYVY
 * 1 1:VYUY
 */
#define OV7670_COM13 0x3D
#define OV7670_COM14 0x3E
#define OV7670_COM15 0x40
#define OV7670_COM16 0x41
#define OV7670_COM17 0x42

#define OV7670_MANU 0x67
#define OV7670_MANV 0x68

//分辨率设置为82*61
#define OV7670_ROWS_SIZE 82
#define OV7670_COLS_SIZE 61


//extern char g_ov7670_image[10000];
//extern u32 g_ov7670_irq_flag;
//extern u32 g_ov7670_line_cnt;

#define OV2640_CUSTOM_CONFIG_SIZE 8

int ov7670_reg_write(u8 reg_addr,u8 data_write);
int ov7670_reg_read(u8 reg_addr,u8 *data_read);
int ov7670_init(void);
int ov7670_get_image();
/* 该函数在OV7670_PCLK上升沿中断处理函数中调用
 * 在该函数里，主要负责采集图像，如果初始化函数中设置了采集二值化图像，则会采集的
 * 同时，也会进行二值化的处理，而且还算出了图像的质心(用于循迹)；
 */
void ov7670_irq_handler();  
//该函数检测ov7670是否正常
int ov7670_check();
int ov7670_scl_set(u32 status);
int ov2640_save_data_toram(u8 dst_array[][2],u8 reg_addr,u8 reg_data,u8 reg_type);
void ov7670_reset();
u8 ov2640_outsize_set(u16 width,u16 height);
void ov2640_get_data(u8 read_array[][2]);
void ov2640_window_set(u16 sx,u16 sy,u16 width,u16 height);
void ov2640_config(u8 reg_tb[][2]);
u8 ov2640_imagewin_set(u16 offx,u16 offy,u16 width,u16 height);
u8 ov2640_imagesize_set(u16 width,u16 height);
void ov2640_stop();
void ov2640_start();
//void DCMI_IRQHandler(void);





