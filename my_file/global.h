#include "configuration.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"


#define CMD_BUFFER_LEN 100
#define camera_IMAGE_SIZE 20000
#define OV2650_FLASH_SETOR ((uint16_t)0x0028)   //ÉÈÇø5
#define OV2640_FLASH_ADDR_START 0x08020000
#define OV2640_FLASH_ADDR_END 0x080201FC
#define CODER_PERIOD 2000.f
#define NULL 0


extern int g_tim1_irq_flg;
extern int g_rotary;
extern float g_round;
extern u32 g_camera_irq_flag;
extern char g_camera_image[40000];
extern u8 g_ov2640_custom_config[][2]; 
extern u32 g_camera_line_cnt;
extern u16 g_camera_cols_cnt;
extern int g_light_sensor_left,g_light_sensor_right;
extern int g_color_right_flag;
extern u8 g_exp_lock;


void uprintf (USART_TypeDef* USARTx, char *fmt, ...);
void delay_us(uint32_t us);
void delay_ms(uint16_t ms);
void tim3_pwm_set(u16 freq,u16 duty);
void exti_color_disable();
void exti_color_enable();
void can_rcv_cb(CanRxMsg *can_rx_msg);
