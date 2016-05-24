
/******************************代码简要说明**********************************/
/* 功能：通过CAN总线来读取vega的旋转角度和正交编码器的数据
 * 使用的外设:CAN1、CAN2、NVIC中断控制器
 * 使用的引脚:CAN1: CAN_RX PA11 , CAN_TX PA12
 *            或
 *            CAN2: CAN_RX PB12 , CAN_TX PB13
 * 使用说明:   自己定义变量int pos_x,int pos_y,float angle。其分别表示X坐标、Y
 *          坐标，和旋转角度。然后以这些变量的地址作为参数执行函数
 *          vega_init(&pos_x,&pos_y,&angle)。函数执行成功之后，变量pos_x,pos_y
 *          ,angle的值就会不断地更新。本代码是通过CAN总线来读取VEGA数据的，关于
 *          CAN总线的配置，请看can.h文件。
 */
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"


#define VEGA_CAN_ID 0x11   //vega数据包的标识符


typedef union{
    u8 u8_form[4];
    int s32_form;
    float float_form;
}data_convert;

void vega_msg_rcv_callback(CanRxMsg *can_rx_msg);
int vega_init(int *p_pos_x,int *p_pos_y,float *angle);
int vega_reset();
int vega_set_angle(float angle);
