#include "can.h"
#include "vega.h"
#include "string.h"
#include "stm32f4xx_can.h"


static int *g_vega_pos_x,*g_vega_pos_y;
static float *g_vega_angle;

void vega_msg_rcv_callback(CanRxMsg *can_rx_msg);
/* 函数名：int vega_init()
 * 功能：初始化vega
 * 参数：int *p_pos_x,存储x坐标的变量的指针
 *       int *p_pos_y,存储y坐标的变量的指针
 *       float *p_angle,存储角度的变量的指针
 * 返回值：1,初始化成功
 *         -1,初始化失败
 */
int vega_init(int *p_pos_x,int *p_pos_y,float *p_angle){

    g_vega_pos_x = p_pos_x;
    g_vega_pos_y = p_pos_y;
    g_vega_angle = p_angle;


    can_add_callback(VEGA_CAN_ID,vega_msg_rcv_callback);
    

    return 1;
}

/* 函数名：void vege_msg_rc_callback()
 * 功能：接收消息的回调函数
 * 参数：接收消息的数据结构的指针
 * 返回值：无
 */
void vega_msg_rcv_callback(CanRxMsg *can_rx_msg){
    data_convert temp;
    //if(can_rx_msg->StdId == VEGA_CAN_ID){   //如果是VEGA的包
        if(can_rx_msg->DLC == 8){
            temp.u8_form[0] = can_rx_msg->Data[0];
            temp.u8_form[1] = can_rx_msg->Data[1];
            temp.u8_form[2] = can_rx_msg->Data[2];
            temp.u8_form[3] = can_rx_msg->Data[3];
            memcpy((void*)g_vega_pos_x,&temp.s32_form,4);
            temp.u8_form[0] = can_rx_msg->Data[4];
            temp.u8_form[1] = can_rx_msg->Data[5];
            temp.u8_form[2] = can_rx_msg->Data[6];
            temp.u8_form[3] = can_rx_msg->Data[7];
            memcpy((void*)g_vega_pos_y,&temp.s32_form,4);
        }else if(can_rx_msg->DLC == 4){
            temp.u8_form[0] = can_rx_msg->Data[0];
            temp.u8_form[1] = can_rx_msg->Data[1];
            temp.u8_form[2] = can_rx_msg->Data[2];
            temp.u8_form[3] = can_rx_msg->Data[3];
            memcpy((void*)g_vega_angle,&temp.float_form,4);
        }
    //} 
}


/* 函数名：void vega_set_angle()
 * 功能：设置vega角度
 * 参数：float angle,需要设置的角度
 * 返回值：1,设置成功
 *         -1,设置失败
 */
int vega_set_angle(float angle){
    data_convert temp;
    temp.float_form = angle;
    return can_send_msg(CMD_CAN_ID,temp.u8_form,4);
}


/* 函数名：void vega_set_pos()
 * 功能：设置vega的x,y坐标
 * 参数：int pos_x 需要设置的x坐标
 *       int pos_y 需要设置的y坐标
 * 返回值：1,设置成功
 *         -1,设置失败
 */
int vega_set_pos(int pos_x,int pos_y){
    u8 send_data[8];

    send_data[0] = (u8)(pos_x >> 0 * 8);
    send_data[1] = (u8)(pos_x >> 1 * 8);
    send_data[2] = (u8)(pos_x >> 2 * 8);
    send_data[3] = (u8)(pos_x >> 3 * 8);
    send_data[4] = (u8)(pos_y >> 0 * 8);
    send_data[5] = (u8)(pos_y >> 1 * 8);
    send_data[6] = (u8)(pos_y >> 2 * 8);
    send_data[7] = (u8)(pos_y >> 3 * 8);
    return can_send_msg(CMD_CAN_ID,send_data,8);
}

/* 函数名：int vega_reset()
 * 功能：软件复位vega
 * 参数：无
 * 返回值：1,复位成功
 *         -1,复位失败
 */
int vega_reset(){
    u8 send_data[2];
    send_data[0] = 0x55;
    send_data[1] = 0xff;
    return can_send_msg(CMD_CAN_ID,send_data,2);
}
