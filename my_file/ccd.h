/******************************代码简要说明**********************************/
/* 功能：通过ADC采集CCD数据
 * 使用的外设:ADC1、DMA、NVIC中断控制器
 * 使用的引脚:ADC1_CH5:PA5
 *            EXTI11:PG11
 *            EXTIx:PG12(CCD数据采集的开始中断)
 * 使用说明：
 */


#include "stm32f4xx.h"




//配置CCD行采集中断的优先级
#define CCD_NVIC_IRQPP 0              //中断抢断优先级
#define CCD_NVIC_IRQSP 0              //中断子优先级

#define CCD_DMA DMA2_Stream0           //CCD数据采集所用的DMA
#define CCD_DEFAUT_LINE_SIZE 128        //CCD行数据数

//线性ccd的数据结构，有行大小成员，有数据成员
typedef struct {
    u32 size;
    u16 *data;
}ccd_d;





int ccd_init(ccd_d *p_ccd_d);
void ccd_dma_config();
void ccd_adc_config();
void ccd_read_start();
void ccd_tim5_config();
void ccd_gpio_config();
void ccd_nvic_config();
void ccd_start();
void ccd_stop();
void ccd_rcc_config();


