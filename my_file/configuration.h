#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_fmc.h"
#include "stm32f4xx_flash.h"
#include "misc.h"

#define DCMI_DR_ADDR 0x50050028

extern char g_camera_image[];

void rcc_configuration(void);
void gpio_config(void);
void dma_config(void);
void tim1_configuration(void);
void TIM2_Configuration(void);
void TIM4_Configuration();
void USART_Configuration(void);
void NVIC_Configuration();
void SPI_Configuration(void);
void iic_configuration(void);
void tim3_config(void);
void fsmc_config(void);
void tim3_config();
void tim1_config();
void tim4_config();
void exti_config();
