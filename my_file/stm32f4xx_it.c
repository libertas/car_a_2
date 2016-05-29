/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include "global.h"
#include "tftlcd.h"
#include "stdlib.h"
#include "camera.h"
#include "parameter.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
	char str_temp[100];
	int g_light_sensor_left = 0,g_light_sensor_right = 0;
	int g_color_right_flag = 0;
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}




/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}


/* void EXTI3_IRQHandler(void){ */
		/* if(EXTI_GetITStatus(EXTI_Line3) != RESET){ */
				/* int i; */
				/* if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) == 0){ */
						/* DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN; //??DMA */
						/* while(DMA2_Stream1->CR&0X01);//??DMA2_Stream1??? */
						/* DMA2->LIFCR|=0X3D<<6;	 //????1??????? */
						/* DMA2_Stream1->NDTR = 20000; */
						/* DMA2_Stream1->FCR=0X0000021;//?????? */
						/* DMA2_Stream1->M0AR=(uint32_t)(g_camera_image);//(g_camera_image)(0x60000000|0x0C000000|0x00000080);   //????????? */
						/* g_camera_line_cnt = 0; */
						/* DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;  //??DMA */
						/* g_camera_irq_flag = 1; */
						/* g_camera_cols_cnt++; */
				/* } */
				/* EXTI_ClearITPendingBit(EXTI_Line3); */
		/* } */
/* } */

void EXTI1_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line1) != RESET){
        int i;
        if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1) == 1){
            g_death_turn_flag = 1;
            //tim3_pwm_set(100,1100);  //往右打死 
            //for(i = 0;i < 10;i++){
            //    g_pre_centroid_x[i] = 180;   //固定弯拐完之后，白线还是不见的话，继续右拐
            //}
            //sprintf(str_temp,"g:1"); 
            //lcd_show_string(100,20,200,100,16,str_temp);
            //delay_ms((g_param->servo_d_base) * 10);
            exti_color_disable();  //关中断
        }
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI9_5_IRQHandler(void){
		static int color_sensor_left_cnt = 0,color_sensor_right_cnt = 0;
		static int light_sensor_left_cnt = 0,light_sensor_right_cnt = 0;
//		if(EXTI_GetITStatus(EXTI_Line6) != RESET){
//			//	delay_ms(500);
//				if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_6) == 1){
////						sprintf(str_temp,"1:%d",color_sensor_left_cnt++);
////						lcd_show_string(100,20,200,100,16,str_temp);

//				}else if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_6) == 0){
//			//			delay_ms(1500);
////						sprintf(str_temp,"0:%d",++color_sensor_right_cnt);
////						lcd_show_string(150,20,200,100,16,str_temp);
//						if(color_sensor_right_cnt == 3){
//								g_color_right_flag = 1;
//						}
//				}
//				EXTI_ClearITPendingBit(EXTI_Line6);
//		}
		if(EXTI_GetITStatus(EXTI_Line9) != RESET){
				int i;
				if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_9) == 1){
						tim3_pwm_set(100,550);  //往右打死 
						for(i = 0;i < 10;i++){
								g_pre_centroid_x[i] = 180;   //固定弯拐完之后，白线还是不见的话，继续右拐
						}
						sprintf(str_temp,"g:1"); 
						lcd_show_string(100,20,200,100,16,str_temp);
						delay_ms((g_param->servo_d_base) * 10);
						exti_color_disable();  //关中断
				}
				EXTI_ClearITPendingBit(EXTI_Line9);
		}
		
//		if(EXTI_GetITStatus(EXTI_Line8) != RESET){
//				delay_ms(300);
//				if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_8) == 1){
//						g_light_sensor_right = 1;
//				}else if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_8) == 0){
//						g_light_sensor_right = 0;
//				}
//				EXTI_ClearITPendingBit(EXTI_Line8);
//		}
}  

void EXTI15_10_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line10) != RESET){
        delay_ms(30);
        if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10) == 0){
            g_key1_flag = 1;
        }
        EXTI_ClearITPendingBit(EXTI_Line10);
    }
}

void TIM2_IRQHandler(void){
	  if( TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) 
  	{
			TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);//必须清除中断标志位否则一直中断
		}	
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//  TimingDelay_Decrement();
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
