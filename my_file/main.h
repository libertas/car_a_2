#include "stm32f4xx.h"

#define DEBUG_USARTx UART5
#define SENSOR_LEFT_LIGHT GPIO_Pin_7
#define SENSOR_RIGHT_LIGHT GPIO_Pin_3
#define SENSOR_COLOR GPIO_Pin_1
#define WHITE_LINE_WIDTH 15
#define WHITE_LINE_WIDTH_MAX 20   //最大线宽
#define WHITE_LINE_WIDTH_MIN 5    //最小线宽
#define STEP_LENGTH 4
#define WHITE_AREA_SIZE_MIN 500

extern float g_pre_centroid_x[10];
extern u8 g_death_turn_flag;//打死标志
void servo_lock(u8 lock);
