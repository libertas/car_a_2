/***********************************代码简要说明***********************************/
/* 功能：通过串口接受命令而且执行命令对应的函数
 * 使用的外设:USART1、USART2、USART3（三选一）,NVIC中断控制器
 *            GPIO：USART1，TX PA9 RX PA10
 *                  USART2，TX PA2 RX PA3
 *                  USART3，TX PB10 RX PB11
 *                  UART5,  TX PC12 RX PD2
 *                  （若需改引脚，请看cmd.c文件中的cmd_init()函数）
 * 使用说明：在使用之前，必须要配置相关的宏（下面有注释需要定义的宏）。代码所需
 * 要的串口是可以选择的，定义宏CMD_USARTn为所需要的串口号即可。代码可以自己配置外
 * 设(#define CMD_PERIPH_INIT_EN 0)也可以自动配置外设(EN 1)（执行cmd_init()即可
 * 配置好外设）。
 *     添加命令很简单，只需三个步骤：
 *     步骤一：
 *         在cmd_func.c文件中的cmd_tbl数组中加上
 *             CMD_ADD("命令名","命令信息",对应命令的执行函数名)
 *     步骤二：
 *         在文件cmd_func.h中声明对应命令的执行函数，函数的返回值为void，形参有
 *         两个：一个是int argc，命令参数个数（包括命令名称）；一个是
 *         char *argv[]，参数数组。如  void test_func(int argc,char *argv[]);
 *     步骤三：
 *         在定义对应命令的执行函数的文件中，包含cmd_func.h头文件，或者(建议)在文件
 *         cmd_fun.c里定义对应命令的执行函数。
 * 注意事项：命令执行函数是在中断里面执行的，建议不能执行太久，以防因长时间抢断而
 *           发生一些未知错误。为了协调好工程代码执行，请配置好命令执行函数的中断
 *           优先级。
 */


#include "stm32f4xx.h"
/****************************以下是使用代码之前，必须配置的宏*******************************/
#define CMD_USARTn 5    //接收命令的串口号
//自动初始化外设使能,如果使能了，则调用cmd_init()函数之后，即可初始化好串口和中断
#define CMD_PERIPH_INIT_EN 0    

//如果没有使能自动初始化，则无需配置下面的宏了
#if CMD_PERIPH_INIT_EN == 1
    #define CMD_NVIC_GROUP NVIC_PriorityGroup_1               //中断组别 
    #define CMD_NVIC_IRQPP 0              //中断抢断优先级
    #define CMD_NVIC_IRQSP 0              //中断子优先级
    #define CMD_USART_BAUD 115200         //串口波特率
#endif

/**************以上是使用代码之前，必须配置的宏****************/


#if CMD_USARTn == 1
    #define CMD_USARTx USART1 
#elif CMD_USARTn == 2
    #define CMD_USARTx USART2 
#elif CMD_USARTn == 3
    #define CMD_USARTx USART3
#elif CMD_USARTn == 5
    #define CMD_USARTx UART5
#endif


#define MAX_CMD_ARG_LENGTH 16
#define MAX_CMD_INFO_LENGTH 64
#define MAX_CMD_LINE_LENGTH 128
#define MAX_CMD_LINE_CNT 5
#define MAX_ARGC 8
#define STR_BUFFER_LEN 128


typedef struct {
    char cmd_name[MAX_CMD_ARG_LENGTH];   //命令的名字
    char cmd_usage[MAX_CMD_INFO_LENGTH];   //命令的信息
    void (*cmd_func)(int acgc,char *argv[]); //命令执行函数
}cmd_struct;


#define CMD_ADD(cmd_name,cmd_usage,cmd_func) \
    {\
        cmd_name,\
        cmd_usage,\
        cmd_func\
    }

void cmd_init();
int cmd_parse(char *cmd_line,int *argc,char *argv[]);  //命令行解析
int cmd_exec(int argc,char *argv[]);

