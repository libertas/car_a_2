#include "stm32f4xx.h"

#define PARAM_NAME_LENGTH 32
#define PARAM_GROUP_LENGTH 3
//链表节点的数据
typedef struct{
    char param_name[PARAM_NAME_LENGTH];
    float param_value[PARAM_GROUP_LENGTH];   //有三组参数
}param;

typedef struct node{
    param *data;
    struct node *link;
}list_node,*link_list;


int list_init(link_list *first);
void list_clear(link_list *first);
int list_get_length(link_list *first);
int list_isempty(link_list *first);
list_node* list_search(link_list *first,char *param_name);
list_node *list_locate(link_list *first,int i);
int list_insert(link_list *first,int i,char *param_name,float *param_value);
int list_remove(link_list *first,char *param_name);
void list_copy(link_list *dest,link_list *src);
void list_print(USART_TypeDef *USARTx,link_list *first,int param_group);

