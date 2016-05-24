#include "link_list.h"
#include "global.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"



//初始化链表
int list_init(link_list *first){
    *first = (list_node *)malloc(sizeof(list_node));
    if(first == NULL){
        return -1;
    }
    (*first)->link = NULL;
		return 1;
}

void list_clear(link_list *first){
    list_node *q;
    while((*first)->link != NULL){
        q = (*first)->link;
        (*first)->link = q->link;
        free(q);
    }
}

int list_get_length(link_list *first){
    list_node *p = (*first)->link;
    int k = 0;
    while(p != NULL){
        k++;
        p = p->link;
    }
    return k;
}

int list_isempty(link_list *first){
    return ((*first)->link == NULL);
}

list_node* list_search(link_list *first,char *param_name){
    list_node *p = (*first)->link;
    while(p != NULL && strcmp(p->data->param_name,param_name)){
        p = p->link;
    }
    return p;

}

list_node *list_locate(link_list *first,int i){
    if(i < 0){
        return NULL;
    }
    list_node *p = *first;
    int k = 0;
    while(p != NULL && k < i){
        k++;
        p = p->link;
    }
    return p;
}


int list_insert(link_list *first,int i,char *param_name,float *param_value){
    list_node *p = list_locate(first,i-1);   //定位到第i-1个节点
    param *new_param;
    list_node *new_node;
		int k;
    if(p == NULL){
        return 0;  //指针是空的，也即是没有那么长的链表
    }
    new_node = (list_node *)malloc(sizeof(list_node));
    new_param = (param *)malloc(sizeof(param));
    strcpy(new_param->param_name,param_name);//复制参数名称
    for(k = 0;k < PARAM_GROUP_LENGTH;k++){
        new_param->param_value[k] = param_value[k];
    }
    new_node->data = new_param;
    new_node->link = p->link;
    p->link = new_node;
    return 1;
}

int list_remove(link_list *first,char *param_name){
    list_node *q = *first,*p = (*first)->link;
    while(p != NULL&&strcmp(p->data->param_name,param_name)){
        q = q->link;
        p = p->link;
    }
    if(p == NULL){
        return 0;
    }
    q->link = p->link;
    free(p->data);
    free(p);
    return 1;
}

void list_copy(link_list *dest,link_list *src){
    list_node *src_ptr = (*src)->link;
    list_node *dest_ptr = (*dest)->link;
    list_node *new_node;
    param *param_ptr;
		int i;
    while(src_ptr != NULL){
        param_ptr = (param *)malloc(sizeof( param));
        strcpy(param_ptr->param_name,src_ptr->data->param_name);
        for(i = 0;i < PARAM_GROUP_LENGTH;i++){
            param_ptr->param_value[i] = src_ptr->data->param_value[i];
        }
        new_node->data = param_ptr;
        new_node->link = NULL;
        dest_ptr->link = new_node;
        dest_ptr = dest_ptr->link;
        src_ptr = src_ptr->link;
    }
    dest_ptr->link = NULL;
}


void list_print(USART_TypeDef *USARTx,link_list  *first,int param_group){
    int i,j;
    if((*first)->link != NULL){
        uprintf(USARTx,"%s=%3.2f",(*first)->link->data->param_name,(*first)->link->data->param_value[param_group]);
        uprintf(USARTx,"\n");
//				for(i = 0;i < 10;i++)
//					for(j = 0;j < 100;j++); //延迟那么一小会儿
        list_print(USARTx,&((*first)->link),param_group);  //递归
    }
    
}

