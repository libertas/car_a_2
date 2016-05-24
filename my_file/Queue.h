#include <stdlib.h>
#include <iostream>
using namespace std;
typedef char* QElemType;
typedef struct queuenode{
	QElemType data;
    struct queuenode *link;
}QueueNode;
typedef struct {
    QueueNode *front,*rear;
}LinkQueue;

void InitQueue(LinkQueue& Q){
    QueueNode *p=new QueueNode;
	if(p==NULL)cerr<<"存储分配失败！"<<endl;
	Q.front=Q.rear=p;
}

void ClearQueue(LinkQueue& Q)
{
    QueueNode *p=Q.front->link;
	while (Q.front->link!=NULL)
	{
		Q.front->link=p->link;
		delete p;
		p=Q.front->link;
	}
	Q.rear=Q.front;
}

bool EnQueue(LinkQueue& Q,QElemType x){
    QueueNode *p=new QueueNode;
	if (p==NULL){cerr<<"存储分配失败！"<<endl;exit(1);}
	p->data=x;
	p->link=NULL;
	Q.rear->link=p;
	Q.rear=p;
	return true;
}

bool DeQueue(LinkQueue& Q,QElemType& x){
	if(Q.front==Q.rear){cerr<<"出队列错误：队列为空";return false;}
    QueueNode *p=Q.front->link;
	Q.front->link=p->link;
	x=p->data;
    if(Q.front->link==NULL)
    {
        Q.rear=Q.front;
    }
        delete p;
	return true;
	
}

bool GetFront(LinkQueue& Q,QElemType x){
	if (Q.front->link==NULL)
	{
		cerr<<"错误：为空队列!"<<endl;
		return false;
	}
	x=Q.front->link->data;
	return true;
}

bool QueueEmpty(LinkQueue Q)
{
	return Q.front->link==NULL;
}

int QueueSize(LinkQueue Q){
    QueueNode *p;
	int k;
	p=Q.front;
	while (p!=NULL)
	{
		k++;
		p=p->link;
	}
	return k;
}
