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
	if(p==NULL)cerr<<"�洢����ʧ�ܣ�"<<endl;
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
	if (p==NULL){cerr<<"�洢����ʧ�ܣ�"<<endl;exit(1);}
	p->data=x;
	p->link=NULL;
	Q.rear->link=p;
	Q.rear=p;
	return true;
}

bool DeQueue(LinkQueue& Q,QElemType& x){
	if(Q.front==Q.rear){cerr<<"�����д��󣺶���Ϊ��";return false;}
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
		cerr<<"����Ϊ�ն���!"<<endl;
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
