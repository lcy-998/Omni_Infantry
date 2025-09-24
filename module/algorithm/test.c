#include "test.h"
#include "stdlib.h"

Queue *initQueue(void)
{
	Queue *q = (Queue *)malloc(sizeof(Queue));
	q->data = (ElemType *)malloc(sizeof(ElemType) * MAXSIZE);
	q->front = 0;
	q->rear = 0;
    q->sum = 0.0;
	return q;
}

int isEmpty(Queue *Q)
{
	if(Q->rear == Q->front)
	{
		
		return 1;
	}
	else
	{
		return 0;
	}
}

int isFull(Queue *Q)
{
	if((Q->rear + 1) % MAXSIZE == Q->front)
	{
		
		return 1;
	}
	else
	{
		return 0;
	}
}

int equeue(Queue *Q, ElemType e)
{
	if(isFull(Q))
	{
		return 0;
	}
	Q->data[Q->rear] = e;
	Q->rear = (Q->rear + 1) % MAXSIZE;
    Q->sum += e;
	return 1;
}

int dequeue(Queue *Q, ElemType *e)
{
	if(isEmpty(Q))
	{
		return 0;
	}
	*e = Q->data[Q->front];
	Q->front = (Q->front + 1) % MAXSIZE;
    Q->sum -= *e;
	return 1;
}

int getHead(Queue *Q, ElemType *e)
{
	if(isEmpty(Q))
	{
		return 0;
	}
	*e = Q->data[Q->front];
	return 1;
}
