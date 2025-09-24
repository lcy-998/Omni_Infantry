#ifndef _TEST_H__
#define _TEST_H__

#define MAXSIZE 50
typedef float ElemType;

typedef struct
{
	ElemType *data;
	int front;
	int rear;
    ElemType sum;
}Queue;

Queue *initQueue(void);
int isEmpty(Queue *Q);
int isFull(Queue *Q);
int equeue(Queue *Q, ElemType e);
int dequeue(Queue *Q, ElemType *e);
int getHead(Queue *Q, ElemType *e);

#endif
