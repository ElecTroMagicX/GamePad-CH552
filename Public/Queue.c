
#include "Queue.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned char queue[QUEUE_LEN];
static unsigned char head = 0;
static unsigned char end = 0;

void initQueue()
{
    *queue = malloc(QUEUE_LEN);
}

unsigned char getNextIndex(unsigned char p)
{
    if (p < (QUEUE_LEN - 1))
    {
        return (p + 1);
    }
    return 0;
}

unsigned char getLastIndex(unsigned char p)
{
    if (p > 0)
    {
        return (p - 1);
    }
    return (QUEUE_LEN - 1);
}

void enQueue(unsigned char input)
{
    if (getLastIndex(head) == end)
    {
        head = getNextIndex(head);
    }
    queue[end] = input;
    end = getNextIndex(end);
}

unsigned char deQueue(unsigned char *res)
{
    if (head == end)
    {
        return 0;
    }
    *res = queue[head];
    printf("deQueue: %x\n", queue[head]);
    head = getNextIndex(head);
    return 1;
}

void testQueue()
{
    unsigned char a1;
    unsigned char i;
    unsigned char j;

    initQueue();
    printf("deQueue empty: %d\n", deQueue(&a1));
    for (i = 1; i <= QUEUE_LEN; i++)
    {
        enQueue(i);
    }
    for (j = 1; i <= QUEUE_LEN; i++)
    {
        if (deQueue(&a1))
        {
            printf("deQueue: %d\n", a1);
        }
        else
        {
            printf("deQueue fail\n");
        }
    }
    enQueue(QUEUE_LEN + 1);
    deQueue(&a1);
    printf("deQueue: %d\n", a1);
}
