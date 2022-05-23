
#ifndef __QUEUE_H__
#define __QUEUE_H__

#ifndef QUEUE_LEN
#define QUEUE_LEN 6
#endif

void initQueue();

void enQueue(unsigned char input);

unsigned char deQueue(unsigned char *res);

#endif
