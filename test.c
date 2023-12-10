#include <stdio.h>

#define Q_LENGTH 16
#define true 1
#define false 0
typedef int bool;
volatile unsigned char ptr_read = 0;
volatile unsigned char ptr_write = 0;
volatile unsigned char q[Q_LENGTH];
void q_push(unsigned char data);
unsigned char q_pop();
bool q_isFull();
bool q_isEmpty();

void q_push(unsigned char data){
    if(q_isFull()){
        return;
    }
    q[ptr_write] = data;
    ptr_write++;
    ptr_write &= (Q_LENGTH-1);
}

unsigned char q_pop(){
    if(q_isEmpty()){
        return -1;
    }
    unsigned char q_tmp;
    q_tmp = q[ptr_read];
    ptr_read++;
    ptr_read &= (Q_LENGTH-1);
    return q_tmp;
}

bool q_isFull(){
    if(ptr_write - ptr_read >= Q_LENGTH -1){
        return true;
    }
    else{
        return false;
    }
}   

bool q_isEmpty(){
    if(ptr_read == ptr_write){
        return true;
    }
    else{
        return false;
    }
    
}



int main(){
    q_push(1);
    q_push(2);
    q_push(3);
    q_push(4);
    q_push(5);
    q_push(6);
    q_push(7);
    q_push(8);
    q_push(9);
    q_push(10);
    q_push(11);
    q_push(12);
    q_push(13);
    q_push(14);
    q_push(15);
    q_push(16);//full

    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());

    q_push(1);
    q_push(2);
    q_push(3);
    q_push(4);
    q_push(5);
    q_push(6);
    q_push(7);
    q_push(8);
    q_push(9);
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
    printf("%d \n",q_pop());
}