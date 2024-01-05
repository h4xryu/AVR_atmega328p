#include "Queue.h"

/*Push data to queue.*/
/*Data size : 8bit*/
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

void q_clear(){
	while(!q_isEmpty()){
		q_pop();
	}
}