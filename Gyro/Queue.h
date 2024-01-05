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
void q_clear();