
#include "CounterInput.h"

CounterInput::CounterInput()
{
    fd = open("/dev/CNT", O_RDWR);
    if(fd == -1) {
        printf("fopen error\n");
    }
}

CounterInput::~CounterInput()
{
    close(fd);
}

void CounterInput::UpdateIn()
{
    char* bufptr = buf;
    
    read(fd, buf, sizeof(buf));
    
    bufptr = buf;
    for(int i=0; i<4; i++) {
        counterData[i] = *(uint32_t*)bufptr;        
        bufptr += sizeof(uint32_t);
    }
}

int CounterInput::GetCounterValue(int Channel)
{
    if(Channel<0) return -1;
    if(Channel>=4) return -1;
    return(counterData[Channel]);
}

void CounterInput::CounterReset(int Channel)
{
    if(Channel<0) return;
    if(Channel>=4) return;
    obuf[0] = 0x01 << Channel;
    write(fd, obuf, sizeof(obuf));
}

void CounterInput::CounterResetAll()
{
    obuf[0] = 0x0F;
    write(fd, obuf, sizeof(obuf));
}


