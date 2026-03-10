#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "AnalogInput.h"

AnalogInput::AnalogInput()
{
    fd = open("/dev/AI", O_RDWR);
    if(fd == -1) {
        printf("fopen error\n");
    }
    
    for(int i=0; i<4; i++) digitalData[i] = 0;
}

AnalogInput::~AnalogInput()
{
    close(fd);
}

void AnalogInput::UpdateIn()
{
    char* bufptr = buf;
    
    read(fd, buf, sizeof(buf));
    
    bufptr = buf;
    for(int i=0; i<16; i++) {
        analogData[i] = *(unsigned short int*)bufptr;
        bufptr += sizeof(unsigned short int);
    }
}

int AnalogInput::GetAnalogIn(int Channel)
{
    if(Channel<0) return -1;
    if(Channel>=16) return -1;
    return(analogData[Channel]);
}

void AnalogInput::UpdateOut()
{
    obuf[0] = (digitalData[3]<<3) | (digitalData[2]<<2) | (digitalData[1]<<1) | (digitalData[0]);
    
    write(fd, obuf, sizeof(obuf));
}

void AnalogInput::SetDigitalOut(int Channel, bool Data)
{
    if(Channel<0) return;
    if(Channel>=4) return;
    digitalData[Channel] = Data;
}

