#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory.h>

#include "AnalogOutput.h"

AnalogOutput::AnalogOutput()
{
    fd = open("/dev/AO", O_RDWR);
    if(fd == -1) {
        printf("fopen error\n");
    }
    
    for(int i=0; i<4; i++) digitalData[i] = 0;
}

AnalogOutput::~AnalogOutput()
{
    close(fd);
}

void AnalogOutput::SetAnalogOut(int Channel, int Data)
{
    if(Channel<0) return;
    if(Channel>=8) return;
    analogData[Channel] = Data;
}

void AnalogOutput::SetDigitalOut(int Channel, bool Data)
{
    if(Channel<0) return;
    if(Channel>=4) return;
    digitalData[Channel] = Data;
}

void AnalogOutput::UpdateOut()
{
    char* bufptr = buf;
    unsigned short int dadata[9];
    
    unsigned char k = (digitalData[3]<<3) | (digitalData[2]<<2) | (digitalData[1]<<1) | (digitalData[0]);
    
    bufptr = buf;
    for(int i=0; i<8; i++) dadata[i] = (unsigned short int)analogData[i];
    dadata[8] = 0x0000000F & k;    
        
    for(int i=0; i<9; i++) {   
        memcpy(bufptr, &dadata[i], sizeof(unsigned short int));
        
        bufptr += sizeof(unsigned short int);
        
    }
    
    write(fd, buf, sizeof(buf));
}

