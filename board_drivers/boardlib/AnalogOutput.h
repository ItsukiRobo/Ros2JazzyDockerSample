// AO1608LLPE Library

#ifndef ANALOG_OUTPUT_H
#define ANALOG_OUTPUT_H

class AnalogOutput
{
    int fd;
    int analogData[16];
    bool digitalData[4];
    
    char buf[32];
    char obuf[1];

public:
    AnalogOutput();
    ~AnalogOutput();
    
    void SetAnalogOut(int Channel, int Data);
    
    void SetDigitalOut(int Channel, bool Data);
    
    void UpdateOut();
};

#endif

