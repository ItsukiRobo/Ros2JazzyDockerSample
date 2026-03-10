// AI1616LLPE Library

#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

class AnalogInput
{
    int fd;
    int analogData[16];
    bool digitalData[4];
    
    char buf[32];
    unsigned char obuf[1];

public:
    AnalogInput();
    ~AnalogInput();
    
    void UpdateIn();
    int GetAnalogIn(int Channel);
    
    void UpdateOut();
    void SetDigitalOut(int Channel, bool Data);
};

#endif

