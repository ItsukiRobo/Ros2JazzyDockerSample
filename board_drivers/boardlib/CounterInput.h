// CNT3204MTLPE Library

#ifndef COUNTER_INPUT_H
#define COUNTER_INPUT_H

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory.h>
#include <stdint.h>


class CounterInput
{
    char buf[128];
    char obuf[1];
    int fd;
    uint32_t counterData[4];

public:
    CounterInput();
    ~CounterInput();
    
    // カウンタ値を更新する。（制御ループで毎回呼び出す。）
    void UpdateIn();
    
    // カウンタ値を読む
    int GetCounterValue(int Channel);
    
    // カウンタを0にリセットする。
    void CounterReset(int Channel);
    void CounterResetAll();
};

#endif

