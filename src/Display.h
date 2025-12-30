#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "Status.h"
#include "Config.h"

class Display
{
public:
    Display(uint8_t screen_h, uint8_t screen_w, int8_t rotation, status_t *status)
        : disp_(screen_h, screen_w)
    {
        screen_h_ = screen_h;
        screen_w_ = screen_w;
        rotation_ = rotation;
        status_ = status;
    }

    void begin();

    volatile void update();

private:
    status_t *status_;
    TFT_eSPI disp_;
    uint8_t screen_w_;
    uint8_t screen_h_;
    int8_t rotation_;

    static float measure_input_volts_();
};
