#pragma once

#include <Arduino.h>

#include <ArduinoBLE.h>

#define SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define MAX_VAL 32 // max chars in characteristic value

class Bluetooth
{
public:
    using Callback = void (*)(const uint8_t *, uint16_t);

    Bluetooth() : service_(SERVICE_UUID),
                  characteristic_(CHARACTERISTIC_UUID, BLERead | BLEWrite, MAX_VAL) {};

    void handle_message(const uint8_t *msg, uint16_t msg_len)
    {
        if (msg_handler_cb_)
        {
            msg_handler_cb_(msg, msg_len);
        }
    }

    bool begin(Callback msg_handler_cb);
    void update();

private:
    Callback msg_handler_cb_;
    BLEService service_;
    BLECharacteristic characteristic_;
};
