#include "Arduino.h"
#include "Bluetooth.h"

bool Bluetooth::begin(Callback msg_handler_cb)
{
    // COnfigure bluetooth:
    if (!BLE.begin())
        return false;
    BLE.setLocalName("ESP32_BLE");
    BLE.setAdvertisedService(service_);
    service_.addCharacteristic(characteristic_);
    BLE.addService(service_);
    characteristic_.writeValue(0);
    BLE.advertise();

    // Set callback function to be called when message is received:
    msg_handler_cb_ = msg_handler_cb;

    return true;
}

void Bluetooth::update()
{
    BLEDevice _central = BLE.central();
    if (_central)
    {
        if (_central.connected())
            Serial.println("Device connected");
        while (_central.connected())
        {
            if (characteristic_.written())
                handle_message(characteristic_.value(), characteristic_.valueLength());
        }
        Serial.println("Device disconnected");
    }
}
