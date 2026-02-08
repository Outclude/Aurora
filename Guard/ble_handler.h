#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <string>

class BLEHandler {
public:
    // Process the received message and return the response string.
    // Returns an empty string if no response is needed.
    static std::string handleMessage(const std::string& message);
};

#endif
