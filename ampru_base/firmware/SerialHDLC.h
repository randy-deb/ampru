
#ifndef ___ARDUINO_SERIAL_HDLC_H___
#define ___ARDUINO_SERIAL_HDLC_H___

#include <Arduino.h>
#include <inttypes.h>

typedef void (*serial_hdlc_frame_handler)(const uint8_t* frameBuffer, uint16_t frameLength);

class SerialHDLC
{
public:
    SerialHDLC(serial_hdlc_frame_handler frameHandler, uint16_t maxFrameLength);
    ~SerialHDLC();

public:
    void read();
    void write(const uint8_t* frameBuffer, uint16_t frameLength);

private:
    void readByte(uint8_t data);
    void writeByte(uint8_t data);

private:
    serial_hdlc_frame_handler _frameHandler;
    uint8_t* _frameBuffer;
    uint16_t _frameOffset;
    uint16_t _frameChecksum;
    uint16_t _maxFrameLength;
    bool _escapeByte;

};

#endif // ___ARDUINO_SERIAL_HDLC_H___
