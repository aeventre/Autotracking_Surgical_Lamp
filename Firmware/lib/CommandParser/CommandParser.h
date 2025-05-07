#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H
#include <Arduino.h>

#define RS485_DIR 6  // Pico GPIO6 controls RE/DE

struct CommandMessage
{
    float a1, a2, a3, a4;
    int lightMode;
};
class CommandParser {
public:
    void begin(HardwareSerial &serialPort);
    bool readCommand(CommandMessage &msg);
    void sendStatus(const CommandMessage &msg);
    bool messageReceived() const;
    void clearReceivedFlag();

private:
    HardwareSerial* serial = nullptr;
    String buffer;
    bool receiving = false;
    bool _receivedFlag = false;
};


#endif