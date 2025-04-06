#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H
#include <Arduino.h>

struct CommandMessage
{
    float a1, a2, a3, a4;
    int lightMode;
};

class CommandParser
{
  public:
    void begin(HardwareSerial &serial);
    bool readCommand(CommandMessage &msg);
    void sendStatus(const CommandMessage &current);

  private:
    HardwareSerial *serial;
    String buffer;
    bool receiving;
};

#endif