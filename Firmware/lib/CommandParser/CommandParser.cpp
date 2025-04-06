#include "CommandParser.h"

void CommandParser::begin(HardwareSerial &serial)
{
    serial.begin(115200);
}

#include "CommandParser.h"

CommandParser::CommandParser()
{
}

void CommandParser::begin(HardwareSerial &serialPort)
{
    serial = &serialPort;
    buffer = "";
    receiving = false;
}

bool CommandParser::readCommand(CommandMessage &msg)
{
    while (serial->available())
    {
        char c = serial->read();

        if (c == '<')
        {
            buffer = "";
            receiving = true;
        }
        else if (c == '>' && receiving)
        {
            receiving = false;

            // Parse buffer into CommandMessage
            int c1 = buffer.indexOf(',');
            int c2 = buffer.indexOf(',', c1 + 1);
            int c3 = buffer.indexOf(',', c2 + 1);
            int c4 = buffer.indexOf(',', c3 + 1);

            if (c1 == -1 || c2 == -1 || c3 == -1 || c4 == -1)
                return false;

            msg.a1 = buffer.substring(0, c1).toFloat();
            msg.a2 = buffer.substring(c1 + 1, c2).toFloat();
            msg.a3 = buffer.substring(c2 + 1, c3).toFloat();
            msg.a4 = buffer.substring(c3 + 1, c4).toFloat();
            msg.lightMode = buffer.substring(c4 + 1).toInt();

            return true;
        }
        else if (receiving)
        {
            buffer += c;
        }
    }

    return false;
}

void CommandParser::sendStatus(const CommandMessage &msg)
{
    if (!serial)
        return;

    serial->print("<");
    serial->print(msg.a1, 2);
    serial->print(",");
    serial->print(msg.a2, 2);
    serial->print(",");
    serial->print(msg.a3, 2);
    serial->print(",");
    serial->print(msg.a4, 2);
    serial->println(">");
}
