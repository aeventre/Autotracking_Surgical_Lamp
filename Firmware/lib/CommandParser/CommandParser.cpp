#include "CommandParser.h"

#include "CommandParser.h"

void CommandParser::begin(HardwareSerial &serialPort)
{
    serial = &serialPort;
    buffer = "";
    receiving = false;

    pinMode(RS485_DIR, OUTPUT);
    digitalWrite(RS485_DIR, LOW);  // Start in RX mode

    // RP2040: explicitly set TX/RX pins
    #ifdef ARDUINO_ARCH_RP2040
      Serial2.setTX(4);
      Serial2.setRX(5);
    #endif

    serial->begin(115200);
    delay(200);

    // Send boot message
    digitalWrite(RS485_DIR, HIGH);  // TX mode
    serial->println("<BOOT,0.0,0.0,0.0,0>");
    serial->flush();
    delayMicroseconds(100);
    digitalWrite(RS485_DIR, LOW);   // Back to RX
}


bool CommandParser::readCommand(CommandMessage &msg)
{
    while (serial->available())
    {
        char c = serial->read();

        if (c == '<') {
            buffer = "";
            receiving = true;
        }
        else if (c == '>' && receiving) {
            receiving = false;

            // Split into tokens
            int values[5];
            int index = 0;
            int start = 0;

            for (int i = 0; i < buffer.length() && index < 5; i++) {
                if (buffer[i] == ',' || i == buffer.length() - 1) {
                    int end = (buffer[i] == ',') ? i : i + 1;
                    String token = buffer.substring(start, end);
                    values[index++] = token.toFloat();
                    start = i + 1;
                }
            }

            if (index != 5) return false;  // Not enough values

            msg.a1 = values[0];
            msg.a2 = values[1];
            msg.a3 = values[2];
            msg.a4 = values[3];
            msg.lightMode = values[4];

            _receivedFlag = true;
            return true;
        }
        else if (receiving) {
            buffer += c;
        }
    }

    return false;
}


void CommandParser::sendStatus(const CommandMessage &msg)
{
    if (!serial) return;

    digitalWrite(RS485_DIR, HIGH);  // ✅ TX mode
    delayMicroseconds(10);

    serial->print("<");
    serial->print(msg.a1, 2);
    serial->print(",");
    serial->print(msg.a2, 2);
    serial->print(",");
    serial->print(msg.a3, 2);
    serial->print(",");
    serial->print(msg.a4, 2);
    serial->print(",");
    serial->print(msg.lightMode);
    serial->println(">");

    serial->flush();
    delayMicroseconds(10);

    digitalWrite(RS485_DIR, LOW);
}


bool CommandParser::messageReceived() const {
    return _receivedFlag;
}

void CommandParser::clearReceivedFlag() {
    _receivedFlag = false;
}
