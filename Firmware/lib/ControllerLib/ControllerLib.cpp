#include "ControllerLib.h"

ControllerLib::ControllerLib()
{
}

void ControllerLib::begin()
{
// Initialize serial to RS-485 comms
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200);

}

// void ControllerLib::recieveCommand(float &angle2, float &angle3, float &angle4, int &lightMode)
// {
//     static String input = "";
//     while (Serial2.available())
//     {
//         char c = Serial2.read();

//         if (c == '<')
//         {
//             input = ""; // Start new packet
//         }
//         else if (c == '>')
//         {
//             // End of packet: parse
//             int c1 = input.indexOf(',');
//             int c2 = input.indexOf(',', c1 + 1);
//             int c3 = input.indexOf(',', c2 + 1);

//             if (c1 == -1 || c2 == -1 || c3 == -1)
//                 return;

//             angle2 = input.substring(0, c1).toFloat();
//             angle3 = input.substring(c1 + 1, c2).toFloat();
//             angle4 = input.substring(c2 + 1, c3).toFloat();
//             lightMode = input.substring(c3 + 1).toInt();
//         }
//         else
//         {
//             input += c;
//         }
        
//         receiveState = true;

//     }

