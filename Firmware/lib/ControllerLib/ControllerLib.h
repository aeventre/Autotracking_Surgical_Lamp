#ifndef CONTROLLERLIB_H
#define CONTROLLERLIB_H

#include <Arduino.h>


#define dirPin 16
#define stepPin 17

class ControllerLib
{
  public:
    ControllerLib();

    void begin();

    void recieveCommand(float &angle2, float &angle3, float &angle4, int &lightMode);


  private:

  bool receiveState = false;
};

#endif
