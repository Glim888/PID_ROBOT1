#ifndef __A4988__
#define __A4988__

#include <Arduino.h>

enum Directions {
  STOP = 100,
  LEFT = 101,
  RIGHT = 102
};

class A4988 {

  private:
    int a4988_speed = 0;
    int a4988_dir = STOP;
    int a4988_i = 0;

    int pinDir = 0;
    int pinStepps = 0;

  public:

    //minimale Zyklusezeit und maximale Zykluszeit
    A4988 (int pinDir, int pinStepps);
    
    // initialisieren der Motoren
    void init();
    
    //spd von (0-255)
    //dir -> Directions Enum
    void setMove(int spd, int dir);
};


#endif
