#include "A4988.h"

A4988::A4988(int pinDir, int pinStepps) {

  this->pinDir = pinDir;
  this->pinStepps = pinStepps;

  pinMode(pinDir, OUTPUT);
  pinMode(pinStepps, OUTPUT);
}

volatile int k = 0;
volatile int a4988_waitTime = 0;
volatile bool a4988_stop = false;

void A4988::init() {
  // Timer
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;

  OCR4A = 50;
  TCCR4B |= (1 << WGM12);
  TCCR4B |= (1 << CS10);
  TCCR4B |= (1 << CS11);
  TIMSK4 |= (1 << OCIE4A);
  interrupts();
}

void A4988::setMove(int spd, int dir) {
  a4988_waitTime = 1000000 / spd / 800 - 1;
  //Serial.println(a4988_waitTime);
  a4988_dir = dir;

  a4988_stop = (dir == STOP);

  if (spd == 0) a4988_dir = STOP;

  bool _state = a4988_dir == RIGHT ? HIGH : LOW;
  digitalWrite(pinDir, _state);

}

ISR (TIMER4_COMPA_vect)
{
  if (a4988_stop) return;

  if (k++ >= a4988_waitTime) {
    PORTE |= (1 << PE5);
    k = 0;
  } else if (k == 1) {
    PORTE &= (0 << PE5);
  }
}

