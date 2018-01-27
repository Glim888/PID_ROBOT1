#include "Serial_Input.h"


String Serial_Input::getCmd(String _s)
{
  String _cmd = "";
  for (int _i = 0; _i < 3; _i++)
  {
    _cmd += _s.charAt(_i);
  }
  return _cmd;
}
/*
   ließt den Serial buffer aus
*/
String Serial_Input::getSerial()
{
  if (Serial.available())
    return Serial.readString();
  else
    return "NULL";
}
/*
   data, startPos, len
*/
int Serial_Input::getData(String _s, int _start, int _len)
{
  String _d = "";
  for (int _i = _start; _i < _start + _len; _i++)
  {
    _d += _s.charAt(_i);
  }
  return _d.toInt();
}
