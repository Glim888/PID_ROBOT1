#ifndef __SERIAL_INPUT__
#define __SERIAL_INPUT__

#include <Arduino.h>


class Serial_Input {

public:

///Funktionen für andere C Datein

/*
 * getCmd gibt den Befehl als String an die aufrufende Funktion zurück
 * _s ist der String, von getSerial stammt
 */
String getCmd(String _s);

/*
 * getSerial gibt entweder "NULL" oder die Eingabe des Users an die aufrufende Funktion zurück
 */
String getSerial();

/*
 * getData gibt weitere Parameter zurück (welche vom User nach dem Befehlt eingegeben wurden)
 * _s ist der String von getSerial(); _start ist der Start Index und _len die länge der Eingabe
 */
int getData(String _s, int _start, int _len);

};
#endif
