#include "MPU_6050_v2.h"
#include "A4988.h"
#include "PID.h"
#include <Serial_Input.h>
#include <SoftwareSerial.h>

long lastTime = millis();

long tick = 0;

////A4988////
A4988 a4988_l(A14, 3);
A4988 a4988_r(4, 3);

////MPU6050////
float ac_x, ac_y, ac_z, gy_x, gy_y, gy_z;
MPU_6050_v2 mpu;

////PID////
float setpoint = 0;
PID pid(300, 20, 0.03, &setpoint);

////Serial////
Serial_Input si;
SoftwareSerial bt(13, 12);
String data;

void setup() {
  bt.begin(9600);
  bt.println("Bluetooth init...done ");
  Serial.begin(9600);

  mpu.init(0x68);
  mpu.set_bluetooth(&bt);
  a4988_l.init();
  a4988_r.init();

  //PID
  pid.set_limits(-600, 600);

  // Schraubklemmen
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);

  //MSx
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  //LED´s
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  digitalWrite(5, LOW);  // MS3
  digitalWrite(6, HIGH); // MS2
  digitalWrite(7, HIGH); // MS1

  // Wenn der Roboter gerade steht, dann wird die Endlosschleife verlassen & das Balancieren kann beginnen
  // Gyrowinkel wird auf Acc Winkel gesetzt
  do {
    mpu.get_data(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z);
  } while (abs(mpu.get_acc_angle(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z)) > 0.05);

  mpu.set_gyro_angle(mpu.get_acc_angle(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z));
}

void loop() {
  //mpu.get_offset(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z, -G_IN_VALUE, 0, 0);
  //mpu.test(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z);

  // Serial Data empfangen
  if (tick++ % 1000 == 0) {
    handleSerial();
  }

  // Balancing
  if (millis() - lastTime > 40) {
    lastTime = millis();
    mpu.get_data(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z);
    float _angle = mpu.get_angle(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z);
    if (abs(_angle) < 10)
      balancing(_angle);
  }

}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/*
   PID
*/

void balancing (float _angle) {
  //myPID.Compute();
  float _output = pid.calc(_angle);

  if (tick % 15 == 0) {
    Serial.print("Angle: "); Serial.println(_angle);
    Serial.print("SP: "); Serial.println(setpoint);
  }

  if (_output > 0) setpoint -= 0.005;
  if (_output < 0) setpoint += 0.005;

  // LED
  if (_angle > setpoint + 0.1) {
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
  } else if (_angle < setpoint - 0.1) {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  } else {
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
  }


  if (_angle > setpoint + 0.05) {
    a4988_l.setMove(abs(_output), RIGHT);
    a4988_r.setMove(abs(_output), LEFT);
  } else if (_angle < setpoint - 0.05) {
    a4988_l.setMove(abs(_output), LEFT);
    a4988_r.setMove(abs(_output), RIGHT);
  } else {
    a4988_l.setMove(0, STOP);
    a4988_r.setMove(0, STOP);
  }

}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/*
   Serial & Funktionen, welche über Serial gesteuert werden
*/

String btGetSerial() {
  if (bt.available()) {
    return bt.readString();
  } else {
    return "NULL";
  }
}

void handleSerial() {
  data = btGetSerial();

  if (data != "NULL") {
    String _cmd = si.getCmd(data);
    if (_cmd == "MOV") MOV(data);
    if (_cmd == "PID") TUNE(data);
    if (_cmd == "ANG") bt.println(mpu.get_angle(&ac_x, &ac_y, &ac_z, &gy_x, &gy_y, &gy_z));
  }
}

void TUNE(String _data) {
  float _p = si.getData(_data, 4, 4);
  float _i = si.getData(_data, 9, 4);
  float _d = si.getData(_data, 14, 4);

  bt.println(_p);
  bt.println(_i);
  bt.println(_d);
  bt.println("########");


  pid.set_pid(_p, _i, _d / 100);
  bt.println(pid.get_P());
  bt.println(pid.get_I());
  bt.println(pid.get_D());

}

void MOV (String _data) {
  int _d = si.getData(_data, 4, 1);
  int _spd = si.getData(_data, 6, 4);
  int _dir_l = 0;
  int _dir_r = 0;
  switch (_d) {
    case 0: _dir_l = STOP; _dir_r = STOP; break;
    case 1: _dir_l = LEFT; _dir_r = RIGHT; break;
    case 2: _dir_l = RIGHT; _dir_r = LEFT; break;
  }

  a4988_l.setMove(_spd, _dir_l);
  a4988_r.setMove(_spd, _dir_r);
}




