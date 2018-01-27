#include "MPU_6050_v2.h"

volatile float deltaGyro = 0;
volatile float angle_gyro = 0;

MPU_6050_v2::MPU_6050_v2 () {
  Wire.begin();
}

void MPU_6050_v2::init(int _adress) {

  mpu_adress = _adress;

  Wire.beginTransmission(mpu_adress);

  // deaktivieren des Stromsparmodus
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Auflösung des Gyros + Beschleunigungssensors
  set_register(0x1B, 0x00);
  set_register(0x1C, 0x00);

  // Dämpfung
  set_register(0x1A, 0x06);

  // Timerinterrupt aktivieren
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 62500 / FREQUENCY;
  TCCR1B |= (1 << WGM12);
  //TCCR1B |= (1 << CS10);
  //TCCR1B |= (1 << CS11);
  TCCR1B |= (1 << CS12);  // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

}

void MPU_6050_v2::test(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z) {

  get_data(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z);
  get_info(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z);

  delay(200);
}

void MPU_6050_v2::get_rawData(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z) {

  Wire.beginTransmission(mpu_adress);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_adress, 14, true); // 14 Register abfragen und Verbindung beenden

  // 2 Bytes zusammenfügen und in der jeweiligen Variable abspeichern
  *ac_x = Wire.read() << 8 | Wire.read(); // 0x3B & 0x3C
  *ac_y = Wire.read() << 8 | Wire.read(); // 0x3D  & 0x3E
  *ac_z = Wire.read() << 8 | Wire.read(); // 0x3F  & 0x40
  // 2 unnütze Bytes
  Wire.read(); // 0x41
  Wire.read(); // 0x42
  *gy_x = Wire.read() << 8 | Wire.read(); // 0x43  & 0x44
  *gy_y = Wire.read() << 8 | Wire.read(); // 0x45  & 0x46
  *gy_z = Wire.read() << 8 | Wire.read(); // 0x47  & 0x48

  //Offsets berechnen
  *ac_x += ac_xOffset;
  *ac_y += ac_yOffset;
  *ac_z += ac_zOffset;

  *gy_x += gy_xOffset;
  *gy_y += gy_yOffset;
  *gy_z += gy_zOffset;

}

void MPU_6050_v2::get_data(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z) {

  get_rawData(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z);

  // Raw Value in Maßeinheiten umrechnen
  *ac_x /= G_IN_VALUE;
  *ac_y /= G_IN_VALUE;
  *ac_z /= G_IN_VALUE;

  *gy_x /= DEG_IN_VALUE;
  *gy_y /= DEG_IN_VALUE;
  *gy_z /= DEG_IN_VALUE;

  deltaGyro = *gy_z;
}

void MPU_6050_v2::set_gyro_angle(float _angle){
  angle_gyro = _angle;
}

float MPU_6050_v2::get_acc_angle(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z) {
  return asin(*ac_y / sqrt(pow(*ac_x, 2) + pow(*ac_y, 2) + pow(*ac_z, 2))) * 57.295779513082320876798154814105;
}

float MPU_6050_v2::get_angle(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z) {

  float acc_angle = get_acc_angle(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z);

  // Complimentary Filter
  angle_gyro = acc_angle * FILTER_ACC + angle_gyro * FILTER_GYRO;

  return angle_gyro;
}

ISR (TIMER1_COMPA_vect)
{
  angle_gyro += deltaGyro / FREQUENCY;
}

void MPU_6050_v2::set_bluetooth(SoftwareSerial *_bt) {
  bt = _bt;
  bluetoothIsEnabled = true;
  bt->println("MPU init...done");
}

void MPU_6050_v2::set_register(byte _register, byte _data) {

  Wire.beginTransmission(mpu_adress);
  Wire.write(_register);
  Wire.write(_data);
  Wire.endTransmission(false);
}

byte MPU_6050_v2::get_register(byte _register) {

  Wire.beginTransmission(mpu_adress);
  Wire.write(_register);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_adress, 1, true);

  return Wire.read();
}

/*
  #Ermittlung der Offsets des Sensors (Aufgrund der Produktionstoleranz)
  #während Messung nicht berühren/ bewegen -> außerdem muss der Roboter gerade stehen, sonst weiß man den Soll Wert nicht.
  #setAccXYZ -> Sollwert einstellen, den Wert auf den der Sensor geeicht werden soll
    --> 2 Werte müssen 0 sein und ein Wert 1G
  # Ergebnisse in die globalen Offset Variablen eintragen
    --> alle Gyro Werte sollten nun 0 sein genau so wie 2 von 3 Acc Werte. Ein wert sollte 1G sein


   #Wie sollen die Werte mit Offset sein? -> wenn er gerade auf meinem Schreibisch liegt mit LED nach oben
   ac_x = 0;
   ac_y = 0;
   ac_z = 1;
   gy_x = 0;
   gy_y = 0;
   gy_z = 0;
*/

void MPU_6050_v2::get_info(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z) {

  if (bluetoothIsEnabled) {
    bt->print("AcX = "); bt->print(*ac_x);
    bt->print(" | AcY = "); bt->print(*ac_y);
    bt->print(" | AcZ = "); bt->print(*ac_z);
    bt->print(" | GyX = "); bt->print(*gy_x);
    bt->print(" | GyY = "); bt->print(*gy_y);
    bt->print(" | GyZ = "); bt->print(*gy_z);
    bt->print(" | Angle = "); bt->println(get_angle(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z));
    bt->println("---------------------");
  }
}

void MPU_6050_v2::get_offset(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z, int setAccX, int setAccY, int setAccZ) {

  if (bluetoothIsEnabled) {
    if (offset_counter == 0) {
      bt->println("Start measurement");

      // Offsets nullen
      ac_xOffset = 0;
      ac_yOffset = 0;
      ac_zOffset = 0;
      gy_xOffset = 0;
      gy_yOffset = 0;
      gy_zOffset = 0;

      // Zähler nullen
      offset_counter = 0;
      ac_xSum = 0;
      ac_ySum = 0;
      ac_zSum = 0;
      gy_xSum = 0;
      gy_ySum = 0;
      gy_zSum = 0;
    }

    get_rawData(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z);

    ac_xSum += *ac_x;
    ac_ySum += *ac_y;
    ac_zSum += *ac_z;
    gy_xSum += *gy_x;
    gy_ySum += *gy_y;
    gy_zSum += *gy_z;

    if (offset_counter % (OFFSET_STEPS / 10) == 0) bt->print(".");
    if (offset_counter++ >= OFFSET_STEPS) {

      ac_xOffset += setAccX - (ac_xSum / OFFSET_STEPS);  // wird auf setAccX kalibriert
      ac_yOffset += setAccY - (ac_ySum / OFFSET_STEPS);  // wird auf setAccY kalibriert
      ac_zOffset += setAccZ - (ac_zSum / OFFSET_STEPS);  // wird auf setAccZ kalibriert

      gy_xOffset += -gy_xSum / OFFSET_STEPS;
      gy_yOffset += -gy_ySum / OFFSET_STEPS;
      gy_zOffset += -gy_zSum / OFFSET_STEPS;

      // Offsets anzeigen
      bt->println("");
      bt->print("offset ACC_X = "); bt->println(ac_xOffset);
      bt->print("offset_ACC_Y = "); bt->println(ac_yOffset);
      bt->print("offset ACC_Z = "); bt->println(ac_zOffset);
      bt->print("offsetGY_X = "); bt->println(gy_xOffset);
      bt->print("offsetGY_Y = "); bt->println(gy_yOffset);
      bt->print("offsetGY_Z = "); bt->println(gy_zOffset);
      bt->println("---------------------");

      get_data(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z);

      // neue Werte (mit Offsets)
      get_info(ac_x, ac_y, ac_z, gy_x, gy_y, gy_z);

      delay(400000);
    }
  }
}


