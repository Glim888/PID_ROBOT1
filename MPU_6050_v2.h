#ifndef __MPU6050_V2__
#define __MPU6050_V2__

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define OFFSET_STEPS 10000    // Kalibrier Stepps 
#define G_IN_VALUE 16384.     // 16384 bedeutet, das 1 G wirkt (bei +-2g)
#define DEG_IN_VALUE 131.     // 131 bedeutet, das um 1° gedreht wurde (bei +-250°)
#define FREQUENCY 500         // Timerinterrupt Frequenz
#define FILTER_ACC 0.03       // complementary  filter
#define FILTER_GYRO 0.93      // complementary  filter


class MPU_6050_v2 {

  private:
    int mpu_adress;

    bool bluetoothIsEnabled = false;

    SoftwareSerial *bt;
    
    float ac_xOffset = 350.00;
    float ac_yOffset = 529.00;
    float ac_zOffset = 1568.00;

    float gy_xOffset = -374.00;
    float gy_yOffset = -503.00;
    float gy_zOffset = -266.00;

    float angle = 0;

    long ac_xSum = 0, ac_ySum = 0, ac_zSum = 0, gy_xSum = 0, gy_ySum = 0, gy_zSum = 0;
    int offset_counter = 0;

    void get_info(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z);

  public:

    MPU_6050_v2 ();
    // MPU initialisieren
    void init(int _adress);
    // Bluetooth Objekt übergeben 
    void set_bluetooth(SoftwareSerial *_bt);  
    // Register auslesen
    byte get_register(byte _register);
    // Register setzen
    void set_register(byte _register, byte _data);
    // Ausgabe der Sensorwerte
    void test(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z);
    // Rohdaten auslesen
    void get_rawData(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z);
    // Werte auslesen im Maßeinheitenformat
    void get_data(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z);
    // MPU kalibrieren
    void get_offset(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z, int setAccX, int setAccY, int setAccZ);
    // Winkel auslesen
    float get_angle(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z);
    // Beschleunigungswinkel auslesen
    float get_acc_angle(float *ac_x, float *ac_y, float *ac_z, float *gy_x, float *gy_y, float *gy_z);
    // ändern des Gyro Winkels
    void set_gyro_angle(float _angle);
};

#endif
