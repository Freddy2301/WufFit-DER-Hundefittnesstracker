// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_LSM9DS1.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_Sensor.h>


void setup() {

 
  
 
}

void loop() {
    Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // i2c sensor
 
 if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS1 ... check your connections */
    Particle.publish("Check your wiring!", PRIVATE);
    while(1);
  }
  
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

 sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  //new
  // Get some data
  //String data = String(10);
  // Trigger the integration
  //Particle.publish("acceloration_sensor", data, PRIVATE);
  //Particle.publish("acceloration_sensor", String(accel.acceleration.x), PRIVATE);
  Particle.publish("acceleration_x", String(accel.acceleration.x), PRIVATE);
  Particle.publish("acceleration_y", String(accel.acceleration.y), PRIVATE);
  Particle.publish("acceleration_z", String(accel.acceleration.z), PRIVATE);
  //Particle.publish("acceloration_sensor", String(accel.acceleration.x), PRIVAT);
 
 /*
 Serial.print("x: ");
 Serial.print(accel.acceleration.x);
 Serial.print(" ; ");
 
 Serial.print("y: ");
 Serial.print(accel.acceleration.y);
 Serial.print(" ; ");
 
 Serial.print("z: ");
 Serial.print(accel.acceleration.z);
 Serial.println(" ; ");
 */

  delay(500);

}
