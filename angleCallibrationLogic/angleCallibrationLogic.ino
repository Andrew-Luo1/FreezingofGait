/*
 # Product: 6 DOF Sensor-MPU6050 
 # SKU    : SEN0142 
 # Description:     
 # To read  accel/gyro data from 6 DOF Sensor 
*/

#include "Wire.h"                 
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;  
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz  
double dax, day, daz;        
#define LED_PIN 6
#define LED_PIN_FREEZING 7
bool blinkState = false;

double angle = 0;

void setup() {
  Wire.begin();      // join I2C bus   
  Serial.begin(9600);    //  initialize serial communication
  accelgyro.initialize();  
  pinMode(LED_PIN, OUTPUT);  // configure LED pin
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  Serial.print(millis());
  Serial.print(",");
  Serial.print(ax); 
  Serial.print(",");
  Serial.print(ay); 
  Serial.print(",");
  Serial.print(az); 
  Serial.print(",");
  Serial.print(gx); 
  Serial.print(",");
  Serial.print(gy); 
  Serial.print(",");
  Serial.println(gz);
  
  dax = (double) ax;
  day = (double) ay;
  daz = (double) az;

  angle = ( acos(day / sqrt((dax*dax) + (day*day) + (daz*daz))) - PI/2 );
  if(millis() < 3000 && abs(angle) < 0.1){  
    digitalWrite(LED_PIN, HIGH);
  }
  else{
    digitalWrite(LED_PIN, LOW);
  }



}
