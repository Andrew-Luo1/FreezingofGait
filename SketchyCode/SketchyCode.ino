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
#define LED_PIN 7
bool blinkState = false;
double dax, day, daz;
double dgx, dgy, dgz;
double angle = 0;

void setup() {
  Wire.begin();      // join I2C bus   
  Serial.begin(9600);    //  initialize serial communication
//  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();  

  // verify connection
//  Serial.println("Testing device connections...");
//  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(LED_PIN, OUTPUT);  // configure LED pin
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  // display tab-separated accel/gyro x/y/z values
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
  Serial.print(gz);
  Serial.print(",");
  dax = (double) ax;
  day = (double) ay;
  daz = (double) az;
  angle = ( acos(day / sqrt((dax*dax) + (day*day) + (daz*daz))) );
  Serial.println(angle);
  
  //a) Stopping
//if your angle is around zero for more than a few readings and your x,y,z are within a certain threshold 
  //if gyration is zero
  //b) walking
  //c) freezing of gait 

}
