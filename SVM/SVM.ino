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

//SVM variables
double w[] = {1.4099, 2.1296};
double b = -10.3315;
int freezeCount[] = {0, 0, 0, 0};
double f1 = 0; //fillers for the features. For example f1 would be scaled gait length
double f2 = 0; 

void setup() {
  Wire.begin();      // join I2C bus   
  Serial.begin(9600);    //  initialize serial communication
//  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();  

}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  // display tab-separated accel/gyro x/y/z values
  Serial.print(millis());


//SVM decision boundary comparison engine. If classified as freezing 4 times in a row, "vibrate the motor"
//This is assuming that below the boundary is freezing and below is normal.  
  if(-(w[0]*f1+b)/w[1] < f2){
      freezeCount[3] = freezeCount[2];
      freezeCount[2] = freezeCount[1];
      freezeCount[1] = freezeCount[0];
      freezeCount[0] = 1;
      if(freezeCount[3] == 1){
        //flash LED
        int a = 0; 
      }
  }
  else{ //reset all values to 0, aka false. 
    memset(freezeCount,0,4); 
  }
  

}
