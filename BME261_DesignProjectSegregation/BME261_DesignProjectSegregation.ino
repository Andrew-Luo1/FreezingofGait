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
double dgx, dgy, dgz;
double angle = 0;
double dt;
double oldTime = 0;

//segregation variables
double detectIfClose = 0;
double timeQueue[] = {0, 0, 0, 0};
double gaitStore[] = {0,0, 0, 0, 0, 0, 0, 0}; //gait length cycle through these values.

//The rolls-royce of the code
double gaitLength = 0;

void setup() {
  Wire.begin();      // join I2C bus   
  Serial.begin(9600);    //  initialize serial communication
  accelgyro.initialize();  
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  // display tab-separated accel/gyro x/y/z values
  //Serial.print(millis());
  //Serial.print(",");
//  Serial.print(ax); 
//  Serial.print(",");
//  Serial.print(ay); 
//  Serial.print(",");
//  Serial.print(az); 
//  Serial.print(",");
//  Serial.print(gx); 
//  Serial.print(",");
//  Serial.print(gy); 
//  Serial.print(",");
//  Serial.println(gz);

  dax = (double) ax;
  day = (double) ay;
  daz = (double) az;
  dt = millis() - oldTime;
  oldTime = millis();
  angle = ( acos(day / sqrt((dax*dax) + (day*day) + (daz*daz))) - PI/2 );
  //Serial.print(dt); 
//  Serial.print(",");
//  Serial.print(angle);
//  Serial.print(",");


  if(abs(angle) < 0.01){ //Change based on sampling rate. Higher sampling rate = larger threshold
    timeQueue[3] = timeQueue[2];
    timeQueue[2] = timeQueue[1];
    timeQueue[1] = timeQueue[0];
    timeQueue[0] = millis();
    detectIfClose = timeQueue[0] - timeQueue[1];

    //If four time points are very close to eachother, you're probably freezing. (or stopping) 
    //If the points are decently spaced ("decent" by an arbitrary measure), it's simple.
    //If two points are very close to eachother, do nothing for now and see if the next few points sate the first if statment 
    
    if(timeQueue[0]-timeQueue[3] < (dt * 10)){ //10 is arbitrarily chosen, use matlab code to see what's up. 
      gaitLength = timeQueue[0]-timeQueue[3]; 
      Serial.print("hello");
      Serial.println(gaitLength); 
    }
    
    if(detectIfClose > dt* 6){
      gaitLength = detectIfClose;
      Serial.println(gaitLength);
    } 
    
    //POTENTIAL ERROR: I SHOULD USE BREAK OR CONTINUE WHEN TWO POINTS CLOSE OR ELSE REST OF LOOP WILL INTERACT WITH IT!
  }
      //if the same value repeats 6 times, set gait length to zero.
      
//  Serial.println(gaitLength);
}


