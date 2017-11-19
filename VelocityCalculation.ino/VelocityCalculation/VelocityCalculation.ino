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

//gravity-corrected ay and ax
double nay = 0;
double nax = 0;
double ayTrue = 0;

double oldTime = 0;
double dt = 0; 
double v = 0;
double aveV = 0;
double t = 0; 
  
#define LED_PIN 13
bool blinkState = false;

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

  //calculate angle 

  dax = (double) ax;
  day = (double) ay;
  daz = (double) az;

  angle = ( acos(day / sqrt((dax*dax) + (day*day) + (daz*daz))) - PI/2 );
  
  Serial.print(angle);
  Serial.print(",");

  //remove influence of gravity on ay, nay. g is 16384 in the scaling system of the imu. 
  nay = day + 16384*sin(angle);
  Serial.print(nay);
  Serial.print(",");
  
  //remove influence of gravity on ax, nax
  nax = dax - 16384*cos(angle) - 400; //this gives a algebraicly better looking number. Idk why it's not plus. Subtract 400 ghetto. 
  Serial.print(nax);
  Serial.print(",");
  //calculate acceleration in absolute y.(y is horizontal in our case) 

  ayTrue = nay*cos(angle) + nax*sin(angle); //horizontal velocity greater than components because they add up. 
  //Serial.println(ayTrue);


  //calculate velocity using accelerometer data 
  dt = millis() - oldTime;
  oldTime = millis();

  //Filter out the noise with most ghetto filter of life 
  if (nay > 20 && nax > 100 || nay < -20 && nax < -100){
    v = v + ayTrue*dt;
    Serial.println(v);
  }
  else{
    v = 0;
    Serial.println(v);
  }

  

  //calculate velocity using gyro data and average the two
 

  //Length of time of gait times the average velocity of the gait. 
  //For testing rn, literally just take info 50 ms, find averages. Shake the device quickly then see how the value changes. 
  

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
