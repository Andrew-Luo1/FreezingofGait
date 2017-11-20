#include <Wire.h>

//For the filters
double times[]    = {0,0};
double values[]   = {0,0};

// VARIABLES FOR WEIGHTED MOVING AVERAGE FILTER
double cutoffFreq;
double sampleTime;
double sampleFreq;
double elapsed;
double tauUS;
double ampFactor;
double prevGz;
double prevAx;

int LED = 13;
double queueGz[15] = {};
double queueAx[15] = {}; 
int x = 0;
int angle = 0;
int i = 0;
int k = 0;
double meanGz = 0;
int16_t Ax = 0;
/*
int16_t Ay = 0;
int16_t Az = 0;
int16_t Gx = 0;
int16_t Gy = 0;
//*/
int16_t Gz = 0;
boolean isLedOn = false;

//For callibration:
int16_t cAx, cAy, cGz;
int count = 0;

void setup() {
  cutoffFreq = 20;
  sampleTime = 30;
  sampleFreq = 1000/sampleTime;
  elapsed = 1/sampleFreq;
  tauUS = 1/cutoffFreq;
  ampFactor = exp((-1)*elapsed/tauUS);
  Serial.begin(38400);
  Serial.println("Hello");
  pinMode (LED, OUTPUT);
  Wire.begin(9); 
  Wire.onReceive(receiveEvent);
}
void receiveEvent(int bytes) {
  int16_t data[bytes];
  for (int i = 0; i < bytes; i++){
    data[i] = Wire.read();
  }

  if(millis() < 1200) {
    count++;
    cAx = (cAx + (data[0]*256)|(data[1])) / count;
    cAy = (cAy + (data[2]*256)|(data[3])) / count;
    cGz = (cGz + (data[10]*256)|(data[11])) / count;
  }

  Ax = (data[0]*256) |  (data[1]);
  /*
  Ay = (data[2]*256) |  (data[3]);
  Az = (data[4]*256) |  (data[5]);
  Gx = (data[6]*256) |  (data[7]);
  Gy = (data[8]*256) |  (data[9]);
  //*/
  Gz = (data[10]*256) |  (data[11]);

  /*
  Serial.print(millis());
  Serial.print(",");
  Serial.print(Ax);
  Serial.print(",");
  Serial.print(Ay);
  Serial.print(",");
  Serial.print(Az);
  Serial.print(",");
  Serial.print(Gx);
  Serial.print(",");
  Serial.print(Gy);
  Serial.print(",");
  Serial.println(Gz);
  //*/
  
}
void loop() {
  if(i > 14){ //Array is 15 indicies long, largest index is 14. 
    i = 0; 
  }
  Gz = movingAvgFilter(prevGz, Gz-cGz);
  Ax = movingAvgFilter(prevAx, Ax-cAx);
  
  queueGz[i] = Gz;
  for(int j = 0; j < 15; j++) {
    meanGz = meanGz + queueGz[i];
  }
  meanGz = meanGz / 15;

  

  queueAx[i] = Ax;
  if(meanGz > 25000000){  //The filter magnifies the values. 
    //FOG: Light up LED
    boolean isStop = true;
    for(int j = 0; j < 15; j++) { 
      if(queueAx[j] > 10000000) {
        isStop = false;
        break;
      }
    }
    
    if(isStop) {
      Serial.print("STOPPED");
      if(isLedOn) {
        isLedOn = false;
      }
    } else {
      Serial.print("FREEZE");
      digitalWrite(LED, HIGH);
    }
  } else {
    if(isLedOn) {
      isLedOn = false;
    }
  }
  
  prevGz = Gz - cGz;
  prevAx = Ax - cAx;
  
}
double movingAvgFilter( double previousOutput, double currentInput )
{
  double currentOutput = (1 - ampFactor)*currentInput + (ampFactor)*previousOutput;
  return currentOutput;
}
