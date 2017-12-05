#include <Wire.h>

// VARIABLES FOR WEIGHTED MOVING AVERAGE FILTER
double cutoffFreq, sampleTime, sampleFreq, elapsed;
double tauUS, ampFactor, prevGz, prevAx, oldTime;

// VARIABLES FOR FOG DETECTION
int LED = 13;
const int n = 150;
double queueGz[n] = {};
double queueAx[n] = {}; 
int x = 0;
int angle = 0;
int i = 0;
int k = 0;
double meanGz = 0;
int16_t Ax = 0;
int16_t Gz = 0;
boolean isLedOn = false;

// VARIABLES FOR CALIBRATION
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
    cAx = (cAx + (data[0]*256)|(data[1])) / count; //using the bitwise or. 
    cAy = (cAy + (data[2]*256)|(data[3])) / count;
    cGz = (cGz + (data[10]*256)|(data[11])) / count;
  }
  Ax = (data[0]*256)|(data[1]);
  Gz = (data[10]*256)|(data[11]);
  
}

void loop() {
  if(i > n - 1){ //Array is n indicies long, largest index is 14. 
    i = 0; 
  }
  
  Gz = movingAvgFilter(prevGz, Gz-cGz);
  Ax = movingAvgFilter(prevAx, Ax-cAx);
  queueGz[i] = Gz;
  for(int j = 0; j < n; j++) {
    meanGz = meanGz + queueGz[i];
  }
  meanGz = meanGz / n;
  Serial.print(Ax);
  queueAx[i] = Ax;
  if(meanGz < 500){
    boolean isStop = true;
    for(int j = 0; j < n; j++) { 
      if(queueAx[j] > 5000) {
        isStop = false;
        break;
      }
    }
    
    if(isStop) {
      Serial.println("STOPPED");
      if(isLedOn) {
        isLedOn = false;
      }
    } else {
      Serial.println("FREEZE");
      digitalWrite(LED, HIGH);
    }
  } else {
    Serial.println ("WALKING");
    if(isLedOn) {  
      isLedOn = false;
    }
  }
  i++;
  prevGz = Gz - cGz;
  prevAx = Ax - cAx;  
}

double movingAvgFilter( double previousOutput, double currentInput )
{
  double currentOutput = (1 - ampFactor)*currentInput + (ampFactor)*previousOutput;
  return currentOutput;
}
