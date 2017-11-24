#include <Wire.h>

// VARIABLES FOR DATA STREAMING
int16_t Gz;
const int capacity = 100;
double GzFiltered[capacity];
int current;

// VARIABLES FOR WEIGHTED MOVING AVERAGE FILTER
double cutoffFreq, sampleTime, sampleFreq, elapsed;
double tauUS, ampFactor, prevGz, prevAx, oldTime;


void setup()
{
  // SET UP DATA STREAMING
  Serial.begin(38400);
  Wire.begin(9);
  Wire.onReceive(receiveEvent);

  // SET UP THE CIRCULAR ARRAY FOR DATA
  GzFiltered[0] = 0;
  current = 1;
}

void loop()
{
  //
}


// FUNCTION FOR RECEIVING DATA
void receiveEvent(int bytes)
{
  int16_t data[bytes];
  for (int i = 0; i < bytes; i++)
  {
    data[i] = Wire.read();
  }
  Gz = (double)((data[10]*256)|(data[11]));
  
  GzFiltered[current] = movingAvgFilter( GzFiltered[(current - 1)%capacity], Gz );
  current = (current++) % capacity;
}


// FUNCTION FOR WEIGHTED MOVING AVERAGE FILTER
double movingAvgFilter( double previousOutput, double currentInput )
{
  double currentOutput = (1 - ampFactor)*currentInput + (ampFactor)*previousOutput;
  return currentOutput;
}
