#include <Wire.h>

int LED = 13;
int x = 0;
int angle = 0;
void setup() {
  // Define the LED pin as Output
  Serial.begin(38400);
  Serial.println("Hello");
  pinMode (LED, OUTPUT);
  // Start the I2C Bus as Slave on address 9
  Wire.begin(9); 
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
}
void receiveEvent(int bytes) {
  int16_t data[12];
  for (int i = 0; i < bytes; i++){
    data[i] = Wire.read();
  }
  int16_t Ax = (data[0]*256) |  (data[1]);
  int16_t Ay = (data[2]*256) |  (data[3]);
  int16_t Az = (data[4]*256) |  (data[5]);
  int16_t Gx = (data[6]*256) |  (data[7]);
  int16_t Gy = (data[8]*256) |  (data[9]);
  int16_t Gz = (data[10]*256) |  (data[11]);
  
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
  angle = ( acos(Ay / sqrt((Ax*Ax) + (Ay*Ay) + (Az*Az))) );
  Serial.println(angle);

  /*
  //Serial.print("Received: ");
  //Serial.print(bytes);
  for (int i = 0; i < bytes; i++) {
    x = Wire.read();    // read one character from the I2C
    Serial.print(" ");
    Serial.print(x, HEX);    
  }
  Serial.println();
  */
}
void loop() {
  //If value received is 0 blink LED for 200 ms
  delay(10);
}
