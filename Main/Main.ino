
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// angle and accleration data variable
double angCal, accCal, angNew, accWorldX;
const int arrSize = 13;
const int midIndex = (arrSize+1)/2; 
double angleArr[arrSize];

// Feature 1 varialbes
unsigned long prevTime, currTime;
double strideStartTime;
float strideFreq = 0;

// Feature 2 variables
double maxAngle = 0;

// Weighted moving average filter variables
const double cutoffFreq = 20;
const double sampleTime = 30; 
const double sampleFreq = 1000/sampleTime;
const double elapsed = 1/sampleFreq;
const double tauUS = 1/cutoffFreq;
const double ampFactor = exp((-1)*elapsed/tauUS);
double prevAx;


//SVM Hardcoded values.

double w[] = {0.1681, -0.6992};
double b = -3.5986; 

int freezeCount[] = {0, 0};




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(57600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    Serial.print("Wait for Calibration ...");
}


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        // initialize values
        angNew = ypr[1]*180/M_PI;
        prevTime = currTime;
        currTime = millis();
        accWorldX = aaWorld.x;
        if(millis() < 30000) {       //it takes about 40 seconds for the values to callibrate themselves. 
          Serial.print(millis());
          Serial.print(", ");
          Serial.print(angNew);
          Serial.print(", ");
          Serial.println(accWorldX);
          
          return;
        }

        // calibrate angle and horizontal accerlation
        if(angCal == 0){
          angCal = angNew;
          accCal = accWorldX;
        }
        accWorldX = accWorldX - accCal;
        angNew = angNew - angCal;
        accWorldX = movingAvgFilter(prevAx, accWorldX);
        prevAx = accWorldX; //need two values for filter.

        if(millis()< 50000){
          Serial.print(millis());
          Serial.print(", ");
          Serial.println(angNew);
          return;      
        }


        updateAngArr(angNew);
        if(millis() < 20000) { //this and below 2 lines prob useless. 
          return;
        }
        
        //if user standing still, break main loop.
        if(isStill(accWorldX)){
          return; 
        }
        
        if(!isStep()) {
          //memset(freezeCount,0,sizeof(freezeCount)); //Set both values back to 0.

          return;
        } else {
          maxAngle = angleArr[midIndex-1];                                // feature 1
          strideFreq = 1 / ((millis() - strideStartTime) / 1000); // feature2
          Serial.print(maxAngle);
          Serial.print(", ");
          Serial.println(strideFreq);
          strideStartTime = millis();
          //Detect if it's freezing using SVM. 
          
          //Comparing incoming features to SVM decision boundary. 
          if(-(w[0]*maxAngle + b)/w[1] < strideFreq){
            //If you get "freezing" twice in a row, it's freezing. 
            freezeCount[1] = freezeCount[0];
            freezeCount[0] = 1; 
            if(freezeCount[1] == 1){ //2nd value of freezecount would only be 1 if you freeze twice in a row. (Or else it gets reset)
              Serial.println("Freezing!");
            } 
          }
          else{
          freezeCount[0] = 0;
          freezeCount[1] = 0; 
          }
         
          
        }
    } else {
      //Serial.println("else statement was called");
    }
}

/**
 * Check if user is standing still
 * 
 * @param accWorldX horizontal accleration with gravity correction
 * @return boolean indicate whether user is still
 */
boolean isStill(double accWorldX) {
  if(abs(accWorldX) < 200) {
    return true;
  } else {
    return false;
  }
}
    
/**
 * Move all angles right by one and insert new angle at first index
 * 
 * @param angArr previous angle array
 * @param newAng new angle to be inserted
 * @return angArr updated angle array
 */
void updateAngArr(double newAng) {    
  for(int k = arrSize-1; k>0; k--){
    angleArr[k] = angleArr[k-1];
  }
  angleArr[0] = newAng;
  return;
}

/**
 * Check if array contains a peak
 * 
 * @param angArr current angle array
 * @return boolean whether is step or not
 * This function essentially separates the signals into individual steps. 
 */
boolean isStep() {
    for(int j=0; j<arrSize-1; j++){
      if(angleArr[j]>angleArr[midIndex-1]){
        return false;
      }
    }
    return true;
}

double movingAvgFilter( double previousOutput, double currentInput )
{
  double currentOutput = (1 - ampFactor)*currentInput + (ampFactor)*previousOutput;
  return currentOutput;
}
