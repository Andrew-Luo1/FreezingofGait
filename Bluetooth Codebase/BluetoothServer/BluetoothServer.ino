#include  <BLE_API.h>
#include <Wire.h>
//#include <math.h>
#define DEVICE_NAME   "BLE_Accelerometer"
#define TXRX_BUF_LEN  20

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;


//#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
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
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

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


//SVM Hardcoded values

double w[] = {0.0086, -2.9729};
double b = 3.1642; 

int freezeCount[] = {0, 0};
int vibrateTime = 0;
bool vibrateOn = false;

// Creation of BLE instance
BLE       ble;
// Creation of timer task
Ticker    ticker1s;

// uuid is unique universal id, it acts as a device name
// written in hex (0x) acts as identifier for hex, the next two digits is a 2 bit hex number
// service and characteristic uuids
static const uint8_t service1_uuid[]          = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_chars1_uuid[]   = {0x71, 0x3D, 0, 1, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_chars2_uuid[]   = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
//reverse of service id, used in advertisement
static const uint8_t service1_uuid_rev[]    = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

// Initialization of chars1 values
uint8_t chars1_value[TXRX_BUF_LEN] = {0};
uint8_t chars2_value[TXRX_BUF_LEN] = {0};

// Create a characteristic
GattCharacteristic  characteristic1(service1_chars1_uuid, chars1_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );
GattCharacteristic  characteristic2(service1_chars2_uuid, chars2_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};

GattService         service1(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

DeviceInformationService *deviceInfo;

void disconnectionCallBack( const Gap::DisconnectionCallbackParams_t *params) {
  ble.startAdvertising();
}

void connectionCallBack( const Gap::ConnectionCallbackParams_t *params ) {
  uint8_t index;
  Serial.print("The min connection interval : ");
  Serial.println(params->connectionParams->minConnectionInterval, HEX);
  Serial.print("The max connection interval : ");
  Serial.println(params->connectionParams->maxConnectionInterval, HEX);
  Serial.print("The slaveLatency : ");
  Serial.println(params->connectionParams->slaveLatency, HEX);
  Serial.print("The connectionSupervisionTimeout : ");
  Serial.println(params->connectionParams->connectionSupervisionTimeout, HEX);
}

void gattServerWriteCallBack(const GattWriteCallbackParams *Handler) {
  uint8_t index;

  Serial.print("Handler->connHandle : ");
  Serial.println(Handler->connHandle, HEX);
  Serial.print("Handler->handle : ");
  Serial.println(Handler->handle, HEX);
  Serial.print("Handler->writeOp : ");
  Serial.println(Handler->writeOp, HEX);
  Serial.print("Handler->offset : ");
  Serial.println(Handler->offset, HEX);
  Serial.print("Handler->len : ");
  Serial.println(Handler->len, HEX);
  for (index = 0; index < Handler->len; index++) {
    Serial.print(Handler->data[index], HEX);
  }
  Serial.println(" ");

  uint8_t buf[TXRX_BUF_LEN];
  uint16_t bytesRead;

  Serial.println("Write Handle : ");
  // Check the attribute belong to which characteristic
  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
    // Read the value of characteristic
    ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
    for (index = 0; index < bytesRead; index++) {
      Serial.print(buf[index], HEX);
    }
    Serial.println(" ");
  }

  if (Handler->data[0] == 0) {
    digitalWrite(D13 , HIGH);
  }
  else {
    digitalWrite(D13, LOW);
  }
}

bool datasent = true; 
void gattServerSentCallBack(unsigned int count)
{
  Serial.print(millis());
  Serial.print(" The count : ");
  Serial.println(count, DEC);
  //value++;
  // if true, saved to attribute, no notification or indication is generated.
  //ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (uint8_t *)&value, 2, true);
  // if false or ignore, notification or indication is generated if permit.
  //ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (uint8_t *)&value, 2);
  //ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buff, sizeof(buff));
  if (!datasent) {
    datasent = true;
  }
}


void task_handle(void) {
  static uint8_t buff[15] = {0};
  Serial.println("Task handle ");
  if(buff[0] == 0){
    //write the first values
    buff[0]++;
  }
  else if(buff[0] == 1){
    //write to second set
    buff[0]++;
  }
  else{
    buff[0] = 0;
    Serial.println("Overflow error, reintialized buff");
    datasent=true;
  }
  if ((buff[0] > 0)&& datasent) {
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buff, sizeof(buff));
    buff[0]=0;
    datasent= false;
  }
  //value++;
  // if true, saved to attribute, no notification or indication is generated.
  //ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (uint8_t *)&value, 2, true);
  // if false or ignore, notification or indication is generated if permit.
  //ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (uint8_t *)&value, 2);

}

void setAdvertisement(void) {
  // Advertisement required element
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  // Add short name to advertisement
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME, (const uint8_t *)"TXRX", 4);
  // Add complete 128bit_uuid to advertisement
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (const uint8_t *)service1_uuid_rev, sizeof(service1_uuid_rev));
  // Add complete device name to scan response data
  ble.accumulateScanResponse(GapAdvertisingData::COMPLETE_LOCAL_NAME, (const uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME) - 1);
}

void setup() {
    
    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
//        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println("Newly Coded");

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
//    pinMode(INTERRUPT_PIN, INPUT);

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
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
    
  // put your setup code here, to run once
  Serial.println("Start ");
  pinMode(D13, OUTPUT);
  digitalWrite(D13, HIGH);

  // Init timer task
 // ticker1s.attach(task_handle, 0.03);
  // Init ble
  ble.init();
  ble.onConnection(connectionCallBack);
  ble.onDisconnection(disconnectionCallBack);
  ble.onDataWritten(gattServerWriteCallBack);
//  ble.onDataSent(gattServerSentCallBack);
  // set advertisement
  setAdvertisement();
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  deviceInfo = new DeviceInformationService(ble, "ARM", "Model1", "SN1", "hw-rev1", "fw-rev1", "soft-rev1");
  ble.addService(service1);
  // set device name
  ble.setDeviceName((const uint8_t *)DEVICE_NAME);
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // ger BLE stack version
  Serial.print("BLE stack verison is : ");
  Serial.println(ble.getVersion());
  // start advertising
  ble.startAdvertising();
  Serial.println("start advertising ");

}

void loop() {

    if(vibrateOn && (millis() > vibrateTime+100)){
            digitalWrite(7, LOW);
            Serial.println("off");
            vibrateOn = false;
    }
    // if programming failed, don't try to do anything
   if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
//    while (!mpuInterrupt && fifoCount < packetSize) {
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
//    }

    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpu.getFIFOCount() >= packetSize) {
        // wait for correct available data length, should be a VERY short wait
      //  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        int xx = mpu.getFIFOCount();
        int yy = millis();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        //mpu.getFIFOBytes(fifoBuffer, packetSize);
        //mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        //fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        //mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        //mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        // initialize values
        angNew = ypr[1]*180/M_PI;
        prevTime = currTime;
        currTime = millis();
        accWorldX = aaWorld.x;
        
        if(millis() < 40000) {       //it takes about 40 seconds for the values to callibrate themselves. 
          Serial.print(millis());
          Serial.print(", ");
          /*
          Serial.print(yy);
          Serial.print(", ");
          Serial.print(xx);
          Serial.print(", ");
          Serial.print(mpu.getFIFOCount());
          Serial.print(", ");
          */
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
        //accWorldX = movingAvgFilter(prevAx, accWorldX);
        //prevAx = accWorldX; //need two values for filter.

        Serial.print(xx);
        Serial.print(", ");
        Serial.print(angNew);
        Serial.print(", ");
        Serial.println(accWorldX);

        updateAngArr(angNew);
                
        //if user standing still, break main loop.
        if(isStill(accWorldX)){
          return; 
        }
        
        if(!isStep()) {
          //memset(freezeCount,0,sizeof(freezeCount)); //Set both values back to 0.

          return;
        } else {
          maxAngle = angleArr[midIndex-1];                                // feature 1
          strideFreq = 1.0 / ((millis() - strideStartTime) / 1000); // feature2



          
          Serial.print(maxAngle);
          Serial.print(", ");
          Serial.println(strideFreq);
          strideStartTime = millis();
          //Detect if it's freezing using SVM. 
          
          //COMMENT 
          if(-(w[0]*maxAngle + b)/w[1] < strideFreq){
            //If you get "freezing" twice in a row, it's freezing. 
            freezeCount[1] = freezeCount[0];
            freezeCount[0] = 1;
          }
          else{
            freezeCount[0] = 0;
            freezeCount[1] = 0; 
          }
          
          if(freezeCount[1] == 1){ //2nd value of freezecount would only be 1 if you freeze twice in a row. (Or else it gets reset)
              Serial.println("Freezing!");
//              digitalWrite(7, HIGH);
//              vibrateOn = true;
//              vibrateTime = millis();
          } 
           
  static uint8_t buff[15] = {0};
  Serial.print(millis());
  Serial.println(" Task handle ");
  
  if(buff[0] == 0){
    int16_t v = maxAngle * 10.0;
    buff[1] = v >> 8;  buff[2] = v & 0xff;
    v = strideFreq * 100.0;
    buff[3] = v >> 8;  buff[4] = v & 0xff;
    buff[0]++;
  }
  else if(buff[0] == 1){
    int16_t v = maxAngle * 10.0;
    buff[5] = v >> 8;  buff[6] = v & 0xff;
    v = strideFreq * 100.0;
    buff[7] = v >> 8;  buff[8] = v & 0xff;
    buff[0]++;
  }
  else{
    buff[0] = 0;
    Serial.print(millis());
    Serial.println(" Overflow error, reintialized buff");
    datasent=true;
  }
  buff[14] = freezeCount[1] == 1;

  if (buff[0] > 0) {
    bool queued = (ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buff, sizeof(buff))==BLE_ERROR_NONE);
    Serial.print(buff[0]);
    Serial.println(queued);
    if (queued) {
      buff[0] = 0;
    }
  }

          //COMMENT

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
  if(abs(accWorldX) <  200) {
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
 */
boolean isStep() {
    if(angleArr[midIndex-1]-angleArr[0] < 0.5){
       return false;
    }
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
