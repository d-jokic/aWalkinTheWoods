#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

int blinkPin = 13;
int fadePin = 5;
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int amplitude = 0; //amplitude measured by gyro
#define button 7 //button to start
int rawY;
int PixelX;
int reading;
int scaledY;
int i = 0;
int WindowsAppWidth = 360;
int WindowsAppHeight = 360;
int terrainCoord;
int ypos;
int xspeed;
long Time = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void buttonWait(){
  while (digitalRead(button) == 0)
  delay(100);
  while(digitalRead(button) == 1)
  delay(100);
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
      //new stuff
      pinMode(blinkPin, OUTPUT);
      pinMode(fadePin,OUTPUT);
      interruptSetup();


      
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
          Wire.begin();
          TWBR = 24;
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
          Fastwire::setup(400, true);
      #endif
      Serial.begin(9600);
      while (!Serial); 
      mpu.initialize();

      //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      devStatus = mpu.dmpInitialize();
      buttonWait();

      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); 

      if (devStatus == 0) {
          //Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);
          //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
          attachInterrupt(0, dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();

          //Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;
          packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
//          Serial.print(F("DMP Initialization failed (code "));
//          Serial.print(devStatus);
//          Serial.println(F(")"));
      }   
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int gyroFxn(){
    //if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) 

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
            amplitude = ypr[2] * 180/M_PI;
            
            if (-10 < amplitude && amplitude <= 10){
              amplitude = 0;
            }
              for (int i = 1; i<= 8; i++){
               if(10*i < amplitude && amplitude <= 10*(i+1)){
                 amplitude = i;
                 }
                 else if (-10*i >= amplitude && amplitude > -10*(i+1)){
                   amplitude = -(i);
                 }
              }
              //Serial.println(amplitude);
              return amplitude;
        #endif
    }
}
  
int heartRateFxn(){
    int coord;
    reading = analogRead(A0);
    rawY = (1023 - reading) - 212;
    scaledY = map(reading, 0, 1023, 0, WindowsAppHeight);
    coord = scaledY;
    //i++;
    return coord;
}

void loop() { 
  for (int i = 0; i < 1199; i++) {
    gyroFxn();
    terrainCoord = heartRateFxn();
    terrainCoord *= amplitude;
    terrainCoord = map(terrainCoord, -2000, 2000, 0, WindowsAppHeight);
    Serial.println(heartRateFxn());
  }
  buttonWait();
  Time = millis();
  while(true) {
    gyroFxn();
    ypos = map(amplitude, -8, 8, -3, 3);
    xspeed = map(BPM, 60, 90, 0, 2);
    if (millis() - Time >= 50)
    {
      Time = millis();
      Serial.print(ypos);
      Serial.print(",");
      Serial.println(xspeed);
    }
  }
}
