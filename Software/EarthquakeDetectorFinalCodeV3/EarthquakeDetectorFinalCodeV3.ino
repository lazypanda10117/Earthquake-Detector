#include <SD.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "Wire.h"
#include <RTClib.h>
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"

RTC_DS3231 rtc;
MPU6050 mpu;

#define INTERRUPT_PIN 2

const int buzzerPin = 5;
const int orangeLedPin = 6;
const int greenLedPin = 7;
const int redLedPin = 8;
const int yellowLedPin = 9;

// MPU control/status vars
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
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
float xyzAccel[3];    
const int accelG = 16384;
//own variables
float prevYpr[3];
float offsetYpr[3]; //initial positions
boolean stable = false;
boolean stableN = false;

double yawRaw;
double pitchRaw;
double rollRaw;

double axRaw;
double ayRaw;
double azRaw;

int arrayCount = 0;
const int recurseLength = 12;
double timeDelay = 0.08;
int timeCounter = 0;
int timeStackToUpload = 3;

long ti; //unix time
String tS; //string unix time
String sdTimeName = "";

double gyroAlarmThreshold = 2.5;
double accelAlarmThreshold1 = 3;
double accelAlarmThreshold2 = 3;
double accelAlarmThreshold3 = 3.8;

double yawState[recurseLength];
double pitchState[recurseLength];
double rollState[recurseLength];

double dtYaw[recurseLength];
double dtPitch[recurseLength];
double dtRoll[recurseLength];

double axState[recurseLength];
double ayState[recurseLength];
double azState[recurseLength];
//end of Setting MPU variables

//equation that model gyroscope magnitude: signal = ke^(c(magnitude))
double expConst = 0.4642; //c
double scaleConst = 1.5578; //k

//equation that model acceleration magnitude: magnitude = a + b(ln(signal))
double baseAccelConst = 7.66395201; //a
double scaleAccelConst = 0.7781699; //b

double mag[3] = {0, 0, 0};
double magA[3] = {0, 0, 0};
double avgD[3] = {0, 0, 0};
double avgA[3] = {0, 0, 0};

File logFile;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
  Serial3.begin(9600);
  if (! rtc.begin()) {
    Serial.println("RTC Failed");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC Failed");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  DateTime now = rtc.now();
  sdTimeName = "E";
  String sm = String(now.month());
  if(now.month() < 10){
    sm = "0" + sm;
  }
  String sd = String(now.day());
  if(now.day() < 10){
    sd = "0" + sd;
  }
  sdTimeName += String(now.year()).substring(2,4);
  sdTimeName += sm;
  sdTimeName += sd;
  if (!SD.begin(4)) {
    Serial.println("SD error");
    return;
  }
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  Serial.println(F("Connecting"));
  Serial.println(mpu.testConnection() ? F("Success") : F("Failed"));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("Start"));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.println("MPU Failed");
  }
  for (int i = 0; i < 3; i++) {
    prevYpr[i] = 9999;
  }
}

void loop() {
    if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
    }else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      if (stable) {      
        yawRaw = ypr[0] * 180 / M_PI;
        pitchRaw = ypr[1] * 180 / M_PI;
        rollRaw = ypr[2] * 180 / M_PI;
        axRaw = aaReal.x-430;
        ayRaw = aaReal.y;
        azRaw = aaReal.z;
        if (arrayCount < recurseLength) {
          set6AxisState(arrayCount, yawRaw, pitchRaw, rollRaw, axRaw, ayRaw, azRaw);
          if(arrayCount != 0){
            setInitialDerivative(arrayCount - 1);
          }
          arrayCount++;  
        }else{
          for (int i = 0; i < recurseLength-1; i++) {
            set6AxisState(i, yawState[i+1], pitchState[i+1], rollState[i+1], axState[i+1], ayState[i+1], azState[i+1]);
            if(arrayCount>recurseLength){
              set3AxisDerivative(i, dtYaw[i + 1], dtPitch[i + 1], dtRoll[i + 1]);
            }
          }
          set6AxisState(recurseLength - 1, yawRaw, pitchRaw, rollRaw, axRaw, ayRaw, azRaw);
          setAfterDerivative();
          if(arrayCount>recurseLength){
            analyzeGyroSignals();
            analyzeAccelSignals();
            String ts = getPreciseTime(ti);
            transferData(ts, yawRaw, pitchRaw, rollRaw, axRaw, ayRaw, azRaw);
            if(!stableN){
              if(magA[1] < 2.5 && magA[2] < 3.5){
                stableN = true;
              }
            }else{
              alarm();
              statusStable();
            }
            statusOK();
            timeCounter++;
            delay(timeDelay * 1000);
          }else{
            arrayCount++;  
          }
        }
        
      } else {
        if (abs(prevYpr[0] - (ypr[0] * 180 / M_PI)) <= 0.01 && abs(prevYpr[1] - (ypr[1] * 180 / M_PI)) <= 0.01 && abs(prevYpr[2] - (ypr[2] * 180 / M_PI)) <= 0.01) {
          delay(3000);
          if (abs(prevYpr[0] - (ypr[0] * 180 / M_PI)) <= 0.01 && abs(prevYpr[1] - (ypr[1] * 180 / M_PI)) <= 0.01 && abs(prevYpr[2] - (ypr[2] * 180 / M_PI)) <= 0.01) {
              Serial.println(ayRaw);
              stable = true;
              for (int i = 0; i < 3; i++) {
                offsetYpr[i] = ypr[i] * 180 / M_PI;
              }
              while(aaReal.y > 60 && aaReal.z > 60){
                delay(50);
              }
          }
        } else {       
          prevYpr[0] = ypr[0] * 180 / M_PI;
          prevYpr[1] = ypr[1] * 180 / M_PI;
          prevYpr[2] = ypr[2] * 180 / M_PI;         
          delay(50);
        }
      }
    }
}

void statusStable(){
  digitalWrite(orangeLedPin,HIGH);
}

void analyzeGyroSignals() {
  //average last recureseLength d(angle)/dt to see which range it is in
  double tempVal1 = 0;
  avgD[0] = averageDerivative(dtYaw);
  avgD[1] = averageDerivative(dtPitch);
  avgD[2] = averageDerivative(dtRoll);
  for (int i = 0; i < 3; i++) {
    tempVal1 = gyroEquation(avgD[i]);
    mag[i] = tempVal1;
  }
}

double gyroEquation(double d){
  return (log((d/scaleConst)+1) / expConst);
}

void analyzeAccelSignals(){
  double tempVal2 = 0;
  avgA[0] = averageDerivative(axState);
  avgA[1] = averageDerivative(ayState);
  avgA[2] = averageDerivative(azState);  
  for (int i = 0; i < 3; i++) {
    tempVal2 = accelEquation(avgA[i]);
    magA[i] = tempVal2;
  }
}

double accelEquation(double d){
  double tD = d/(double)accelG;
  return baseAccelConst + (scaleAccelConst*log(tD));
}

void setInitialDerivative(int i) {
  dtYaw[i] = yawState[i + 1] - yawState[i];
  dtPitch[i] = pitchState[i + 1] - pitchState[i];
  dtRoll[i] = rollState[i + 1] - rollState[i];
}

void setAfterDerivative() {
  dtYaw[recurseLength - 1] = yawState[recurseLength - 1] - yawState[recurseLength - 2];
  dtPitch[recurseLength - 1] = pitchState[recurseLength - 1] - pitchState[recurseLength - 2];
  dtRoll[recurseLength - 1] = rollState[recurseLength - 1] - rollState[recurseLength - 2];
}

void set6AxisState(int id, double y, double p, double r, double ax, double ay, double az) {
  yawState[id] = y;
  pitchState[id] = p;
  rollState[id] = r;
  axState[id] = ax;
  ayState[id] = ay;
  azState[id] = az;
}

void set3AxisDerivative(int id, double dy, double dp, double dr) {
  dtYaw[id] = dy;
  dtPitch[id] = dp;
  dtRoll[id] = dr;
}

double averageDerivative(double d[]) {
  double total = 0;
  for (int i = 0; i < recurseLength; i++) {
    total += abs(d[i]);
  }
  return total / (double)recurseLength;
}

void transferData(String preciseTime, double y, double p, double r, double ax, double ay, double az) {
  String tS = "{" + preciseTime + "," + String(y,2) + "," + String(p,2) + "," + String(r,2) + "," + String(mag[0],2) + "," + String(mag[1],2) + "," + String(mag[2],2) + "," + String(ax,2) + "," + String(ay,2) + "," + String(az,2) + "," + String(magA[0],2) + "," + String(magA[1],2) + "," + String(magA[2],2)+"}";
  String tP =  "{" + preciseTime + "," /*+ String(mag[0],2) + "," + String(mag[1],2) + "," + String(mag[2],2) +  ","*/ + String(magA[0],2) + "," + String(magA[1],2) + "," + String(magA[2],2)+"}";

  if(timeCounter%((int)((timeStackToUpload*100)/(int)(100*timeDelay))) == 0){
    DateTime now = rtc.now();
    ti = now.unixtime();
    timeCounter=0;
  }
  Serial.println(tP);
  Serial3.println(tP);
  putToSDCard(tS);
}

void statusOK() {
  digitalWrite(greenLedPin, HIGH);
}

boolean gyroAlarmCheck() {
  if (mag[0] >= gyroAlarmThreshold || mag[1] >= gyroAlarmThreshold || mag[2] >= gyroAlarmThreshold) {
    digitalWrite(yellowLedPin, HIGH);
    return true;
  }else{
    digitalWrite(yellowLedPin, LOW);
    return false;
  }

}

boolean accelAlarmCheck(){
  if (magA[0] >= accelAlarmThreshold1 || magA[1] >= accelAlarmThreshold2 || magA[2] >= accelAlarmThreshold3){
    digitalWrite(redLedPin, HIGH);
    return true;
  }else{
    digitalWrite(redLedPin, LOW);
    return false;
  }
}

void alarm(){
  boolean a = accelAlarmCheck();
  boolean b = gyroAlarmCheck();
  if(a || b){
    digitalWrite(buzzerPin,HIGH);
  }else{
    digitalWrite(buzzerPin,LOW);
  }
}

String getPreciseTime(long t){
  String tempS = String(t);
  String condition0 = "";
  char cA1[8];
  char cA2[4];
  (tempS.substring(3,10)).toCharArray(cA1,8);
  (tempS.substring(0,3)).toCharArray(cA2,4);
  long tempL1 = atol(cA1);
  int tempL2 = atol(cA2);
  if(tempS.charAt(3) == '0'){
    condition0 = "0";
  }
  long temp = (tempL1*100)+(long)(1.51*(timeDelay*100*timeCounter));
  String tempST = String(temp);
  if(tempST.charAt(0) == '1'){
    tempL2 += 1;
    tempST = tempST.substring(1,10);
  }
  int lenOfTST = tempST.length();
  String finalS = String(tempL2) + condition0 + tempST.substring(0,lenOfTST-2) + "." + tempST.substring(lenOfTST-2,lenOfTST);
  return finalS;
}

void putToSDCard(String s){
  String fileName = sdTimeName + ".txt";
  logFile = SD.open(fileName, FILE_WRITE);
  if (logFile) {
    logFile.println(s);
    logFile.close();
  }else{
    Serial.println("error opening logfile.txt");
  }
}
