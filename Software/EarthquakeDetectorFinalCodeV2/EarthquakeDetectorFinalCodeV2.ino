void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}#include "I2Cdev.h"
#include "Wire.h"
#include <RTClib.h>
#include "MPU6050_6Axis_MotionApps20.h"

RTC_DS3231 rtc;
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

const int fVibPin = 3; //fast vibration sensor input
const int buzzerPin = 5;
const int orangeLedPin = 6;
const int greenLedPin = 7;
const int yellowLedPin = 8;
const int redLedPin = 9;

boolean fVConnect = false;

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
      
//own variables
float prevYpr[3];
float offsetYpr[3]; //initial positions
boolean stable = false;

double yawRaw;
double pitchRaw;
double rollRaw;

double axRaw;
double ayRaw;
double azRaw;

int arrayCount = 0;

const int recurseLength = 6;
double timeDelay = 0.1;
int timeCounter = 0;
int timeStackToUpload = 3;

long ti; //unix time

int gyroAlarmThreshold = 2.5;
int accelAlarmThreshold = 2.5;

double yawState[recurseLength];
double pitchState[recurseLength];
double rollState[recurseLength];

double dtYaw[recurseLength];
double dtPitch[recurseLength];
double dtRoll[recurseLength];

double axState[recurseLength];
double ayState[recurseLength];
double azState[recurseLength];

double dtAX[recurseLength];
double dtAY[recurseLength];
double dtAZ[recurseLength];

//end of Setting MPU variables

//equation that model gyroscope magnitude: signal = ke^(c(magnitude))
double expConst = 0.4642; //c
double scaleConst = 1.5578; //k

//equation that model acceleration magnitude: signal = ke^(c(magnitude))
double expAccelConst = 0.4642; //c
double scaleAccelConst = 1.5578; //k

double mag[3] = {0, 0, 0};
double magA[3] = {0, 0, 0};
double avgAD[3] = {0,0,0};
double avgD[3] = {0, 0, 0};

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mySerial.begin(9600);
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initialize device
  if (! rtc.begin()) {
    Serial.println("RTC Failed");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC Failed");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  if (!SD.begin(4)) {
    Serial.println("SD Failed");
    return;
  }
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  pinMode(fVibPin, INPUT);//sets pin as fast vibration sensor input
  pinMode(buzzerPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  // verify connection
  //Serial.println(F("Connecting"));
  //Serial.println(mpu.testConnection() ? F("Success") : F("Failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    //Serial.println(F("Setting interrupt"));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("Start"));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
    Serial.println("MPU Failed");
  }

  for (int i = 0; i < 3; i++) {
    prevYpr[i] = 9999;
  }
}

void loop() {
    if (!dmpReady) return;
    /*while (!mpuInterrupt && fifoCount < packetSize) {
      }*/
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      //Serial.println(F("Overflow"));
      //delay(100);
    } else if (mpuIntStatus & 0x02) {
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
        
        axRaw = aaReal.x;
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
              set6AxisDerivative(i, dtYaw[i + 1], dtPitch[i + 1], dtRoll[i + 1], dtAX[i+1], dtAY[i+1], dtAZ[i+1]);
            }
          }
          set6AxisState(recurseLength - 1, yawRaw, pitchRaw, rollRaw, axRaw, ayRaw, azRaw);
          setAfterDerivative();
          if(arrayCount>recurseLength){
            analyzeGyroSignals();
            analyzeAccelSignals();
            transferData(ts, yawRaw, pitchRaw, rollRaw, axRaw, ayRaw, azRaw);
            hardwareCheck();
            alarm();
            statusOK();
            timeCounter++;
            delay(timeDelay * 1000);
          }else{
            arrayCount++;  
          }
        }
        
      } else {
        if (abs(prevYpr[0] - (ypr[0] * 180 / M_PI)) <= 0.01 && abs(prevYpr[1] - (ypr[1] * 180 / M_PI)) <= 0.01 && abs(prevYpr[2] - (ypr[2] * 180 / M_PI)) <= 0.01) {
          delay(3000); // wait 3 seconds to see if the difference stabilzation is actually true
          if (abs(prevYpr[0] - (ypr[0] * 180 / M_PI)) <= 0.01 && abs(prevYpr[1] - (ypr[1] * 180 / M_PI)) <= 0.01 && abs(prevYpr[2] - (ypr[2] * 180 / M_PI)) <= 0.01) {
            stable = true;
            for (int i = 0; i < 3; i++) {
              offsetYpr[i] = ypr[i] * 180 / M_PI;
            }
          }
        } else {       
          prevYpr[0] = ypr[0] * 180 / M_PI;
          prevYpr[1] = ypr[1] * 180 / M_PI;
          prevYpr[2] = ypr[2] * 180 / M_PI;         
          //Serial.println("Unstable");
          delay(50);
        }
      }
    }
}

void analyzeGyroSignals() {
  //average last recureseLength d(angle)/dt to see which range it is in
  double tempVal1 = 0;
  avgD[0] = averageDerivative(dtYaw);
  avgD[1] = averageDerivative(dtPitch);
  avgD[2] = averageDerivative(dtRoll);
  for (int i = 0; i < 3; i++) {
    tempVal1 = (log((avgD[i]/scaleConst)+1) / expConst);
    mag[i] = tempVal1;
  }
}

void analyzeAccelSignals(){
  double tempVal2 = 0;
  avgAD[0] = averageDerivative(dtAX);
  avgAD[1] = averageDerivative(dtAY);
  avgAD[2] = averageDerivative(dtAZ);  
  for (int i = 0; i < 3; i++) {
    tempVal2 = (log((avgAD[i]/scaleAccelConst)+1) / expAccelConst);
    magA[i] = tempVal2;
  }
}

void setInitialDerivative(int i) {
  dtYaw[i] = yawState[i + 1] - yawState[i];
  dtPitch[i] = pitchState[i + 1] - pitchState[i];
  dtRoll[i] = rollState[i + 1] - rollState[i];
  dtAX[i] = axState[i + 1] - axState[i];
  dtAY[i] = ayState[i + 1] - ayState[i];
  dtAZ[i] = azState[i + 1] - azState[i];
}

void setAfterDerivative() {
  dtYaw[recurseLength - 1] = yawState[recurseLength - 1] - yawState[recurseLength - 2];
  dtPitch[recurseLength - 1] = pitchState[recurseLength - 1] - pitchState[recurseLength - 2];
  dtRoll[recurseLength - 1] = rollState[recurseLength - 1] - rollState[recurseLength - 2];
  dtAX[recurseLength - 1] = axState[recurseLength - 1] - axState[recurseLength - 2];
  dtAY[recurseLength - 1] = ayState[recurseLength - 1] - ayState[recurseLength - 2];
  dtAZ[recurseLength - 1] = azState[recurseLength - 1] - azState[recurseLength - 2];
}

void set6AxisState(int id, double y, double p, double r, double ax, double ay, double az) {
  yawState[id] = y;
  pitchState[id] = p;
  rollState[id] = r;
  axState[id] = ax;
  ayState[id] = ay;
  azState[id] = az;
}

void set6AxisDerivative(int id, double dy, double dp, double dr, double dAX, double dAY, double dAZ) {
  dtYaw[id] = dy;
  dtPitch[id] = dp;
  dtRoll[id] = dr;
  dtAX[id] = dAX;
  dtAY[id] = dAY;
  dtAZ[id] = dAZ;
}

double averageDerivative(double d[]) {
  double total = 0;
  for (int i = 0; i < recurseLength; i++) {
    total += abs(d[i]);
  }
  return total / (double)recurseLength;
}

void transferData(String preciseTime, double y, double p, double r, double ax, double ay, double az) {
  String tS = "{" + preciseTime + "," + String(y,2) + "," + String(p,2) + "," + String(r,2) + "," + String(ax,2) + "," + String(ay,2) + "," + String(az,2) + "}";
  if(timeCounter%((int)((timeStackToUpload*100)/(int)(100*timeDelay))) == 0){
    DateTime now = rtc.now();
    ti = now.unixtime();
    timeCounter=0;
  }
  char tA[tS.length()];
  for(int i=0; i<tS.length(); i++){
    tA[i] = tS.charAt(i);
  }
  Serial.println(tS);
  putToSDCard(tS);
  //Serial.println(tS);
}

void hardwareCheck() {
  fVConnect = digitalRead(fVibPin);
  if(fVConnect){
    digitalWrite(yellowLedPin, HIGH);    
  }
}

void statusOK() {
  digitalWrite(greenLedPin, HIGH);
}

boolean gyroAlarmCheck() {
  if (mag[0] >= gyroAlarmThreshold || mag[1] >= gyroAlarmThreshold || mag[2] >= gyroAlarmThreshold) {
    digitalWrite(redLedPin, HIGH);
    return true;
  }else{
    digitalWrite(redLedPin, LOW);
    return false;
  }

}

boolean accelAlarmCheck(){
  if (magA[0] >= accelAlarmThreshold || magA[1] >= accelAlarmThreshold || magA[2] >= accelAlarmThreshold){
    digitalWrite(orangeLedPin, HIGH);
    return true;
  }else{
    digitalWrite(orangeLedPin, LOW);
    return false;
  }
}

void alarm(){
  if(fVConnect || accelAlarmCheck() || gyroAlarmCheck()){
    digitalWrite(buzzerPin,HIGH);
  }else{
    digitalWrite(buzzerPin,LOW);
  }
}

void putToSDCard(String s){
  logFile = SD.open("EarthquakeLogger.txt", FILE_WRITE);
  if (logFile) {
    logFile.println(s);
    logFile.close();
  } else {
    Serial.println("SD Error");
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
  long temp = (tempL1*100)+(long)(4.1*(timeDelay*100*timeCounter));
  String tempST = String(temp);
  if(tempST.charAt(0) == '1'){
    tempL2 += 1;
    tempST = tempST.substring(1,10);
  }
  int lenOfTST = tempST.length();
  String finalS = String(tempL2) + condition0 + tempST.substring(0,lenOfTST-2) + "." + tempST.substring(lenOfTST-2,lenOfTST);
  return finalS;
}

