
#define L1PinLeft 2
#define dcEnablePin1 3
#define L2PinLeft 4

#define L1PinRight 5
#define dcEnablePin2 6
#define L2PinRight 7

#define FORWARD 1
#define BACKWARD 0

int incomingByte = 0;   // for incoming serial data
int leftMotorDirection,rightMotorDirection;

#define STANDARD_SPEED 120
#define HIGH_SPEED 110
#define SLOW_SPEED 50

unsigned long timerA;

//IMU REQUIREMENTS
#include "quaternionFilters.h"
#include "MPU9250.h"
MPU9250 myIMU;
#define Z_AXIS_GYRO_SENSITIVITY_CONSTANT 70

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

int degrees1 = 0;

void setup() {

  setupIMU();

  pinMode(L1PinLeft,OUTPUT);
  pinMode(dcEnablePin1,OUTPUT);
  pinMode(L2PinLeft,OUTPUT);

  pinMode(L1PinRight,OUTPUT);
  pinMode(dcEnablePin2,OUTPUT);
  pinMode(L2PinRight,OUTPUT);
}



void loop() {
  // put your main code here, to run repeatedly:
  //readSerialMotors();
  readDegreeTurn();
 // IMULoopUpdate();
 // IMUGyroUpdate();
}

void setupIMU(){
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed;
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
   myIMU.initMPU9250();
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
}

void IMUGyroUpdate(){
          Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");
}

void IMULoopUpdate(){
    // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
}

void readDegreeTurn(){
//Read input bytes until "." entered then execute left turn by entered degrees
 
 if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    if(incomingByte == '.'){
         stopMotors();
         prepMotors();
         turnLeft();
         turnDegrees(degrees1);
         stopMotors();
         degrees1 = 0;
    }
    else{
      Serial.println(incomingByte);
      degrees1 = degrees1 * 10 + incomingByte-int('0');
      Serial.print("DEGREESONE: ");
      Serial.println(degrees1);
    } 
 }
}

void turnDegrees(int degrees1){
//Watches IMU reading and return after turning by x degrees
  float z = 0;
  Serial.println(abs(z));
  float current,prev = 0;
  while(abs(z) < degrees1) { 
      IMULoopUpdate();
      z = z + myIMU.gz/float(70);
     Serial.println(z);
     delay(10);
  }
  Serial.println(z);
}

void readSerialMotors(){
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    switch(incomingByte){
      case '1':
        setLeftMotorForward(STANDARD_SPEED);
        setRightMotorForward(STANDARD_SPEED);
        delay(500);
        setLeftMotorForward(SLOW_SPEED);
        setRightMotorForward(SLOW_SPEED);
        break;
      case '2':
        setLeftMotorBackward(STANDARD_SPEED);
        setRightMotorForward(STANDARD_SPEED);
        break;
     case '3':
        setLeftMotorForward(STANDARD_SPEED);
        setRightMotorBackward(STANDARD_SPEED);
        break;
     case '4':
        setLeftMotorBackward(STANDARD_SPEED);
        setRightMotorBackward(STANDARD_SPEED);
        break;
     case '5':
        stopMotors();
        break;
     case '6':
//        reverseDirection();
        break;
     case '7':
         stopMotors();
         prepMotors();
         turnLeft();
         waitNinetyDegrees();
         stopMotors();
         break;
     case '8':
        stopMotors();
         prepMotors();
         turnRight();
         waitNinetyDegrees();
         stopMotors();
        break;
     case '9':
        setLeftMotorForward(SLOW_SPEED);
        setRightMotorForward(SLOW_SPEED);
        break;
    } 
  }
  
}

void waitNinetyDegrees(){
  float z = 0;
  Serial.println(abs(z));
  float current,prev = 0;
  while(abs(z) < 90) { 
      IMULoopUpdate();
      z = z + myIMU.gz/float(Z_AXIS_GYRO_SENSITIVITY_CONSTANT);
     Serial.println(z);
     delay(10);
  }
  Serial.println(z);
}

void prepMotors(){
    setLeftMotorBackward(STANDARD_SPEED);
    setRightMotorBackward(STANDARD_SPEED);  
    delay(500);
}

void waitTurn(){
    unsigned long currentTime = millis();
   while((currentTime - timerA) < 500){
    currentTime = millis();
    }
   Serial.println("Hello");
   stopMotors();
   
}
void turnRight(){
  setLeftMotorForward(STANDARD_SPEED);
  setRightMotorBackward(STANDARD_SPEED);
}

void turnLeft(){
   setLeftMotorBackward(STANDARD_SPEED);
   setRightMotorForward(STANDARD_SPEED);
}

void setLeftMotorForward(int speed){
  digitalWrite(L1PinLeft,LOW);
  digitalWrite(L2PinLeft,HIGH);
  analogWrite(dcEnablePin1,speed);
  leftMotorDirection=FORWARD;
}

void setRightMotorForward(int speed){
  digitalWrite(L1PinRight,LOW);
  digitalWrite(L2PinRight,HIGH);
  analogWrite(dcEnablePin2,speed);
  rightMotorDirection = FORWARD;
}

void setLeftMotorBackward(int speed){
  digitalWrite(L1PinLeft,HIGH);
  digitalWrite(L2PinLeft,LOW);
  analogWrite(dcEnablePin1,speed);
  leftMotorDirection=BACKWARD;
}

void setRightMotorBackward(int speed){
  digitalWrite(L1PinRight,HIGH);
  digitalWrite(L2PinRight,LOW);
  analogWrite(dcEnablePin2,speed);
  leftMotorDirection=BACKWARD;
}

//void reverseDirection(){
//  if(leftMotorDirection==FORWARD){
//    setLeftMotorBackward();
//  }
//  else if(leftMotorDirection==BACKWARD)
//    setLeftMotorForward();
//  if(rightMotorDirection==FORWARD){
//    setRightMotorBackward();
//  }
//  else if(rightMotorDirection==BACKWARD){
//    setRightMotorForward();
//  }
//}

void stopMotors(){
  analogWrite(dcEnablePin1,0);
  analogWrite(dcEnablePin2,0);
}


