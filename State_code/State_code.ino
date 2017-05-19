#include <MPU9250.h>
#include <quaternionFilters.h>

#include <MedianFilter.h>

enum states {CALIBRATING, MOVING, MOVING_TO_MOVING, APPROACHING_EDGE, TURNING, STOP,ORIENTING_TO_SEPARATOR, CROSSING_SEPARATOR, ORIENTING_AFTER_SEPARATOR,RETURN_TO_SEPARATOR, FINDING_EDGE, FINISHED};
volatile int state = 0;
unsigned long MOVING_TIMER;

/* Motor Pins */
#define L1PinLeft 12
#define dcEnablePin1 3 
#define L2PinLeft 4

#define L1PinRight 5
#define dcEnablePin2 6
#define L2PinRight 7


#define GRAVITY_COMPENSATION_LEFT 80
#define GRAVITY_COMPENSATION_RIGHT 60

#define STANDARD_SPEED 120
#define SLOW_SPEED 90
#define HIGH_SPEED 160
#define MAX_SPEED 255

#define TIME_PREP_MOTORS 0

#define FORWARD 1
#define BACKWARD 0 


volatile int motorDirection = FORWARD; //INITIAL DIRECTION

//UltraSonicPins
#define FORWARD_ULTRASONIC_SENSOR 18
#define BACKWARD_ULTRASONIC_SENSOR 19
#define LEFT_SIDE_ULTRASONIC_SENSOR 2
#define trigPin 51 // Trigger Pin

#define FORWARD_ULTRASONIC_DISTANCE 3
#define BACKWARD_ULTRASONIC_DISTANCE 3
#define LEFT_SIDE_ULTRA_DISTANCE 32

#define CROSSING_SEPARATOR_STOP_DISTANCE 60 
#define CROSSING_SEPARATOR_RETURN_DISTANCE 10

float duration, distance; // Duration used to calculate distance
int maximumRange = 400; // Maximum range of sensor
int minimumRange = 2; // Minimum range of sensor
MedianFilter forwardSensorMedian(5,INFINITY);
MedianFilter backwardSensorMedian(5,INFINITY);
MedianFilter leftSensorMedian(5,INFINITY);

//Debuggin Delays + timer + state 
unsigned long timerA; 
int ledState;
#define MOVING_BLINK_DELAY 500
#define TURNING_BLINK_DELAY 50
#define APPROACHING_BLINK_DELAY 100
#define ORIENTING_BLINK_DELAY 100
#define CROSSING_BLINK_DELAY 5000
#define LED_PIN 13

#define SWITCH_PIN 11

int currentDirection = 1;
//IMU REQeUIREMENTS
#include "quaternionFilters.h"
#include "MPU9250.h"
MPU9250 myIMU;
#define Z_AXIS_GYRO_SENSITIVITY_CONSTANT 60

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
//int myLed  = 13;  // Set up pin 13 led for toggling

int degrees1 = 0;

bool firstPass=true;
bool passDone = false;
bool crossNext = false;

#define DEGREE_TURN 3
#define DEGREE_ORIENT 45

/*
 * Fan Ciode
 */
//#
#include <Servo.h>
#define FAN_PIN 8
#define FAN_PIN2 9
Servo firstESC, secondESC;

#define FAN_START_VALUE 700
#define FAN_MAX_VALUE 2000

int value = 0; // set values you need to zero

bool separator_crossed = false;


void setup() {
  // put your setup code here, to run once:
  
  setupSensors();
  setupIMU();
  setupFans();
}

void setupFans(){
   firstESC.attach(FAN_PIN);    // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(FAN_PIN2);
}

void setupSensors(){
//  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
 
// pinMode(FORWARD_ULTRASONIC_SENSOR, INPUT);
// pinMode(BACKWARD_ULTRASONIC_SENSOR, INPUT);
// pinMode(LEFT_SIDE_ULTRASONIC_SENSOR, INPUT);

  attachInterrupt(digitalPinToInterrupt(FORWARD_ULTRASONIC_SENSOR),forwardSensorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACKWARD_ULTRASONIC_SENSOR),backwardSensorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_SIDE_ULTRASONIC_SENSOR),leftSensorInterrupt, CHANGE);
  //pinMode(LEFT_SIDE_ULTRASONIC_SENSOR, INPUT);
 
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(SWITCH_PIN,INPUT);
  digitalWrite(SWITCH_PIN,HIGH);
  pinMode(L1PinLeft,OUTPUT);
  pinMode(dcEnablePin1,OUTPUT);
  pinMode(L2PinLeft,OUTPUT);

  pinMode(L1PinRight,OUTPUT);
  pinMode(dcEnablePin2,OUTPUT);
  pinMode(L2PinRight,OUTPUT);


}

unsigned long sensorTimeForward;
void forwardSensorInterrupt(){
  runFans();
  int sensorState = digitalRead(FORWARD_ULTRASONIC_SENSOR);
  if (sensorState == HIGH){
     sensorTimeForward = micros();
  }
  else{
    sensorTimeForward = micros()- sensorTimeForward;
    float calculatedDistance =  interruptSensorValue(sensorTimeForward);
    calculatedDistance = forwardSensorMedian.in(calculatedDistance);
     if(state == MOVING && calculatedDistance<FORWARD_ULTRASONIC_DISTANCE && !firstPass && !passDone && crossNext){
      passDone=true;
      state = ORIENTING_TO_SEPARATOR;
    }
    else if(state == MOVING && calculatedDistance<FORWARD_ULTRASONIC_DISTANCE){
      state = APPROACHING_EDGE;
    }
    else if(state == FINDING_EDGE && calculatedDistance<FORWARD_ULTRASONIC_DISTANCE ){
       motorDirection=!motorDirection;
       state = MOVING_TO_MOVING;
    }
    else if (state == RETURN_TO_SEPARATOR && motorDirection == FORWARD && calculatedDistance < CROSSING_SEPARATOR_RETURN_DISTANCE){
      state=ORIENTING_AFTER_SEPARATOR;
    }
     else if (state == CROSSING_SEPARATOR && calculatedDistance < CROSSING_SEPARATOR_STOP_DISTANCE){
      state = ORIENTING_AFTER_SEPARATOR;
    }
  }
}

unsigned long sensorTimeBackward;
void backwardSensorInterrupt(){
  runFans();
  int sensorState = digitalRead(BACKWARD_ULTRASONIC_SENSOR);
  if (sensorState == HIGH){
     sensorTimeBackward = micros();
  }
  else{
    sensorTimeBackward = micros()-sensorTimeBackward;
    float calculatedDistance =  interruptSensorValue(sensorTimeBackward);
    calculatedDistance = backwardSensorMedian.in(calculatedDistance);
     if(state == MOVING && interruptSensorValue(sensorTimeBackward)<BACKWARD_ULTRASONIC_DISTANCE && !firstPass && !passDone && crossNext){  
      passDone = true;
      state = ORIENTING_TO_SEPARATOR;
    }
    else if(state == MOVING && interruptSensorValue(sensorTimeBackward)<BACKWARD_ULTRASONIC_DISTANCE){
      state = APPROACHING_EDGE;
    }
    else if(state == FINDING_EDGE && calculatedDistance<BACKWARD_ULTRASONIC_DISTANCE ){
       motorDirection=!motorDirection;
       state = MOVING_TO_MOVING;
    }
    else if (state == RETURN_TO_SEPARATOR && motorDirection==BACKWARD && calculatedDistance < CROSSING_SEPARATOR_RETURN_DISTANCE){
      state=ORIENTING_AFTER_SEPARATOR;
    }
     else if (state == CROSSING_SEPARATOR && calculatedDistance < CROSSING_SEPARATOR_STOP_DISTANCE){
      state = ORIENTING_AFTER_SEPARATOR;
    }
  }
}

unsigned long sensorTimeLeft;
void leftSensorInterrupt(){
  runFans();
  int sensorState = digitalRead(LEFT_SIDE_ULTRASONIC_SENSOR);
  if (sensorState == HIGH){
     sensorTimeLeft = micros();
  }
  else{
   sensorTimeLeft = micros()-sensorTimeLeft;
   float calculatedDistance =  interruptSensorValue(sensorTimeLeft);
   calculatedDistance = leftSensorMedian.in(calculatedDistance);
  if(state == MOVING && calculatedDistance < LEFT_SIDE_ULTRA_DISTANCE && firstPass){
       firstPass = false;
      // state=APPROACHING_EDGE;
       
    }
  }
}

float interruptSensorValue(unsigned long durationTime){
   durationTime =durationTime; //return median value after new sample processed

   //Calculate the distance (in cm) based on the speed of sound.
   distance = durationTime/58.2;
   
   if (distance >= maximumRange || distance <= minimumRange){
     /* Send a negative number to computer and Turn LED ON 
     to indicate "out of range" */
     int infinity = 10000;
     distance = infinity;
   }
   else {
     /* Send the distance to the computer using Serial protocol, and
     turn LED OFF to indicate successful reading. */
   }
   return distance;
}

void setupIMU(){
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed;
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);

   myIMU.initMPU9250();
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
}

void loop() {
  //Serial.println(states(state));
  if(state != STOP && state!=CALIBRATING){
  runFans();
 }
  checkSensorsForStateChange();
 // ledDisplayState();
  switch (state) {
    case CALIBRATING:
      stopFans();
      break;
    case MOVING:
      runFans();
      runMotors();
      break;
    case MOVING_TO_MOVING:
      prepMotors();
      break;
    case APPROACHING_EDGE:
      stopMotors();
      delay(400);
      break;
    case TURNING:
      turningProcedure();
      break;
    case STOP:
      stopMotors();
      stopFans();
      break;
    case ORIENTING_TO_SEPARATOR:
      orientationProcedure();
      break;
    case CROSSING_SEPARATOR:
      runMotorsReverseStandard();
      break;
    case RETURN_TO_SEPARATOR:
      runMotors();
      break;
    case ORIENTING_AFTER_SEPARATOR:
      turningProcedure();
      break;
    case FINDING_EDGE:
      runMotorsReverse();
      break;
    case FINISHED:
      shutdownSystem();
      break;  
  }
}

void prepMotors(){
  if(motorDirection == FORWARD){
    setLeftMotorForward(STANDARD_SPEED);
    setRightMotorForward(STANDARD_SPEED);  
    delay(500);
  }
  else{
    setLeftMotorBackward(STANDARD_SPEED);
    setRightMotorBackward(STANDARD_SPEED);  
     delay(500);
  }
}


void stopFans(){
  firstESC.writeMicroseconds(FAN_START_VALUE);
  secondESC.writeMicroseconds(FAN_START_VALUE);
}

void runMotors(){
  if(motorDirection == FORWARD){
    setLeftMotorForward(SLOW_SPEED);
    setRightMotorForward(SLOW_SPEED);
  }
  else if(motorDirection == BACKWARD){
    setLeftMotorBackward(SLOW_SPEED);
    setRightMotorBackward(SLOW_SPEED);
  }
}

void runMotorsReverse(){
  if(motorDirection == BACKWARD){
    setLeftMotorForward(SLOW_SPEED);
    setRightMotorForward(SLOW_SPEED);
  }
  else {
    setLeftMotorBackward(SLOW_SPEED);
    setRightMotorBackward(SLOW_SPEED);
  }
}

void runMotorsReverseStandard(){
  if(motorDirection == BACKWARD){
    setLeftMotorForward(STANDARD_SPEED);
    setRightMotorForward(STANDARD_SPEED);
  }
  else {
    setLeftMotorBackward(STANDARD_SPEED);
    setRightMotorBackward(STANDARD_SPEED);
  }
}

void runFans(){
  firstESC.writeMicroseconds(FAN_MAX_VALUE);
  secondESC.writeMicroseconds(FAN_MAX_VALUE);
 
}

void runMotorsSlow(){
  if(motorDirection == FORWARD){
    setLeftMotorForward(SLOW_SPEED);
    setRightMotorForward(SLOW_SPEED);
  }
  else if(motorDirection == BACKWARD){
    setLeftMotorBackward(SLOW_SPEED);
    setRightMotorBackward(SLOW_SPEED);
  }
}

void runMotorsFast(){
  if(motorDirection == FORWARD){
    setLeftMotorForward(HIGH_SPEED);
    setRightMotorForward(HIGH_SPEED);
  }
  else if(motorDirection == BACKWARD){
    setLeftMotorBackward(HIGH_SPEED);
    setRightMotorBackward(HIGH_SPEED);
  }
}

void turningProcedure(){
  if(motorDirection == FORWARD){
    turnRight(HIGH_SPEED);
  }
  else{
    turnLeft(HIGH_SPEED);
  }
}

void turningProcedureReverse(){
  if(motorDirection == BACKWARD){
    turnRight(HIGH_SPEED);
  }
  else{
    turnLeft(HIGH_SPEED);
  }
}

void orientationProcedure(){
  if(motorDirection == BACKWARD){
    turnRight(MAX_SPEED);
  } 
  else{
    turnLeft(MAX_SPEED);
  }
}

void orientationProcedureFlipped(){
  
  if(motorDirection == BACKWARD){
  turnRightFlipped(HIGH_SPEED);
  } 
  else{
    turnLeftFlipped(HIGH_SPEED);
  }
}

void orientationProcedurePostCross(){
  if(motorDirection == FORWARD){
    turnRightFlipped(HIGH_SPEED);
  } 
  else{
    turnLeftFlipped(HIGH_SPEED);
  }
}


void shutdownSystem(){

}

void checkSensorsForStateChange(){
 // This function contains the functions that monitors and applies states changes using the sensors as opposed
 //  to the main loop which performs the functions of each state.
 switch (state) {
    case CALIBRATING:
      checkGoSignal();
      break;
    case MOVING:
      checkForObstacle();
      break;
    case MOVING_TO_MOVING:
      timePrepMotors();
      break;
    case APPROACHING_EDGE:
      checkForObstacleStop();
      break;
    case TURNING:
      checkOrientationTurnCompleted();
      break;
    case ORIENTING_TO_SEPARATOR:
      checkOrientationProcedure();
      break;
    case CROSSING_SEPARATOR:
      isSeparatorCrossed();
      break;
    case RETURN_TO_SEPARATOR:
      checkReturnToSeparator();
      break;
    case ORIENTING_AFTER_SEPARATOR:
      checkOrientationProcedureAfter();
      break;
    case FINDING_EDGE:
      checkFindingEdge();
      break;
  }
   if (Serial.available() > 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if (incomingByte == '5'){
      state= STOP;
    }
   }
   if(checkSwitch() && state!= CALIBRATING){
      Serial.println("?");
      state= STOP;
   }
}
void checkReturnToSeparator(){
  sendTrig();
}


void checkForObstacle(){
    sendTrig();
}

void sendTrig(){
   digitalWrite(trigPin, LOW); 
   delayMicroseconds(2); 
  
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10); 
   
   digitalWrite(trigPin, LOW);
}

void checkForObstacleStop(){
   if(motorDirection == FORWARD){
        if(!firstPass){
          crossNext = true;
        }
        state=TURNING;
        timerA = millis();
   }
   else if(motorDirection == BACKWARD){
        if(!firstPass){
          crossNext = true;
        }
        state = TURNING;
        timerA = millis();
   }
}


void checkOrientationTurnCompleted(){
  float degrees_turned = turnDegrees(DEGREE_TURN);
  turningProcedureReverse();
  turnDegrees(degrees_turned-DEGREE_TURN);
  motorDirection=!motorDirection;
  state = MOVING_TO_MOVING;
}



void checkOrientationProcedure(){
  turningProcedure();
  turnDegrees(DEGREE_ORIENT);
  if(motorDirection == BACKWARD){
    setLeftMotorForward(HIGH_SPEED+25);
    setRightMotorForward(HIGH_SPEED+25);
  }
  else {
    setLeftMotorBackward(HIGH_SPEED+25);
    setRightMotorBackward(HIGH_SPEED+25);
  }
  delay(1500);
  state = CROSSING_SEPARATOR;
}

void checkOrientationProcedureAfter(){
  turningProcedureReverse();
  turnDegrees(DEGREE_ORIENT+5);
  state = FINDING_EDGE;
}

void checkFindingEdge(){
  sendTrig();
}
  

void isSeparatorCrossed(){
  sendTrig();
}

void checkGoSignal(){
  //Start go signal delay 
  if(checkSwitch()){
    delay(1000);
    runFans();
    sendTrig(); 
    delay(2000);
    state= MOVING_TO_MOVING;
  }
  if (Serial.available() > 0) {
    // read the incoming byte:
    int incomingByte = Serial.read();
    if (incomingByte == '1'){
      runFans();
      sendTrig();
      delay(3000);
      state= MOVING_TO_MOVING;
    }
  }
}

int switchState = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = HIGH;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long goTime = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

boolean checkSwitch(){
//If go signal is pressed, move the state to Moving
  reading = digitalRead(SWITCH_PIN);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (reading == HIGH && previous == LOW && ((millis() - goTime) > debounce)) {
    goTime = millis(); 
    previous = reading; 
    return true;  
  }
  previous = reading;
  return false;
}

void timePrepMotors(){
    delay(TIME_PREP_MOTORS);
    state = MOVING;
}

void ledDisplayState(){
 switch (state) {
    case CALIBRATING:
      digitalWrite(LED_PIN,HIGH);
      break;
    case MOVING:
      blinkLED(MOVING_BLINK_DELAY);
      break;
    case APPROACHING_EDGE:
      blinkLED(APPROACHING_BLINK_DELAY);
      break;
    case TURNING:
      blinkLED(TURNING_BLINK_DELAY);
      break;
    case ORIENTING_TO_SEPARATOR:
      blinkLED(ORIENTING_BLINK_DELAY);
      break;
    case CROSSING_SEPARATOR:
      blinkLED(CROSSING_BLINK_DELAY);
      break;
    case FINISHED:
      digitalWrite(LED_PIN, LOW);
      break;  
  }
}

unsigned long previousMillis = 0; 

void blinkLED(unsigned long ledDelay){

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= ledDelay) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
      digitalWrite(LED_PIN,HIGH);
    } else {
      ledState = LOW;
      digitalWrite(LED_PIN,LOW);
    }
  }
}


void IMUGyroUpdate(){
//          Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
//        Serial.println(" degrees/sec");
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

float turnDegrees(int degrees1){
//Watches IMU reading and return after turning by x degrees
  float z = 0;
 // Serial.println(abs(z));
  float current,prev = 0;
  while(abs(z) < degrees1) { 
      IMULoopUpdate();
      z = z + myIMU.gz/float(Z_AXIS_GYRO_SENSITIVITY_CONSTANT);
   //  Serial.println(z);
     delay(10);
  }
  
 // Serial.println(z);
}


