#include <MedianFilter.h>

enum states {CALIBRATING, MOVING, MOVING_TO_MOVING, APPROACHING_EDGE, TURNING, STOP,ORIENTING_TO_SEPARATOR, CROSSING_SEPARATOR, FINISHED};
int state = 0;
unsigned long MOVING_TIMER;

//#DEFINE LEFT_ULTRASONIC_SENSOR
//#DEFINE RIGHT ULTRASONIC_SENSOR
//#DEFINE TOP_ULTRASONIC_SENSOR
//#DEFINE BOTTOM_ULTRASONIC_SENSOR
//#DEFINE IMU_PIN
/* Motor Pins */
#define L1PinLeft 2
#define dcEnablePin1 3
#define L2PinLeft 4

#define L1PinRight 5
#define dcEnablePin2 6
#define L2PinRight 7

#define FORWARD 1
#define BACKWARD 0 

int motorDirection = FORWARD;

//UltraSonicPins
#define FORWARD_ULTRASONIC_SENSOR 9
#define BACKWARD_ULTRASONIC_SENSOR 10 
#define LEFT_SIDE_ULTRASONIC_SENSOR 11

#define FORWARD_ULTRA_DISTANCE 16
#define BACKWARD_ULTRA_DISTANCE 18.5
//#define RIGHT_SIDE_ULTRASONIC_SENSOR 13
#define trigPin 8 // Trigger Pin
float duration, distance; // Duration used to calculate distance
int maximumRange = 400; // Maximum range of sensor
int minimumRange = 2; // Minimum range of sensor
MedianFilter medianUltra(3,0);

//Debuggin Delays + timer + state 
unsigned long timerA; 
int ledState;
#define MOVING_BLINK_DELAY 500
#define TURNING_BLINK_DELAY 50
#define APPROACHING_BLINK_DELAY 100
#define ORIENTING_BLINK_DELAY 100
#define CROSSING_BLINK_DELAY 5000
#define LED_PIN 13

#define SWITCH_PIN 12

#define STANDARD_SPEED 120
#define SLOW_SPEED 70
#define HIGH_SPEED 120

int currentDirection = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(FORWARD_ULTRASONIC_SENSOR, INPUT);
  pinMode(BACKWARD_ULTRASONIC_SENSOR, INPUT);
  pinMode(LEFT_SIDE_ULTRASONIC_SENSOR, INPUT);
//  pinMode(RIGHT_SIDE_ULTRASONIC_SENSOR, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(L1PinLeft,OUTPUT);
  pinMode(dcEnablePin1,OUTPUT);
  pinMode(L2PinLeft,OUTPUT);

  pinMode(L1PinRight,OUTPUT);
  pinMode(dcEnablePin2,OUTPUT);
  pinMode(L2PinRight,OUTPUT);



//   setLeftMotorBackward();
//   setRightMotorBackward();
//   delay(1000);
//    stopMotors();
//    delay(1000);
//   setLeftMotorForward();
//   setRightMotorForward();
//   delay(1000);
//   stopMotors();
  
}

void loop() {
  Serial.println(states(state));
  checkSensorsForStateChange();
  ledDisplayState();
  switch (state) {
    case CALIBRATING:
      checkForCorner();
      determineDirection();
      startFan();
      break;
    case MOVING:
      runMotors();
      runFans();
      break;
    case MOVING_TO_MOVING:
      prepMotors();
      break;
    case APPROACHING_EDGE:
     // runMotorsSlow();
      stopMotors();
      delay(400);
      break;
    case TURNING:
      turningProcedure();
      break;
    case STOP:
      stopMotors();
      break;
    case ORIENTING_TO_SEPARATOR:
      orientationProcedure();
      break;
    case CROSSING_SEPARATOR:
      adjustFanSpeed();
      adjustMotorSpeed();
      break;
    case FINISHED:
      shutdownSystem();
      break;  
  }
}

void checkForCorner(){

}

void determineDirection(){

}

void startFan(){

}

void prepMotors(){
   if(motorDirection == FORWARD){
    setLeftMotorForward(STANDARD_SPEED);
    setRightMotorForward(STANDARD_SPEED);
  }
  else if(motorDirection == BACKWARD){
    setLeftMotorBackward(STANDARD_SPEED);
    setRightMotorBackward(STANDARD_SPEED);
  }
  delay(500);
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

void runFans(){

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

void turningProcedure(){
if(motorDirection == FORWARD){
  turnLeft(HIGH_SPEED);
}
else{
  turnRight(HIGH_SPEED);
}
//delay(1000);
//motorDirection=!motorDirection;
//     state = MOVING;

}

void adjustFanSpeed(){

}

void adjustMotorSpeed(){

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
      //checkCrossSeparatorReady();
      checkForObstacle();
      isFinished();
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
      orientationProcedure();
      break;
    case CROSSING_SEPARATOR:
      isSeparatorCrossed();
      break;
  }
  if(checkSwitch()){
    state = STOP;
  }
}
void checkCrossSeparatorReady(){
  
}

void checkForObstacle(){
   if(motorDirection == FORWARD){
      if(getUltraSensorValue(FORWARD_ULTRASONIC_SENSOR) < FORWARD_ULTRA_DISTANCE){
       state = APPROACHING_EDGE;
       //  state = TURNING;
      }
   }
   else if(motorDirection == BACKWARD){
      if(getUltraSensorValue(BACKWARD_ULTRASONIC_SENSOR) < BACKWARD_ULTRA_DISTANCE){
       // state = TURNING;
        state = APPROACHING_EDGE;
      }
   }
//  if((getUltraSensorValue() < 10) && currentDirection){
//    state = APPROACHING_EDGE;
//  }
}

void isFinished(){

}

void checkForObstacleStop(){
// if((getUltraSensorValue() < 5)  && currentDirection){
//    state = TURNING;
//    currentDirection = 0;
//  }
if(motorDirection == FORWARD){
      //if(getUltraSensorValue(FORWARD_ULTRASONIC_SENSOR) < FORWARD_ULTRA_DISTANCE){
        state = TURNING;
        timerA = millis();
    //  }
   }
   else if(motorDirection == BACKWARD){
      //if(getUltraSensorValue(BACKWARD_ULTRASONIC_SENSOR) < BACKWARD_ULTRA_DISTANCE){
        state = TURNING;
        timerA = millis();
      //}
   }
}

void checkOrientationTurnCompleted(){
  unsigned long currentTime = millis();
  if((currentTime - timerA) > 100){
      motorDirection=!motorDirection;
     state = MOVING_TO_MOVING;
  }
//  if(checkSwitch()){
////      
//     motorDirection=!motorDirection;
//     state = MOVING;
////  turnRight(STANDARD_SPEED);
//  }
}

void orientationProcedure(){

}

void isSeparatorCrossed(){

}

void checkGoSignal(){
  if(checkSwitch()){
    state = MOVING_TO_MOVING;
  }
}


int switchState = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

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
  if (reading == HIGH && previous == LOW && millis() - goTime > debounce) {
    goTime = millis(); 
     previous = reading; 
    return true;  
  }
  previous = reading;
  return false;
}

void timePrepMotors(){
//  unsigned long currentTime = millis();
//  if((currentTime - MOVING_TIMER)>=500){
    state = MOVING;
//  }
}

float getUltraSensorValue(int echoPin){
  for (int i =0; i<3; i++){
   digitalWrite(trigPin, LOW); 
   delayMicroseconds(2); 
  
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10); 
   
   digitalWrite(trigPin, LOW);
   duration = pulseIn(echoPin, HIGH);
   duration = medianUltra.in(duration); //return median value after new sample processed

   //Calculate the distance (in cm) based on the speed of sound.
   distance = duration/58.2;
   
   if (distance >= maximumRange || distance <= minimumRange){
     /* Send a negative number to computer and Turn LED ON 
     to indicate "out of range" */
     Serial.println("-1");
     int infinity = 10000;
     distance = infinity;
   }
   else {
     /* Send the distance to the computer using Serial protocol, and
     turn LED OFF to indicate successful reading. */
     Serial.println("ultrasonic:"+String(distance)+ " cm");
   }
  }
   return distance;
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

