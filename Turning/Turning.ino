
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

#define STANDARD_SPEED 110
#define HIGH_SPEED 110
#define SLOW_SPEED 50

unsigned long timerA;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps

  pinMode(L1PinLeft,OUTPUT);
  pinMode(dcEnablePin1,OUTPUT);
  pinMode(L2PinLeft,OUTPUT);

  pinMode(L1PinRight,OUTPUT);
  pinMode(dcEnablePin2,OUTPUT);
  pinMode(L2PinRight,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  readSerialMotors();
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
         timerA = millis();
         setLeftMotorBackward(HIGH_SPEED);
         setRightMotorForward(HIGH_SPEED);
         waitTurn();
         break;
     case '8':
         timerA = millis();
         setLeftMotorForward(HIGH_SPEED);
         setRightMotorBackward(HIGH_SPEED);
         waitTurn();
        break;
     case '9':
        setLeftMotorForward(SLOW_SPEED);
        setRightMotorForward(SLOW_SPEED);
        break;
    } 
  }
  
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


