//
//#define L1PinLeft 2
//#define dcEnablePin1 3
//#define L2PinLeft 4
//
//#define L1PinRight 5
//#define dcEnablePin2 6
//#define L2PinRight 7
//
//#define FORWARD 1
//#define BACKWARD 0 

int incomingByte = 0;   // for incoming serial data
int leftMotorDirection,rightMotorDirection;


//void setup() {
//  // put your setup code here, to run once:
//  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
//
//  pinMode(L1PinLeft,OUTPUT);
//  pinMode(dcEnablePin1,OUTPUT);
//  pinMode(L2PinLeft,OUTPUT);
//
//  pinMode(L1PinRight,OUTPUT);
//  pinMode(dcEnablePin2,OUTPUT);
//  pinMode(L2PinRight,OUTPUT);
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//  readSerialMotors();
//}

//void readSerialMotors(){
//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    incomingByte = Serial.read();
//    switch(incomingByte){
//      case '1':
//        setLeftMotorForward();
//        setRightMotorForward();
//        break;
//      case '2':
//        setLeftMotorBackward();
//        setRightMotorForward();
//        break;
//     case '3':
//        setLeftMotorForward();
//        setRightMotorBackward();
//        break;
//     case '4':
//        setLeftMotorBackward();
//        setRightMotorBackward();
//        break;
//     case '5':
//        stopMotors();
//        break;
//     case '6':
//        reverseDirection();
//        break;
//    } 
//  }
//}
void turnRight(int speed){
  setLeftMotorForward(speed-GRAVITY_COMPENSATION_LEFT);
  setRightMotorBackward(speed-GRAVITY_COMPENSATION_RIGHT);
}

void turnLeft(int speed){
   setLeftMotorBackward(speed-GRAVITY_COMPENSATION_LEFT);
   setRightMotorForward(speed-GRAVITY_COMPENSATION_RIGHT);
}

void setLeftMotorForward(int speed){
  digitalWrite(L1PinLeft,LOW);
  digitalWrite(L2PinLeft,HIGH);
  analogWrite(dcEnablePin1,speed+GRAVITY_COMPENSATION_LEFT);
  leftMotorDirection=FORWARD;
}

void setRightMotorForward(int speed){
  digitalWrite(L1PinRight,LOW);
  digitalWrite(L2PinRight,HIGH);
  analogWrite(dcEnablePin2,speed+GRAVITY_COMPENSATION_RIGHT);
  rightMotorDirection = FORWARD;
}

void setLeftMotorBackward(int speed){
  digitalWrite(L1PinLeft,HIGH);
  digitalWrite(L2PinLeft,LOW);
  analogWrite(dcEnablePin1,speed+GRAVITY_COMPENSATION_LEFT);
  leftMotorDirection=BACKWARD;
}

void setRightMotorBackward(int speed){
  digitalWrite(L1PinRight,HIGH);
  digitalWrite(L2PinRight,LOW);
  analogWrite(dcEnablePin2,speed+GRAVITY_COMPENSATION_RIGHT);
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
  digitalWrite(L1PinLeft,HIGH);
  digitalWrite(L2PinLeft,HIGH);
  digitalWrite(L1PinRight,HIGH);
  digitalWrite(L2PinRight,HIGH);
  analogWrite(dcEnablePin1,0);
  analogWrite(dcEnablePin2,0);
}


