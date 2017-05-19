int incomingByte = 0;   // for incoming serial data
int leftMotorDirection,rightMotorDirection;


void turnRight(int speed){
  setLeftMotorForward(speed);
  setRightMotorBackward(speed);
}

void turnLeft(int speed){
   setLeftMotorBackward(speed);
   setRightMotorForward(speed);
}

void turnRightFlipped(int speed){
  setLeftMotorForward(speed);                             
  setRightMotorBackward(speed);
}

void turnLeftFlipped(int speed){
   setLeftMotorBackward(speed);
   setRightMotorForward(speed);
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

void stopMotors(){
  digitalWrite(L1PinLeft,HIGH);
  digitalWrite(L2PinLeft,HIGH);
  digitalWrite(L1PinRight,HIGH);
  digitalWrite(L2PinRight,HIGH);
  analogWrite(dcEnablePin1,0);
  analogWrite(dcEnablePin2,0);
} 



