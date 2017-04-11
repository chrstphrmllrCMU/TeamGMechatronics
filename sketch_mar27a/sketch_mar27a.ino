#define SWITCH_PIN 26

void setup() {
  Serial.begin(38400);
}

void loop() {
  if(checkSwitch()){
    Serial.println("Yes");  
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
      
     Serial.println(reading);
    Serial.println(previous);
    Serial.println(millis());
    Serial.println(goTime);
   Serial.println(debounce);
  if (reading == HIGH && previous == LOW && millis() - goTime > debounce) {
     goTime = millis(); 
     previous = reading; 
    return true;  
  }
  previous = reading;
  return false;
}
