#include <MedianFilter.h>

//UltraSonicPins
#define FORWARD_ULTRASONIC_SENSOR 9
#define BACKWARD_ULTRASONIC_SENSOR 10 
#define LEFT_SIDE_ULTRASONIC_SENSOR 11
 #define RIGHT_SIDE_ULTRASONIC_SENSOR 13
#define trigPin 8 // Trigger Pin
 float duration, distance; // Duration used to calculate distance
int maximumRange = 400; // Maximum range of sensor
int minimumRange = 2; // Minimum range of sensor
MedianFilter medianUltra(3,0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(FORWARD_ULTRASONIC_SENSOR, INPUT);
  pinMode(BACKWARD_ULTRASONIC_SENSOR, INPUT);
  pinMode(LEFT_SIDE_ULTRASONIC_SENSOR, INPUT);
  pinMode(RIGHT_SIDE_ULTRASONIC_SENSOR, INPUT);
}

void loop() { 
   getUltraSensorValue(BACKWARD_ULTRASONIC_SENSOR);
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
