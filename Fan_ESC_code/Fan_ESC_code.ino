/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>
#define FAN_PIN 8
#define FAN_PIN2 9

int value = 0; // set values you need to zero

Servo firstESC, secondESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

  firstESC.attach(FAN_PIN);    // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(FAN_PIN2);
  Serial.begin(38400);   

}

void setupFan(){
   firstESC.attach(FAN_PIN);    // attached to pin 9 I just do this with 1 Servo
  secondESC.attach(FAN_PIN2);
  Serial.begin(38400);    // start serial at 9600 baud
}
/*
 * 
 * 
 * * Connect your ESC
* Configure the code (as much as ESC's you have and Pin-configuration)
* You should hear nothing, because the Arduino sends a zero
* Open your Serial Monitor and send '2000'. it means the highest Signal the ESC can receive
* You will hear the sounds which are described on the picture ( Source: Manual: http://www.hobbyking.com/hobbyking/store/uploads/811103388X7478X20.pdf)


The Hobbyking ESC's can receive a Signal between 700 and 2000 us(microseconds). 700 means throttle at lowest position and 2000 on the highest Position. If you want to know what exactly you du, when you pick a menu visit the manual.

Example:
- Write 2000 us
- Wait until D-D-D-D to chose lipo as batterytype
- When it apperas, write at the third 'D' 700 in your Serial ( short delay, thats why you have to send it at the third 'D')
- ESC will make a sound, and the Option is chosed
 */
 /*
  * 
  * - Upload the code in Arduino
- Write 2000 in the Serial window
- Wait for 4 beeps, then write 700
- Wait for the beeps to finish, then write 750
- Now write whatever speed you like (Min = 755, Max = 2000) .. Be careful when writing 2000! It moves like a monster
  */
void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  firstESC.writeMicroseconds(value);
  secondESC.writeMicroseconds(value);
  if(Serial.available()) 
    value = Serial.parseInt();    // Parse an Integer from Serialji

}

