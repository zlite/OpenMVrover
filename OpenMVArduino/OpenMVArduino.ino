/* Miminimum sketch to translate incoming serial from an OpenMV board to Ardunio commands for motor and servo
 *  Written by Chris Anderson, DIYRobocars, 2017
 */

#include <Servo.h> //servo library
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX
Servo myservoa, myservob; // create servo objects to control servos
int SteeringPin = 10;  // This corresponds to Servo Output 1
int MotorPin = 6;  // This correspondes to Servo Output 2
int CH3 = 5; // Corresponds to Servo Output 3
int CH4 = 4; // Corresponds to Servo Output 4
int CH5 = 9; // Corresponds to Servo Outut 5


unsigned long time;
unsigned long lasttime = 0;
bool LEDState = LOW;

const int BAUD_RATE = 9600;

void setup() {
    Serial.begin(BAUD_RATE);
    myservoa.attach(SteeringPin);// attach servo on Output 1 to servo object
    myservob.attach(MotorPin);// attach servo on Output 2 to servo object
    myservoa.write(90); // test servo
    delay (15);
    myservob.write(10); // test servo
    delay (15);
    mySerial.begin(9600);
    mySerial.println("Hello, world");
    pinMode(LED_BUILTIN, OUTPUT);  // enable LED 
}

void loop() {
  time = millis();
  if (time > lasttime + 1000) {   // flash the LED every second to show "resting" mode
      lasttime = time;
      LEDState = !LEDState; // reverse the LED state
      digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED
       }
  while (Serial.available() > 0) {
   // look for the next valid integer in the incoming serial stream:
    int steer = Serial.parseInt();
    mySerial.println(steer);
    // do it again:
    int motor = Serial.parseInt();
    mySerial.println(motor);
    // look for the newline. That's the end of the commands
    if (Serial.read() == '\n') {
        myservoa.write(steer); // send values to output
        myservob.write(motor);
    LEDState = !LEDState; // reverse the LED state
    digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED to show activity
    delay (15);
    }
  }
}



