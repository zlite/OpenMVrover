
/* Miminimum sketch to translate incoming serial from an OpenMV board to Ardunio commands for motor and servo
 *  Written by Chris Anderson, DIYRobocars, 2017
 */
#include <PWMServo.h>  // use this rather than the standard Servo to avoid interrupt clashes with SoftwareSerial
#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 4); // RX, TX
PWMServo myservoa, myservob, myservoc, myservod, myservoe; // create servo objects to control servos
int SteeringPin = 10;  // This corresponds to Servo Output 1
int MotorPin = 6;  // This correspondes to Servo Output 2
int CH3 = 5; // Corresponds to Servo Output 3
int CH4 = 4; // Corresponds to Servo Output 4
int CH5 = 9; // Corresponds to Servo Outut 5
int steer, motor;

unsigned long time;
unsigned long lasttime = 0;
bool LEDState = LOW;

const int BAUD_RATE = 9600;

void setup() {
    mySerial.begin(BAUD_RATE);
    myservoa.attach(SteeringPin);// attach servo on Output 1 to servo object
    myservob.attach(MotorPin);// attach servo on Output 2 to servo object 
    Serial.begin(BAUD_RATE);  // For debugging
    Serial.println("Hello, world");  // For debugging
    pinMode(LED_BUILTIN, OUTPUT);  // enable LED 
}

void loop() {
  time = millis();
  if (time > lasttime + 1000) {   // flash the LED every second to show "resting" mode
      lasttime = time;
      LEDState = !LEDState; // reverse the LED state
      digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED
       }
  while (mySerial.available() > 0) {
   // look for the next valid integer in the incoming serial stream:
    int tempsteer = mySerial.parseInt();
    Serial.println(steer);   // turn this on for debgging. 
    // do it again:
    int tempmotor = mySerial.parseInt();
    Serial.println(motor);   // turn this on for debgging. 
    // look for the newline. That's the end of the commands
    if (mySerial.read() == '\n') {
        steer=tempsteer;
        motor=tempmotor;
        LEDState = !LEDState; // reverse the LED state
        digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED to show activity
    }
  }
  myservoa.write(steer); // send values to output
  myservob.write(motor);
  delay (20);
}



