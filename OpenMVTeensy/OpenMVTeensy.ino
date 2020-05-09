#include <Servo.h> //servo library

Servo myservoa, myservob; // create servo objects to control servos

#define MotorPin 20
#define SteeringPin 21
#define RC1_pin 3
#define RC2_pin 4
#define RC3_pin 5
#define SwitchPin 10

const int BAUD_RATE = 19200;
int RC;
unsigned long RC1_value;
unsigned long RC2_value;
unsigned long RC3_value;
boolean RC_throttle = true;  // if you want to control the throttle manually when it's under OpenMV control

int steer, motor;

unsigned long time;
unsigned long lasttime = 0;
bool LEDState = LOW;

void setup() {
    Serial.begin(BAUD_RATE);   // USB port
    Serial1.begin(BAUD_RATE);  //OpenMV connection
    pinMode(RC1_pin, INPUT);
    pinMode(RC2_pin, INPUT);
    pinMode(RC3_pin, INPUT);
    pinMode(SwitchPin, OUTPUT);
    myservoa.attach(SteeringPin);// attach servo on Output 1 to servo object
    myservob.attach(MotorPin);// attach servo on Output 2 to servo object
    pinMode(LED_BUILTIN, OUTPUT);  // enable LED 
}

void RCcontrol() {
  Serial.println(RC3_value);  // print switch state for debugging
  digitalWrite(SwitchPin, LOW);
  RC1_value = pulseIn(RC1_pin, HIGH);  // read rc inputs
  RC2_value = pulseIn(RC2_pin, HIGH);
  myservoa.write(RC1_value); // mirror values to output
  myservob.write(RC2_value);
  delay (20);
  
}

void OpenMVcontrol() {
    digitalWrite(SwitchPin, HIGH);  // tell OpenMV to take control and start sending data
    while (Serial1.available() > 0) {
         // look for the next valid integer in the incoming serial stream:
      int tempsteer = Serial1.parseInt();  
        // do it again for motor:
      int tempmotor = Serial1.parseInt();
        // look for the newline. That's the end of the commands
      if (Serial1.read() == '\n') {
        steer=tempsteer;
        motor=tempmotor;
        if (RC_throttle) {
            motor = pulseIn(RC2_pin, HIGH);
        }
        LEDState = !LEDState; // reverse the LED state
        digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED to show activity
        steer = constrain(steer,1200,1700);
        motor = constrain(motor,1000,2000);
        Serial.print("Steer: ");
        Serial.print(tempsteer);
        Serial.print(" Motor: ");
        Serial.println(tempmotor);
        myservoa.write(steer); // send values to output
        myservob.write(motor);
        }
      }  
}

void loop() {
  time = millis();
  if (time > lasttime + 1000) {   // flash the LED every second to show "resting" mode
      lasttime = time;
      LEDState = !LEDState; // reverse the LED state
      digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED
       }
  RC3_value = pulseIn(RC3_pin, HIGH);
  if (RC3_value > 1500) {RCcontrol();}   // Use the CH5 switch to decide whether to pass through RC commands or take OpenMV commands
    else {OpenMVcontrol();}

}
