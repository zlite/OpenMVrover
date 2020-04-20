
const int BAUD_RATE = 9600;
int RC;
int RC1_pin = 3;
int RC2_pin = 4;
int RC3_pin = 5;
unsigned long RC1_value;
unsigned long RC2_value;
unsigned long RC3_value;


void setup() {
    Serial.begin(BAUD_RATE);
    pinMode(RC1_pin, INPUT);
    pinMode(RC2_pin, INPUT);
    pinMode(RC3_pin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  RC1_value = pulseIn(RC1_pin, HIGH);
  RC2_value = pulseIn(RC2_pin, HIGH);
  RC3_value = pulseIn(RC3_pin, HIGH);

  Serial.print(RC1_value);
  Serial.print("  ");
  Serial.print(RC2_value);
  Serial.print("  ");
  Serial.println(RC3_value);
}
