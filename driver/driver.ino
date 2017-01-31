#include <Servo.h>

const int clock_pin = 13;
uint8_t clock_bit;

Servo servo;

void setup() {

  // Begin serial communication
  Serial.begin(57600);

  // Set all communication pins as input
  for (int i = 2; i < 13; i++) {
    pinMode(i, INPUT);
  }

  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);

  // Use analog pin as digital servo pin
  servo.attach(A0);

  // Wait for master to settle
  delay(3000);

  // Initiate clock bit
  clock_bit = digitalRead(clock_pin);
}

void loop()  {

  // Read binary value from other Arduino if clock bit is inverted since last read
  if (digitalRead(clock_pin) != clock_bit) {

    // Invert clock bit and initate reading container
    int pulse = 0;
    clock_bit = !clock_bit;

    // Read pin values and concatenate into 16 bit integer
    for (int i = 0; i < 11; i++) {
      pulse |= digitalRead(i + 2);
      if (i < 10) pulse <<= 1;
    }

    Serial.println(pulse);

    // Change servo drive pulse
    servo.writeMicroseconds(pulse);
  }
}
