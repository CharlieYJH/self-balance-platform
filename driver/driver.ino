#include <Servo.h>
#include "DataReceiver.h"

Servo servo;
DataReceiver receiver(13, A1);

void setup() {

  // Begin serial communication
  Serial.begin(9600);

  // Set all communication pins as input
  for (int i = 2; i < 13; i++) {
    pinMode(i, INPUT);
  }

  // Use analog pin as digital servo pin
  servo.attach(A0);

  // Wait for master to settle
  delay(3000);
}

void loop()  {

    receiver.read();
    int pulse = receiver.getData();

    // Change servo drive pulse
    servo.writeMicroseconds(pulse + 1490);
}
