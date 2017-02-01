#include <Servo.h>
#include "DataReceiver.h"

Servo servo;
DataReceiver receiver(13, A1);

void setup() {

  // Begin serial communication
  Serial.begin(9600);

  // Use analog pin as digital servo pin
  servo.attach(A0);

  // Wait for master to settle
  delay(3000);
}

void loop()  {

    receiver.read();
    int pulse = receiver.getData();

    // Change servo drive pulse
    servo.writeMicroseconds(pulse + 1500);
}
