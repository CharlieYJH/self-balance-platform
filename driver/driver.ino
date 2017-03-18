#include <Servo.h>
#include "DataReceiver.h"

struct DataReceivers {
	DataReceiver x;
	DataReceiver y;
};

Servo servo;
DataReceivers receiver {
	DataReceiver(13, A3),
	DataReceiver(0, A4)
};

void setup() {

  // Begin serial communication
  Serial.begin(9600);

  // Use analog pin as digital servo pin
  servo.attach(A0);

  // Wait for master to settle
  delay(3000);
}

void loop()  {

    receiver.x.read();
	receiver.y.read();
    int pulse1 = receiver.x.getData();
	int pulse2 = receiver.y.getData();

    // Change servo drive pulse
    // servo.writeMicroseconds(pulse1 + 1485);
	Serial.print(pulse1); Serial.print("\t");
	Serial.println(pulse2);
}
