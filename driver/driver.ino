#include <Servo.h>
#include "DataReceiver.h"

struct DataReceivers {
	DataReceiver x;
	DataReceiver y;
};

struct Servos {
	Servo x;
	Servo y;
};

DataReceivers receiver {
	DataReceiver(13, A3),
	DataReceiver(0, A4)
};

Servos servo {
	Servo(),
	Servo()
};

void setup() {

  // Begin serial communication
  Serial.begin(9600);

  // Use analog pin as digital servo pin
  servo.x.attach(A0);
  servo.y.attach(A1);

  // Wait for master to settle
  delay(3000);
}

void loop()  {

    receiver.x.read();
	receiver.y.read();
    int pulse_x = receiver.x.getData();
	int pulse_y = receiver.y.getData();

    // Change servo drive pulse
	// Serial.print(pulse1); Serial.print("\t");
	// Serial.println(pulse2);
	servo.x.writeMicroseconds(pulse_x + 1440);
	servo.y.writeMicroseconds(pulse_y + 1460);
}
