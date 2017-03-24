#include "XV4001BD.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "IMU.h"
#include "DataSender.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

struct DataSenders {
	DataSender x;
	DataSender y;
};

// Create objects
IMU imu(XV4001BD(29), XV4001BD(28), MPU6050(0x68));
DataSenders sender {
	DataSender(A11, A14),
	DataSender(A12, A13)
};

// LED and button pins
const int calibrate_pin = 24;
const int start_pin = 25;
const int read_start = 22;

bool broadcasted = false;

enum Axis {
  x, y, z
};

void setup() {
    
    // Setup Serial
    Serial.begin(9600);

    // Configure LED
    pinMode(calibrate_pin, OUTPUT);
	pinMode(start_pin, OUTPUT);
    digitalWrite(calibrate_pin, LOW);
	digitalWrite(start_pin, HIGH);

    // Configure jumper pin
    pinMode(read_start, INPUT);
  
    // Initialize devices
    bool imu_init = imu.initialize();
  
    // Verify device connection
    Serial.println(imu_init ? "Device connections successful" : "Device connections failed");
  
    // Set accelerometer offsets
    imu.setAccelerometerOffsets(-272, 1304, 2616);

    // Set IMU constants
    imu.setCompFilterConstant(0.90);
    imu.setAccelFilterConstant(1);

    // Reverse x axis of accelerometer
    imu.setReverseAccel(false, false);
    imu.setReverseGyro(false, false);

    // IMU sensor bias calibration
    imu.calibrate();
  
    // Allow time for chip to setup
    delay(5000);

    // Display sensor connection on board
    digitalWrite(calibrate_pin, HIGH);
	digitalWrite(start_pin, LOW);
}

void loop() {

    // Update IMU reading
    imu.update();

	// Read angles from sensors
    float angle_x = imu.getAngle(x);
	float angle_y = imu.getAngle(y);

    // Bound angle within +/-32 degrees
    angle_x = (angle_x > 32) ? 32
                         	 : (angle_x < -32) ? -32
                         				   	   : angle_x;

    angle_y = (angle_y > 32) ? 32
                         	 : (angle_y < -32) ? -32
                         				   	   : angle_y;

    // Determine servo pulse needed from angle
    int pulse_x = (angle_x < 0) ? 1.3 * angle_x * 10.8
								: 1.2 * angle_x * 10.8;
	int pulse_y = 1.39 * angle_y * 10.8;

	sender.x.transmit(pulse_x);
	sender.y.transmit(pulse_y);

    if (!digitalRead(read_start)) {
      //Serial.print(millis()); Serial.print("\t"); Serial.print(imu.getVelocity(x)); Serial.print("\t"); Serial.print(imu.getVelocity(y)); Serial.print("\t"); Serial.println(-imu.getAngle(x));
    }

	// Serial.print(imu.getAccelAngle(x)); Serial.print("\t"); Serial.print(imu.getGyroAngle(x)); Serial.print("\t");
	// Serial.print(imu.getAccelAngle(y)); Serial.print("\t"); Serial.println(imu.getGyroAngle(y));

	Serial.print(imu.getAngle(x)); Serial.print("\t"); Serial.print(imu.getAngle(y)); Serial.print("\t"); Serial.println(imu.getVelocity(y));
}

