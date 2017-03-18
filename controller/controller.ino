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
IMU imu(XV4001BD(11), XV4001BD(10), MPU6050(0x68));
DataSenders sender {
	DataSender(A11, A14),
	DataSender(A12, A13)
};

// LED and button pins
const int led_pin = 13;
const int read_start = 22;

bool broadcasted = false;

enum Acceleration {
  ax, ay, az
};

void setup() {
    
    // Setup Serial
    Serial.begin(9600);

    // Configure LED
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);

    // Configure jumper pin
    pinMode(read_start, INPUT);
  
    // Initialize devices
    bool imu_init = imu.initialize();
  
    // Verify device connection
    Serial.println(imu_init ? "Device connections successful" : "Device connections failed");
  
    // Set accelerometer offsets
    imu.setAccelerometerOffsets(-268, 1260, 2626);

    // Set IMU constants
    imu.setCompFilterConstant(0.90);
    imu.setAccelFilterConstant(1);

    // Reverse x axis of accelerometer
    imu.setReverseAccel(true, false);
    imu.setReverseGyro(false, false);

    // IMU sensor bias calibration
    imu.calibrate();
  
    // Allow time for chip to setup
    delay(2000);

    // Display sensor connection on board
    digitalWrite(led_pin, imu_init);
}

void loop() {

    // Update IMU reading
    imu.update();

	// Read angles from sensors
    float angle_x = imu.getAngle(ax);
	float angle_y = imu.getAngle(ay);

    // Bound angle within +/-32 degrees
    angle_x = (angle_x > 32) ? 32
                         : (angle_x < -32) ? -32
                         				   : angle_x;

    angle_y = (angle_y > 32) ? 32
                         : (angle_y < -32) ? -32
                         				   : angle_y;

    // Determine servo pulse needed from angle
    int pulse_x = -1.39 * angle_x * 10.8;
	int pulse_y = -1.39 * angle_y * 10.8;

    // sender.x.transmit(pulse_x);
	// sender.y.transmit(pulse_y);

	sender.x.transmit((int)random(-400, 400));
	sender.y.transmit((int)random(-400, 400));

    if (!digitalRead(read_start)) {
      //Serial.print(millis()); Serial.print("\t"); Serial.print(imu.getVelocity(ax)); Serial.print("\t"); Serial.print(imu.getVelocity(ay)); Serial.print("\t"); Serial.println(-imu.getAngle(ax));
    }
    Serial.print(imu.getAcceleration(ax)); Serial.print("\t"); Serial.print(imu.getAcceleration(ay)); Serial.print("\t"); Serial.print(imu.getAcceleration(az)); Serial.print("\t"); Serial.print(imu.getVelocity(ax)); Serial.print("\t"); Serial.println(imu.getVelocity(ay));
}

