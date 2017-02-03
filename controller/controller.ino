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

//// Create objects
IMU imu(XV4001BD(53), MPU6050(0x69));
DataSender sender(A0, A1);

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
    imu.setAccelerometerOffsets(-1588, -158, 1036);

    // Set IMU constants
    imu.setCompFilterConstant(0.97);
    imu.setAccelFilterConstant(1);

    // Reverse x axis of accelerometer
    imu.setReverseAccel(true, false);
    imu.setReverseGyro(true);

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

    int angle = -1.35 * imu.getAngle(ax);

    // Determine servo pulse needed from angle
    int pulse = angle * 10.8;

    sender.transmit(pulse);

    if (digitalRead(read_start)) {
      Serial.print(millis()); Serial.print("\t"); Serial.print(imu.getVelocity()); Serial.print("\t"); Serial.println(imu.getAngle(ax));
    }
}

