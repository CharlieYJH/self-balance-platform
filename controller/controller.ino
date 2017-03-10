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
IMU imu(XV4001BD(11), XV4001BD(10), MPU6050(0x68));
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

    float angle = imu.getAngle(ax);

    // Bound angle within +/-32 degrees
    angle = (angle > 32) ? 32
                         : (angle < -32) ? -32
                                         : angle;

    // Determine servo pulse needed from angle
    int pulse = -1.39 * angle * 10.8;

    sender.transmit(pulse);

    if (!digitalRead(read_start)) {
      //Serial.print(millis()); Serial.print("\t"); Serial.print(imu.getVelocity(ax)); Serial.print("\t"); Serial.print(imu.getVelocity(ay)); Serial.print("\t"); Serial.println(-imu.getAngle(ax));
    }
    Serial.print(imu.getAcceleration(ax)); Serial.print("\t"); Serial.print(imu.getAcceleration(ay)); Serial.print("\t"); Serial.print(imu.getAcceleration(az)); Serial.print("\t"); Serial.print(imu.getVelocity(ax)); Serial.print("\t"); Serial.println(imu.getVelocity(ay));
}

