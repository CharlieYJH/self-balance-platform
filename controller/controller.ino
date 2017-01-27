#include "XV4001BD.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "IMU.h"
#include "DataCarrier.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//// Create objects
IMU imu(XV4001BD(53), MPU6050(0x69));
DataCarrier receiver(1, 2, 1);
DataCarrier receiver2(1, 2, 1);
DataCarrier sender(2, 1);

// LED and button pins
const int led_pin = 13;
const int button_pin = 3;
const int servo_pin = 23;
const int read_start = 22;
const int clock_pin = A13;
bool clock_signal = false;

bool broadcasted = false;

// Variables to smooth out servo movements and hold position
int last_pulse;
int pulse_buffer = 0;
int alpha = 0.6;

enum Acceleration {
  ax, ay, az
};

void setup() {
    
    // Setup Serial
    Serial.begin(57600);

    // Activate rows of pins to communicate data directly to second Arduino
    for (int i = A2; i < A14; i++) {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }

    // Configure LED
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    
    // Configure button
    pinMode(button_pin, OUTPUT);
    // Use internal pullup for button
    digitalWrite(button_pin, HIGH);

    // Configure jumper pin
    pinMode(read_start, OUTPUT);
    digitalWrite(read_start, LOW);

    pinMode(servo_pin, OUTPUT);
    digitalWrite(servo_pin, LOW);
  
    // Initialize devices
    bool imu_init = imu.initialize();
  
    // Verify device connection
    Serial.println(imu_init ? "Device connections successful" : "Device connections failed");
  
    // Set accelerometer offsets
    imu.setAccelerometerOffsets(-1616, -174, 1060);

    // Set IMU constants
    imu.setCompFilterConstant(0.995);
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

    if (!broadcasted) {
      digitalWrite(read_start, HIGH);
      broadcasted = true;
    }

    // Determine servo pulse needed from angle
    int pulse = (imu.getAngle(ax)) * 10.8 + 1490;

    int num = pulse;

    // Write binary to pins
    for (int i = 0; i < 11; i++) {
      int num_bit = num & 1;
      num >>= 1;
      digitalWrite(i + A2, LOW);
      digitalWrite(i + A2, num_bit);
    }

    // Invert clock pin
    clock_signal = !clock_signal;
    digitalWrite(clock_pin, clock_signal);
    
    last_pulse = pulse;

    Serial.println(HIGH);
}

