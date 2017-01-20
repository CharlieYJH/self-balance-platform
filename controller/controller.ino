#include "XV4001BD.h"
#include "Servo.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "IMU.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//// Create objects
IMU imu(XV4001BD(53), MPU6050(0x69));
Servo servo;

// LED and button pins
const int led_pin = 13;
const int button_pin = 3;
const int read_start = 22;

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
    Serial.begin(38400);

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
  
    // Initialize devices
    bool imu_init = imu.initialize();
    servo.attach(23);

    // Make servo levelled
    servo.writeMicroseconds(1500);
  
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

    // Low pass filter on output pulse
    pulse = static_cast<int>(alpha * last_pulse + (1 - alpha) * pulse);
    
    Serial.print(imu.getAccelAngle(ax)); Serial.print("\t"); Serial.print(imu.getGyroAngle()); Serial.print("\t"); Serial.println(imu.getAngle(ax));

    if (last_pulse == pulse) {
      // Buffer prevents servo not moving from small movements
      pulse_buffer++;
      if (pulse_buffer > 8) servo.detach();
    } else {
      // Reset buffer if we have to move servo
      pulse_buffer = 0;
      servo.attach(23);
    }
    
    // Write to servo
    servo.writeMicroseconds(pulse);
    last_pulse = pulse;
}

