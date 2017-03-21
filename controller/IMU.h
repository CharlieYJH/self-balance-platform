#ifndef IMU_H_
#define IMU_H_

#include "Arduino.h"
#include "XV4001BD.h"
#include "MPU6050.h"
#include "I2Cdev.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/*
 * Combined IMU class
 */

class IMU {

public:

	/*
	 * IMU()
	 * Constructor
	 * @param[gyro_x]: gyroscope object for the x-axis
	 * @param[gyro_y]: gyroscope object for the y-axis
	 * @param[accel]: accelerometer object
	 */
	IMU(XV4001BD gyro_x, XV4001BD gyro_y, MPU6050 accel);

	/*
	 * initialize()
	 * Initialize devices
	 * @return: initialization status
	 */
	bool initialize();

	/*
	 * calibrate()
	 * Calibrates sensor biases
	 */
	void calibrate();

	/*
	 * setAccelerometerOffsets()
	 * Sets MPU offsets
	 * @param[ax]: Offset for x axis
	 * @param[ay]: Offset for y axis
	 * @param[az]: Offset for z axis
	 */
	void setAccelerometerOffsets(int ax, int ay, int az);

	/*
	 * setCompFilterConstant()
	 * Sets the complementary filter constant
	 * @param[alpha]: the filter constant
	 */
	void setCompFilterConstant(float alpha);

	/*
	 * setAccelFilterConstant()
	 * Sets the accelerometer low pass filter constant
	 * @param[alpha]: the filter constant
	 */
	void setAccelFilterConstant(float alpha);

	/*
	 * reverseAccelerometer()
	 * @param[direction]: accelerometer direction (false: forward, true: backward)
	 * Reverses accelerometer direction
	 */
	void setReverseAccel(bool x_dir, bool y_dir);

	/*
	 * reverseGyro()
	 * Reverses gyroscope direction
	 * @param[x_dir]: gyroscope direction in x (false: forward, true: backward)
	 * @param[y_dir]: gyroscope direction in y (false: forward, true: backward)
	 */
	void setReverseGyro(bool x_dir, bool y_dir);

	/*
	 * update()
	 * Updates gyroscope and accelerometer data
	 */
	void update();

	/*
	 * getAcceleration()
	 * Returns acceleration value in g
	 * @param[axis]: Specified axis (1:x, 2:y: 3:z)
	 * @return: acceleration in g
	 */
	float getAcceleration(int axis) const;

	/*
	 * getVelocity()
	 * Returns gyroscope angular velocity in deg/s
	 * @param[axis]: Specified axis (1:x, 2:y)
	 * @return: gyroscope angular velocit in deg/s
	 */
	float getVelocity(int axis) const;

	/*
	 * getAccelAngle()
	 * Returns accelerometer angle in degrees
	 * @param[axis]: the axis to return (1:x, 1:y)
	 * @return: accelerometer angle in degrees
	 */
	float getAccelAngle(int axis) const;

	/*
	 * getGyroAngle()
	 * Returns gyroscope angle in degrees
	 * @param[axis]: Specified axis (1:x, 2:y)
	 * @return: gyroscope angle in degrees
	 */
	float getGyroAngle(int axis) const;

	/*
	 * getAngle()
	 * Returns the current angle from the combined sensor data
	 * @param[axis]: the desired axis
	 * @return: the filtered angle from the sensors
	 */
	float getAngle(int axis) const;

private:

	// Enum for acceleration axes
	enum Axis {
		x, y, z
	};

	// Struct for two axis gyroscope reading
	struct Gyroscope {
		XV4001BD x;
		XV4001BD y;
	};

	// Gyro and accelerometer objects
	Gyroscope m_gyro;
	MPU6050 m_accel;

	// Gyro and accelerometer data containers
	int16_t m_accel_data[3];
	float m_gyro_data[2];

	// Gyro data bias
	float m_gyro_bias[2];

	// Angle containers
	float m_accel_angle[2];
	float m_gyro_angle[2];
	float m_filtered_angle[2];

	// Complementary filter constant
	float m_output_alpha;

	// Accelerometer low pass filter constant
	float m_accel_alpha;

	// Last execution time
	int m_last;

	// Records how many gyro samples are stationary
	int m_stationary_count[2];

	// Accelerometer and gyroscope directions
	bool m_x_accel_reverse;
	bool m_y_accel_reverse;
	bool m_x_gyro_reverse;
	bool m_y_gyro_reverse;

	/*
	 * initializeGyro()
	 * Initialize gyroscope
	 * @return: returns initialization status
	 */
	bool initializeGyro();

	/*
	 * initializeAccel()
	 * Initialize accelerometer
	 * @return: returns initialization status
	 */
	bool initializeAccel();

	/*
	 * updateAcceleration()
	 * Get raw acceleration data from accelerometer
	 */
	void updateAcceleration();

	/*
	 * updateAccelAngle()
	 * Update the angle obtained from accelerometer
	 */
	void updateAccelAngle();

	/*
	 * updateVelocity()
	 * Get angular rate from gyroscope
	 */
	void updateVelocity();

 };

#endif
