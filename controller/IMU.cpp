#include "IMU.h"

/* 
 * IMU()
 * Constructor
 * @param[gyro]: gyroscope object
 * @param[accel]: accelerometer object
 */
IMU::IMU(XV4001BD gyro, MPU6050 accel) 
    : m_gyro(gyro),
      m_accel(accel),
      m_gyro_reverse(false),
      m_x_accel_reverse(false),
      m_y_accel_reverse(false),
      m_gyro_angle(0),
      m_output_alpha(0.95),
      m_accel_alpha(1),
      m_last(0) {};

/*
 * initialize()
 * Initialize devices
 * @return: initialization status
 */
bool IMU::initialize() {
 	return IMU::initializeGyro() && IMU::initializeAccel();
}

/*
 * initialize_gyro()
 * Initialize gyroscope
 * @return: returns initialization status
 */
bool IMU::initializeGyro() {

	// Initialize gyroscope and return connection test result
	m_gyro.initialize();
	return m_gyro.testConnection();
}

/*
 * initialize_accel()
 * Initialize accelerometer
 * @return: returns initialization status
 */
bool IMU::initializeAccel() {

	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Initialize acceleration and return connection test result
    m_accel.initialize();
    return m_accel.testConnection();
}

/*
 * calibrate()
 * Calibrates sensor biases
 */
void IMU::calibrate() {

    const int sample_size = 1024;
    const int garbage_size = 100;
    float gyro_data = 0;

    // Calculate gyro bias based on 1024 samples of gyro data
    for (int i = 0; i < sample_size + garbage_size; i++) {

        // Update velocity
        IMU::updateVelocity();

        // Throw away first 100 samples
        if (i < garbage_size) continue;

        // Sum up gyro data
        gyro_data += m_gyro_data;
    }

    // Return average bias value
    m_gyro_bias = gyro_data / sample_size;
}

/*
 * setAccelerometerOffsets()
 * Sets MPU offsets
 * @param[ax]: Offset for x axis
 * @param[ay]: Offset for y axis
 * @param[az]: Offset for z axis
 */
void IMU::setAccelerometerOffsets(int ax, int ay, int az) {
    m_accel.setXAccelOffset(ax);
    m_accel.setYAccelOffset(ay);
    m_accel.setZAccelOffset(az);
}

/*
 * setCompFilterConstant()
 * Sets the complementary filter constant
 * @param[alpha]: the filter constant
 */
void IMU::setCompFilterConstant(float alpha) {
    m_output_alpha = alpha;
}

/*
 * setAccelFilterConstant()
 * Sets the accelerometer low pass filter constant
 * @param[alpha]: the filter constant
 */
void IMU::setAccelFilterConstant(float alpha) {
    m_accel_alpha = alpha;
}

/*
 * setReverseAccelerometer()
 * @param[x_dir]: accelerometer direction in x (true: backward, false: forward)
 * @param[y_dir]: accelerometer direction in y (true: backward, false: forward)
 * Reverses accelerometer direction
 */
void IMU::setReverseAccel(bool x_dir, bool y_dir) {
    m_x_accel_reverse = x_dir;
    m_y_accel_reverse = y_dir;
}

/*
 * setReverseGyro()
 * @param[direction]: gyroscope direction (true: backward, false: forward)
 * Reverses gyroscope direction
 */
void IMU::setReverseGyro(bool direction) {
    m_gyro_reverse = direction;
}

/*
 * update()
 * Updates gyroscope and accelerometer data
 */
void IMU::update() {

    // Initiate |m_last| on initial call
    if (!m_last) {
        m_last = millis();
        return;
    }

    // Record current time
    int now = millis();

    // Make sure time difference will be greater than 1ms
    if (now - m_last < 2) return;

    // Get time difference since last call
    float dt = (now - m_last) / 1000.0;
    m_last = now;

    // Update gyroscope velocity and accelerometer angle
    IMU::updateVelocity();
    IMU::updateAccelAngle();

    // Update gyroscope angle and fused angle
    m_gyro_angle += m_gyro_data * dt;
    m_filtered_angle[ax] = m_output_alpha * (m_filtered_angle[ax] + (1.145 * m_gyro_data * dt)) + (1 - m_output_alpha) * m_accel_angle[ax];
    m_filtered_angle[ay] = m_output_alpha * (m_filtered_angle[ay] + (1.145 * m_gyro_data * dt)) + (1 - m_output_alpha) * m_accel_angle[ay];
}

/*
 * updateAcceleration()
 * Get raw acceleration data from accelerometer
 */
void IMU::updateAcceleration() {

    // Fill accelerometer raw data container
    int16_t raw_data[3];

    // Get accelerometer data
    m_accel.getAcceleration(&raw_data[ax], &raw_data[ay], &raw_data[az]);

    // Filter the data to remove high frequency noise
    for (int i = 0; i < 3; i++) {
        m_accel_data[i] = m_accel_alpha * raw_data[i] + (1 - m_accel_alpha) * m_accel_data[i];
    }
}

/*
 * updateAccelAngle()
 * Update the angle obtained from accelerometer
 */
void IMU::updateAccelAngle() {
    
    // Update accelerometer data
    IMU::updateAcceleration();

    // Convert to g units
    float accel_x = m_accel_data[ax] / 16384.0;
    float accel_y = m_accel_data[ay] / 16384.0;
    float accel_z = m_accel_data[az] / 16384.0;

    float angle_ratio = 180 / 3.14159;

    // Get angle values
    m_accel_angle[ax] = atan(accel_x / sqrt(accel_y * accel_y + accel_z * accel_z)) * angle_ratio;
    m_accel_angle[ay] = atan(accel_y / sqrt(accel_x * accel_x + accel_z * accel_z)) * angle_ratio;

    // Reverse directions if necessary
    if (m_x_accel_reverse) m_accel_angle[ax] = - m_accel_angle[ax];
    if (m_y_accel_reverse) m_accel_angle[ay] = - m_accel_angle[ay];
}

/*
 * updateVelocity()
 * Updates gyroscope angular velocity in deg/s
 */
void IMU::updateVelocity() {
    // Assign according to direction indicated
    m_gyro_data = (m_gyro_reverse) ? - m_gyro.getAngularRate() - m_gyro_bias
                                   : m_gyro.getAngularRate() - m_gyro_bias;
}

/*
 * getAcceleration()
 * Returns acceleration value in g
 * @param[axis]: Specified axis (1:x, 2:y: 3:z)
 * @return: acceleration in g
 */
float IMU::getAcceleration(int axis) const {
    return m_accel_data[axis] / 16834.0;
}

/*
 * getVelocity()
 * Returns gyroscope angular velocity in deg/s
 * @return: gyroscope angular velocit in deg/s
 */
float IMU::getVelocity() const {
    return m_gyro_data;
}

/*
 * getAccelAngle()
 * Returns accelerometer angle in degrees
 * @param[axis]: the axis to return (1:x, 1:y)
 * @return: accelerometer angle in degrees
 */
float IMU::getAccelAngle(int axis) const {
    return m_accel_angle[axis];
}

/*
 * getGyroAngle()
 * Returns gyroscope angle in degrees
 * @return: gyroscope angle in degrees
 */
float IMU::getGyroAngle() const {
    return m_gyro_angle;
}

/*
 * getAngle()
 * Returns the current angle from the combined sensor data
 * @param[axis]: the desired axis
 * @return: the filtered angle from the sensors
 */
float IMU::getAngle(int axis) const {
    return m_filtered_angle[axis];
}