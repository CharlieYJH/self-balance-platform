#ifndef XV4001BD_H_
#define XV4001BD_H_

#include <Arduino.h>
#include <SPI.h>

/* 
 * XV4001BD Class
 */
class XV4001BD {

public:

	/* Constructor and Destructor */
	XV4001BD(int address);
	~XV4001BD();

	/*
	 * initialize()
	 * Configures gyro for operation
	 */
	void initialize();

	/*
	 * testConnection()
	 * Verify device connection
	 * @return: device connection status
	 */
	bool testConnection();

	/*
	 * getAngularRate()
	 * Fetches current angular rate through SPI
	 * @return: relative angle in degrees
	 */
	float getAngularRate();

private:

	// Slave address
	int m_address;
  	
	// Command length
	int m_cmd_length;

	// SPI transmission message
  	const byte m_angle_cmd[4] = {0x48, 0x26, 0x53, 0x58};

  	/*
  	 * sendCommand()
  	 * Sends the specified command to the device
  	 * @return: the returned message
  	 */
  	int32_t sendCommand(const byte * cmd);
};

#endif