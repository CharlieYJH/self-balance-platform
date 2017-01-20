#include "XV4001BD.h"

/*
 * XV4001BD Class
 */

/* Constructor */
XV4001BD::XV4001BD(int address = 53) : m_address(address), m_cmd_length(4) {}

/* Destructor */
XV4001BD::~XV4001BD() {}

/*
 * initialize()
 * Configures gyro for operation
 */
void XV4001BD::initialize() {
	// Set slave pin as output pin and start SPI
	SPI.begin();
	pinMode(m_address, OUTPUT);
}

/*
 * testConnection()
 * Verify device connection
 * @return: device connection status
 */
bool XV4001BD::testConnection() {
	// Send a test command to the device and return DIAG flag
	return !((XV4001BD::sendCommand(m_angle_cmd) >> 24) & 0b01000000);
}

/*
 * sendCommand()
 * Sends the specified command to the device
 * @return: the returned message
 */
int32_t XV4001BD::sendCommand(const byte * cmd) {
	// 32 bit message container
	int32_t result = 0;

 	// Loop through all four bytes of commands
 	for (int i = 0; i < m_cmd_length; i++) {
 		// Pull SS pin low
 		digitalWrite(m_address, LOW);

 		// Send command and receive bytes, OR onto result
 		result |= SPI.transfer(cmd[i]);

 		if (i < m_cmd_length - 1) {
 			// If we're not at the last command, shift result left by |sizeof(byte)|
 			result <<= 8;
 		}

 		// Pull SS pin high
 		digitalWrite(m_address, HIGH);
 	}

 	// Return result
 	return result;	
}

/*
 * getAngularRate()
 * Fetches current angular rate through SPI
 * @return: relative angle in degrees
 */
 float XV4001BD::getAngularRate() {
 	// Return result in degree form (Take lower 16 bits)
 	return static_cast<int16_t>(XV4001BD::sendCommand(m_angle_cmd)) / 370.0;
 }