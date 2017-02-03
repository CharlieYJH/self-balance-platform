#ifndef DATARECEIVER_H_
#define DATARECEIVER_H_

#include "Arduino.h"

class DataReceiver {

public:

	/*
	 * DataReceiver()
	 * Class constructor
	 * @param[clk]: Clock signal pin
	 * @param[ack]: ACK signal pin
	 */
	DataReceiver(int clk, int ack);

	/*
	 * receive()
	 * Read data from data pins and store it in internal data variable
	 */
	void read();

	/*
	 * getData()
	 * Returns the current data stored in the object from the read method
	 * @return: The stored data since the last data read
	 */
	int getData() const;

private:

	int m_clk;
	bool m_clk_bit;
	int m_ack;
	int m_data;

	// Data transmission pins for Arduino
	const int kDataLength = 11;
	const int kDataPin[11] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
};

#endif