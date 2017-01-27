#ifndef DATACARRIER_H_
#define DATACARRIER_H_

#include <stdio.h>
#include <stdlib.h>
#include <Arduino.h>

/*
 * Data Carrier Class
 * Used to transfer data using digital pins between two Arduinos
 */
class DataCarrier {

public:

	/*
	 * DataCarrier()
	 * Class constructor for receiver
	 * @param[ack]: Acknowledge bit for receiver
	 * @param[clk]: Clock bit
	 */
	DataCarrier(int ack, int clk, int config);

	/*
	 * DataCarrier()
	 * Class constructor for sender
	 * @param[clk]: Clock bit
	 */
	DataCarrier(int clk, int config);

	/*
	 * transmit()
	 * Transmits the given data over the data lines
	 * @param[data]: The given data in integer form
	 */
	void transmit(int data);

	bool Test();

private:

	static const int kMaxSlaves = 2;

	// Slaves struct to keep track of slave addresses and statuses
	struct slaves {
		int ack_pin;
		int ack_status;
	};

	// Slave array to keep track of slave information (Max number of slaves: 2)
	static slaves m_slave[kMaxSlaves];

	// Slave count keep tracks of how many slaves are currently attached
	static int m_slave_count;

	int m_ack;
	int m_clk;
	int m_config;
	int m_slave_index;
	bool m_is_sender;

	/*
	 * CheckSlaveStatus()
	 * Checks current slave busy status
	 * @return: Whether all slaves are free (true: yes, false: no)
	 */
	bool checkSlaveStatus();
};

#endif