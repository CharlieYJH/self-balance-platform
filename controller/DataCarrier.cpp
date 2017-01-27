#include "DataCarrier.h"

// Initialize and link static variables
int DataCarrier::m_slave_count = 0;
DataCarrier::slaves DataCarrier::m_slave[kMaxSlaves];

/*
 * DataCarrier()
 * Class constructor for receiver
 * @param[ack]: Acknowledge pin for receiver
 * @param[clk]: Clock pin
 * @param[config]: Syncs on high or low pulse (0: low, 1: high)
 */
DataCarrier::DataCarrier(int clk, int config, int ack)
	: m_clk(clk),
	  m_config(config),
	  m_ack(ack),
	  m_is_sender(false)
{
	// Make sure new receiver is within max slave count
	if (m_slave_count < kMaxSlaves) {

		// Assign slave number
		m_slave_index = m_slave_count;
		m_slave[m_slave_index].ack_pin = m_ack;
		m_slave[m_slave_index].ack_status = 1;

		// Increment slave count
		m_slave_count++;

	} else {

		// -1 used to designate too many slaves
		m_slave_index = -1;
	}
}

/*
 * DataCarrier()
 * Class constructor for sender
 * @param[clk]: Clock pin
 * @param[config]: Pulse high or low as synchronisation (0: pulse low, 1: pulse high)
 */
DataCarrier::DataCarrier(int clk, int config)
	: m_clk(clk),
	  m_config(config),
	  m_is_sender(true) {}

void DataCarrier::transmit(int data) {

	// If pin is not the sender or some receiver is still busy, don't transmit
	if (!m_is_sender || !DataCarrier::checkSlaveStatus()) return;
}

/*
 * CheckSlaveStatus()
 * Checks current slave busy status
 * @return: Whether all slaves are free (true: yes, false: no)
 */
bool DataCarrier::checkSlaveStatus() {

	// Look through all slaves and check ACK bit
	for (int i = 0; i < m_slave_count; i++) {

		// If ACK bit is low on any slaves, it is busy
		if (!m_slave[i].ack_status) return false;
	}
	
	return true;
}

bool DataCarrier::Test() {
	return DataCarrier::checkSlaveStatus();
}