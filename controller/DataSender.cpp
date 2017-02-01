#include "DataSender.h"

// Link static variables
DataSender::ACK_pin DataSender::m_receivers[kMaxReceivers] = {0};
int DataSender::ACK_count = 0;

/*
 * Class constructor
 * @param[clk]: Sender clock signal pin
 * @param[ack]: Intended receiver ACK signal pin
 */
DataSender::DataSender(int clk, int ack)
	: m_clk(clk),
	  m_clk_bit(false),
	  m_ack(ack)
{
	if (ACK_count < kMaxReceivers) {

		m_ack_index = ACK_count;

		// Fill ACK signal array with new ACK signal
		m_receivers[m_ack_index].pin = ack;
		m_receivers[m_ack_index].active = true;

		// Set Arduino pin modes
		pinMode(ack, INPUT);
		pinMode(clk, OUTPUT);
		digitalWrite(clk, LOW);

		for (int i = 0; i < kDataLength; i++) {
			pinMode(kDataPin[i], OUTPUT);
			digitalWrite(kDataPin[i], LOW);
		}

		ACK_count++;
	}
}

/*
 * trasnmit()
 * Transmit the given data to the sender through the data pins
 * @param[data]: The given data to transmit
 */
void DataSender::transmit(int data) {

	for (int i = 0; i < kMaxReceivers; i++) {

		// Don't transmit if an active ACK pin is busy
		if (m_receivers[i].active && !digitalRead(m_receivers[i].pin)) {
			
			return;
		}
	}

	// Send data through data pins by bit banging
	for (int i = 0; i < kDataLength; i++) {

		int write_bit = data & 1;
		data >>= 1;

		digitalWrite(kDataPin[i], write_bit);
	}

	// Switch the clock signal to notify receiver
	m_clk_bit = !m_clk_bit;
	digitalWrite(m_clk, m_clk_bit);
}