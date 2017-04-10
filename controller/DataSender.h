#ifndef DATASENDER_H_
#define DATASENDER_H_

#include "Arduino.h"

class DataSender {

public:

	/*
	 * DataSender()
	 * Class constructor
	 * @param[clk]: Sender clock signal pin
	 * @param[ack]: Intended receiver ACK signal pin
	 */
	DataSender(int clk, int ack);

	/*
	 * trasnmit()
	 * Transmit the given data to the sender through the data pins
	 * @param[data]: The given data to transmit
	 */
	void transmit(int data);

private:

	struct ACK_pin {
		int pin;
		bool active;
	};

	// Static variable so each sender knows all ACK pins
	static const uint8_t kMaxReceivers = 2;
	static int ACK_count;

	// Data transmission pins for sender Arduino
	const int kDataLength = 11;
	const int kDataPin[11] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};

	int m_clk;
	int m_ack;
	int m_ack_bit;
	int m_ack_index;
	bool m_clk_bit;
};

#endif
