#include "DataReceiver.h"

DataReceiver::DataReceiver(int clk, int ack)
    : m_clk(clk),
      m_ack(ack),
      m_ack_bit(false),
      m_data(0)
{
    // Set Arduino pin modes
    pinMode(m_clk, INPUT);
    pinMode(m_ack, OUTPUT);
    digitalWrite(m_ack, LOW);

    for (int i = 0; i < kDataLength; i++) {
        pinMode(kDataPin[i], INPUT);
    }

    m_clk_bit = digitalRead(m_clk);
}

/*
 * read()
 * Read data from data pins and store it in internal data variable
 */
void DataReceiver::read() {

    // Read from data pins if clock changed edge
    if (digitalRead(m_clk) != m_clk_bit) {

        // Update clock bit
        m_clk_bit = !m_clk_bit;

        int data = 0;

        for (int i = 0; i < kDataLength; i++) {

            // Bit shift data into data variable
            data |= digitalRead(kDataPin[i]);
            if (i < kDataLength - 1) data <<= 1;
        }

        if (data & (1 << kDataLength - 1)) {

            // Create a bitmask to sign extend the upper bits if it's a negative number
            int bitmask = (1 << (16 - kDataLength)) - 1;
            data |= (bitmask << kDataLength);
        }

        // Update internally stored data
        m_data = data;

        // Alternate ACK bit
        m_ack_bit = !m_ack_bit;
        digitalWrite(m_ack, m_clk_bit);
    }
}

/*
 * getData()
 * Returns the current data stored in the object from the read method
 * @return: The stored data since the last data read
 */
int DataReceiver::getData() const {
    return m_data;
}
