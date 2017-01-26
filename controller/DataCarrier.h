#ifndef DATACARRIER_H_
#define DATACARRIER_H_


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
	DataCarrier(int ack, int clk);

	/*
	 * DataCarrier()
	 * Class constructor for sender
	 * @param[clk]: Clock bit
	 */
	DataCarrier(int clk);

};

#endif