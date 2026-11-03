#pragma once

#include "../mrc_headers.hpp"

//TODO this introduces 1 cycle for WIDTH > 64
template <int WIDTH, int INSTID = 0>
void prepend_ibh_header(
	stream<BaseTransportHeader<WIDTH> >& tx_ibhHeaderFifo,
	stream<net_axis<WIDTH> >& tx_ibhPayloadFifo,
	stream<net_axis<WIDTH> >& m_axis_tx_data,
    ap_uint<32>&		regIbvCountTx
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum pihStatea {GET_HEADER, HEADER, PARTIAL_HEADER, BODY};
	static pihStatea state = GET_HEADER;
	static BaseTransportHeader<WIDTH> header;
	static ap_uint<WIDTH> headerData;
	net_axis<WIDTH> currWord;
    static ap_uint<32> validTx = 0;

	switch (state)
	{
	case GET_HEADER:
		if (!tx_ibhHeaderFifo.empty())
		{
			tx_ibhHeaderFifo.read(header);
			if (BTH_SIZE >= WIDTH)
			{
				state = HEADER;
				std::cout << "[PREPEND IBH HEADER " << INSTID << "]: going to state HEADER" << std::endl;
			}
			else
			{
				state = PARTIAL_HEADER;
				std::cout << "[PREPEND IBH HEADER " << INSTID << "]: going to state PARTIAL_HEADER" << std::endl;
			}
		}
		break;
	case HEADER:
	{
		ap_uint<8> remainingLength = header.consumeWord(currWord.data);
		if (remainingLength < (WIDTH/8))
		{
			state = PARTIAL_HEADER;
		}
		currWord.keep = ~0;
		currWord.last = 0;
		m_axis_tx_data.write(currWord);

		break;
	}
		break;
	case PARTIAL_HEADER:
		if (!tx_ibhPayloadFifo.empty())
		{
			tx_ibhPayloadFifo.read(currWord);
#ifdef DBG_FULL
			std::cout << "[PREPEND IBH HEADER " << INSTID << "]: IBH PARTIAL PAYLOAD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			header.consumeWord(currWord.data);
			m_axis_tx_data.write(currWord);

#ifdef DBG_FULL
			std::cout << "[PREPEND IBH HEADER " << INSTID << "]: IBH PARTIAL HEADER" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			state = BODY;
			if (currWord.last)
			{
                validTx++;
		        regIbvCountTx = validTx;

				state = GET_HEADER;
			}
		}
		break;
	case BODY:
		if (!tx_ibhPayloadFifo.empty())
		{
			tx_ibhPayloadFifo.read(currWord);
			m_axis_tx_data.write(currWord);

#ifdef DBG_FULL
			std::cout << "[PREPEND IBH HEADER " << INSTID << "]: IBH PAYLOAD WORD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			if (currWord.last)
			{
                validTx++;
		        regIbvCountTx = validTx;
                
				state = GET_HEADER;
			}
		}
		break;
	}
}
