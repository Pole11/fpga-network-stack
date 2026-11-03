#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void append_payload(
	stream<txPacketInfo>&	packetInfoFifo,
	stream<net_axis<WIDTH> >&	tx_headerFifo,
	stream<net_axis<WIDTH> >&	tx_aethPayloadFifo,
	stream<net_axis<WIDTH> >&	tx_rethPayloadFifo,
	stream<net_axis<WIDTH> >&	tx_rawPayloadFifo,
	stream<net_axis<WIDTH> >&	tx_packetFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmState {INFO, HEADER, AETH_PAYLOAD, RETH_PAYLOAD, RAW_PAYLOAD};
	static fsmState state = INFO;
	static net_axis<WIDTH> prevWord;
	net_axis<WIDTH> currWord;
	net_axis<WIDTH> sendWord;
	static bool firstPayload = true;

	static txPacketInfo info;

	// TODO align this stuff!!
	switch (state)
	{
	case INFO:
		if (!packetInfoFifo.empty())
		{
			firstPayload = true;
			packetInfoFifo.read(info);

			if (info.hasHeader)
			{
				state = HEADER;
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state HEADER" << std::endl;
			}
			else
			{
				state = RAW_PAYLOAD;
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state RAW_PAYLOAD" << std::endl;
			}
		}
		break;
	case HEADER:
		if (!tx_headerFifo.empty())
		{
			tx_headerFifo.read(prevWord);
			//TODO last is not necessary
			if (!prevWord.last)
			{
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: writing header" << std::endl;
				tx_packetFifo.write(prevWord);
			}
			else //last
			{
				if (!info.hasPayload)
				{
					state = INFO;
					tx_packetFifo.write(prevWord);
					std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state INFO" << std::endl;
				}
				else // hasPayload
				{
					prevWord.last = 0;
					if (info.isAETH)
					{
						state = AETH_PAYLOAD;
						std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state AETH_PAYLOAD" << std::endl;
					}
					else //RETH
					{
						if (WIDTH <= RETH_SIZE)
						{
							tx_packetFifo.write(prevWord);
						}
						state = RETH_PAYLOAD;
						std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state RETH_PAYLOAD" << std::endl;
					}
				}
			}
		}
		break;
	case AETH_PAYLOAD:
		if (!tx_aethPayloadFifo.empty())
		{
			tx_aethPayloadFifo.read(currWord);
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD" << INSTID << "]: PAYLOAD WORD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			sendWord = currWord;
			if (firstPayload)
			{
				sendWord.data(31, 0) = prevWord.data(31, 0);
				firstPayload = false;
			}
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD" << INSTID << "]: AETH PAYLOAD" << std::endl;
			print(std::cout, sendWord);
			std::cout << std::endl;
#endif
			tx_packetFifo.write(sendWord);
			if (currWord.last)
			{
				state = INFO;
			}
		}
		break;
	case RETH_PAYLOAD:
		if (!tx_rethPayloadFifo.empty())
		{
			std::cout << "[APPEND PAYLOAD " << INSTID << "]: reading rethPayloadFifo" << std::endl;
			tx_rethPayloadFifo.read(currWord);
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD " << INSTID << "]: PAYLOAD WORD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			sendWord = currWord;
			if (firstPayload && WIDTH > RETH_SIZE)
			{
				sendWord.data(127, 0) = prevWord.data(127, 0);
				firstPayload = false;
			}
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD" << INSTID << "]: RETH PAYLOAD" << std::endl;
			print(std::cout, sendWord);
			std::cout << std::endl;
#endif
			tx_packetFifo.write(sendWord);
			if (currWord.last)
			{
				state = INFO;
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving state INFO" << std::endl;
			}
		}
		break;
	case RAW_PAYLOAD:
		if (!tx_rawPayloadFifo.empty())
		{
			tx_rawPayloadFifo.read(currWord);
			tx_packetFifo.write(currWord);
			if (currWord.last)
			{
				state = INFO;
			}
		}
		break;
	}
}
