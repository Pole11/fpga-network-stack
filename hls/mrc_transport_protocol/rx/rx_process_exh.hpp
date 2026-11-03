#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void rx_process_exh( // probably it is useless now ... 
	stream<net_axis<WIDTH> >& input, // packet without BTH, without METH and without TSETH, it starts with RETH
	stream<exhOpMeta>& metaIn,
	stream<exhMeta>& exhMetaFifo, // probably useless ?!
	stream<ExHeader<WIDTH> >& metaOut,
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, RETH_HEADER, NO_HEADER};

	static fsmStateType state = META;

	static RdmaExHeader<WIDTH> rdmaHeader;
	static packetHeader<WIDTH, RETH_SIZE + IMMDT_SIZE> rethImmHeader; // NEW: RETH + ImmDt, *_IMM only

	static bool metaWritten = false;
	static bool rethParsed  = false;
	static bool hasImmDt    = false;

	net_axis<WIDTH> currWord;
	static exhOpMeta opMeta;
	static mrcOpCode opCode;

	switch (state)
	{
	case META:
		if (!metaIn.empty())
		{
			metaIn.read(opMeta);
			opCode = opMeta.op_code;
			metaWritten = false;
			rethParsed  = false;
			hasImmDt    = checkIfImmDt(opCode);

			state = opMeta.mrc.hasMeth ? RETH_HEADER : NO_HEADER; // every MRC request carries a RETH
			std::cout << "[RX PROCESS EXH " << INSTID << "] state:" << state << std::endl;
		}
		break;
	case RETH_HEADER:
		if (!input.empty())
		{
			input.read(currWord);
			rdmaHeader.parseWord(currWord.data);
			rethImmHeader.parseWord(currWord.data);

			if (rethParsed && WIDTH <= RETH_SIZE)
			{
				output.write(currWord);
			}

			if (rdmaHeader.isReady())
			{
				rethParsed = true;
				if (checkIfWrite(opCode) && WIDTH > RETH_SIZE)
				{
					output.write(currWord);
				}
			}

			// non-IMM: same word as before. IMM: wait for the ImmDt (or the end of a malformed packet)
			if (!metaWritten && rdmaHeader.isReady() && (!hasImmDt || rethImmHeader.isReady() || currWord.last))
			{
				exhMetaFifo.write(exhMeta()); 

				ExHeader<WIDTH> exh(rdmaHeader);
				exh.mrc = opMeta.mrc;
				if (hasImmDt)
				{
					// ImmDt: the 4 bytes right after the RETH, big-endian
					// on a malformed IMM packet that ends before the ImmDt is complete, the meta is still written
					exh.mrc.hasImmDt = true;
					exh.mrc.immDt    = reverse((ap_uint<IMMDT_SIZE>) rethImmHeader.header(RETH_SIZE + IMMDT_SIZE - 1, RETH_SIZE));
				}
				metaOut.write(exh);
				metaWritten = true;
			}
			if (currWord.last)
			{
				rdmaHeader.clear();
				rethImmHeader.clear();
				state = META;
			}
		}
		break;
	case NO_HEADER:
		if (!input.empty())
		{
			input.read(currWord);
#ifdef DBG_FULL
			std::cout << "\t";
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			output.write(currWord);
			if (!metaWritten)
			{
				exhMetaFifo.write(exhMeta()); 
				metaOut.write(ExHeader<WIDTH>());
				metaWritten = true;
			}
			if (currWord.last)
			{
				state = META;
			}
		}
		break;
	}//switch
}
