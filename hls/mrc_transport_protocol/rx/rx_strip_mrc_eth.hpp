#pragma once

#include "../mrc_headers.hpp"

// joins the tail of 'lo' and the head of 'hi', dropping the first 'offset' bytes of 'lo'.
// a word here is 64 bits, 8 bytes
/*
input word 0 (lo):  M0 M1 M2 M3 R0 R1 R2 R3
input word 1 (hi):  R4 R5 R6 R7 R8 R9 Ra Rb

output word 0:      R0 R1 R2 R3 R4 R5 R6 R7
                     lo[4..7]  |  hi[0..3]

The four METH bytes have disappeared, and the RETH now starts at byte 0. That's what rx_process_exh expects.

It is only used if METH (4 Bytes) is present and TSETH not (4 Bytes too)
*/
template <int W>
net_axis<W> mrcJoin(net_axis<W>& lo, net_axis<W>& hi, bool hasTseth)
{
#pragma HLS inline
	net_axis<W> w;
	if (hasTseth)
	{
		w.data = (hi.data(63, 0), lo.data(W-1, 64));
		w.keep = (hi.keep(7, 0),  lo.keep(W/8-1, 8));
	}
	else
	{
		w.data = (hi.data(31, 0), lo.data(W-1, 32));
		w.keep = (hi.keep(3, 0),  lo.keep(W/8-1, 4));
	}
	w.last = 0;
	return w;
}

// RX strip MRC extension headers
// input starts right after the BTH
// for MRC requests: parses METH (+ TSETH if BTH.ts) from the first word and removes those 4/8 bytes, so rx_process_exh sees the RETH at byte 0
// every other packet passes through unchanged
// requires WIDTH >= 64 (METH + TSETH = 8 bytes).
template <int WIDTH, int INSTID = 0>
void rx_strip_mrc_eth(
	stream<net_axis<WIDTH> >& input,
	stream<mrcEthMeta>&       metaIn,
	stream<exhOpMeta>&        metaOut,
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1 

	enum fsmStateType {META, FIRST, BODY, REMAINDER};
	static fsmStateType state = META;
	static mrcOpCode     opCode;
	static bool         hasTseth;
	static ap_uint<4>   offset;      // bytes to strip: 0, 4 or 8
	static net_axis<WIDTH> prevWord;

	mrcEthMeta      inMeta;
	net_axis<WIDTH> currWord;
	net_axis<WIDTH> sendWord;
	net_axis<WIDTH> emptyWord;
	emptyWord.data = 0;
	emptyWord.keep = 0;
	emptyWord.last = 0;

	switch (state)
	{
	case META: // obtain the correct offest (4 or 8)
		if (!metaIn.empty())
		{
			metaIn.read(inMeta);
			opCode   = inMeta.op_code;
			hasTseth = inMeta.req && inMeta.ts; // ts is only meaningful on requests
			offset   = inMeta.req ? (hasTseth ? 8 : 4) : 0;
			state    = FIRST;

			std::cout << "[RX STRIP MRC ETH " << INSTID << "] offset:" << offset << std::endl;
		}
		break;

	case FIRST:
		if (!input.empty())
		{
			input.read(currWord);
			DBG_WORD("STRIP MRC headers IN", currWord);

			exhOpMeta out;
			out.op_code = opCode;
			if (offset != 0)
			{
				// METH: bytes 0-1 RQMSN, bytes 2-3 MSN (big-endian on the wire)
				// METH is always present on a request packet
				// it gets set also if the packet has no header, fix this!
				out.mrc.hasMeth = true; // TODO: somehow bind it to req bit?
				out.mrc.rqmsn = reverse((ap_uint<16>) currWord.data(15, 0));
				out.mrc.msn   = reverse((ap_uint<16>) currWord.data(31, 16));
				if (hasTseth)
				{
					// TSETH: bytes 4-5 tx_timestamp, byte 6 bit 7 = tsr
					out.mrc.hasTseth    = true;
					out.mrc.txTimestamp = reverse((ap_uint<16>) currWord.data(47, 32));
					out.mrc.tsr         = currWord.data[55];
				}
			}
			metaOut.write(out);

			if (offset == 0)                      // not an MRC request: pass through
			{
				output.write(currWord);
				state = currWord.last ? META : BODY;
				// we just read one word, and we must not remove anything, simply forward the packet
			}
			else if (currWord.last)               // whole packet in one word
			{
				sendWord = mrcJoin(currWord, emptyWord, hasTseth);
				sendWord.last = 1;
				output.write(sendWord);
				DBG_WORD("STRIP MRC headers OUT", sendWord);
				state = META;
				// we must remove the first 4 bytes of the METH header
				// but there is no following packet to borrow bits from
				// so add an empty word in the end and drop the first 
				// "offset" Bytes
				// this should never happen because the bus word is like 512, but can change
			}
			else                                  // hold it, the next word completes the output
			{
				prevWord = currWord;
				state = BODY;
				// the next word to borrow bits from hasn't arrived yet
			}
		}
		break;

	case BODY:
		if (!input.empty())
		{
			input.read(currWord);
			DBG_WORD("STRIP MRC headers IN", currWord);
			if (offset == 0)
			{
				// propagate word as is
				output.write(currWord);
				if (currWord.last)
					state = META;
			}
			else
			{
				sendWord = mrcJoin(prevWord, currWord, hasTseth);
				if (currWord.last)
				{
					// bytes of currWord beyond 'offset' don't fit and need one more output word
					// bool remainder = hasTseth 
					// 		? (currWord.keep(WIDTH/8-1, 8) != 0)
                    //       	: (currWord.keep(WIDTH/8-1, 4) != 0);
					// you probably don't need to read all keep values, 1 is enough 
					bool remainder = hasTseth ? currWord.keep[8] : currWord.keep[4];
					sendWord.last = !remainder;
					state = remainder ? REMAINDER : META;
				}
				DBG_WORD("STRIP MRC headers OUT", sendWord);
				output.write(sendWord);
				prevWord = currWord;
			}
		}
		break;

	case REMAINDER:
		sendWord = mrcJoin(prevWord, emptyWord, hasTseth);
		sendWord.last = 1;
		output.write(sendWord);
		DBG_WORD("STRIP MRC headers OUT", sendWord);
		state = META;
		break;
	}
}
