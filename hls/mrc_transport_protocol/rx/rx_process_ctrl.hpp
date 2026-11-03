#pragma once

#include "../mrc_headers.hpp"

// parses the extension header of a control packet
// the header is collected into one raw MRC_CTRL_HDR_SIZE buffer regardless of the opcode
// the meta is emitted as soon as the buffer is full or the packet ends, whichever comes first, so short packets on a narrow bus can never stall the parser
template <int WIDTH, int INSTID = 0>
void rx_process_ctrl(
	stream<net_axis<WIDTH> >&     input,
	stream<mrcCtrlBthMeta>&       bthMetaIn,
	stream<ipUdpMeta>&            udpMetaIn,
	stream<mrcCtrlMeta<WIDTH> >&  metaOut
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, HEADER, DRAIN};
	static fsmStateType state = META;

	static packetHeader<WIDTH, MRC_CTRL_HDR_SIZE> hdr;
	static mrcCtrlBthMeta bth;
	static ipUdpMeta      udp;

	net_axis<WIDTH> currWord;

	switch (state)
	{
	case META:
		if (!bthMetaIn.empty() && !udpMetaIn.empty())
		{
			bthMetaIn.read(bth);
			udpMetaIn.read(udp);
			hdr.clear();
			hdr.setRawHeader(0); // bytes beyond a short header must not carry the previous packet
			state = HEADER;
		}
		break;

	case HEADER:
		if (!input.empty())
		{
			input.read(currWord);
			DBG_WORD("RX PROCESS CTRL IN", currWord);
			hdr.parseWord(currWord.data);

			if (hdr.isReady() || currWord.last)
			{
				mrcCtrlMeta<WIDTH> m;
				m.op_code       = bth.op_code;
				m.partition_key = bth.partition_key;
				m.dest_qp       = bth.dest_qp;
				m.psn           = bth.psn;
				m.ackReq        = bth.ackReq;
				m.rtx           = bth.rtx;
				m.udpLength     = udp.length;
				m.srcAddress    = udp.their_address;
				m.srcPort       = udp.their_port;
				// UDP(8) + BTH(12) + header + ICRC(4)
				m.lengthOk      = (udp.length >= (ap_uint<16>) (8 + 12 + mrcCtrlHeaderBytes(bth.op_code) + 4));
				m.raw           = hdr.getRawHeader();
				metaOut.write(m);

				state = currWord.last ? META : DRAIN;
			}
		}
		break;

	case DRAIN: // anything after the header (there should be nothing) is discarded
		if (!input.empty())
		{
			input.read(currWord);
			if (currWord.last)
				state = META;
		}
		break;
	}
}
