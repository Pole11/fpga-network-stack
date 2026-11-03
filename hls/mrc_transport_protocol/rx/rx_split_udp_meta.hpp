#pragma once

#include "../mrc_headers.hpp"

// routes the ipUdpMeta of each packet to the data path or the control path
// kept separate from rx_split_ctrl so routing data words never waits for UDP metadata
template <int INSTID = 0>
void rx_split_udp_meta(
	stream<ipUdpMeta>&    input,
	stream<bool>&         splitIn,
	stream<ipUdpMeta>&    dataOut,
	stream<ap_uint<16> >& dataLenOut,
	stream<ipUdpMeta>&    ctrlOut
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	ipUdpMeta meta;
	bool toCtrl;

	if (!input.empty() && !splitIn.empty())
	{
		input.read(meta);
		splitIn.read(toCtrl);
		if (toCtrl)
			ctrlOut.write(meta);
		else
		{
			dataOut.write(meta);
			dataLenOut.write(meta.length);
		}
	} 
}
