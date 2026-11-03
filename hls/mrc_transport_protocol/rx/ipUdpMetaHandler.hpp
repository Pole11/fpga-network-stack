#pragma once

#include "../mrc_headers.hpp"

// UDP meta handler
// is it really usefull in this form?
// TODO maybe all ACKS should be triggered by ibhFSM?? what is the guarantee we should/have to give
// TODO this should become a BRAM, storage type of thing
template <int WIDTH, int INSTID = 0>
void ipUdpMetaHandler(	
	stream<ipUdpMeta>&		input,
	stream<ExHeader<WIDTH> >& exHeaderInput,
	stream<fwdPolicy>&			dropMetaIn,
	//stream<dstTuple>&		output,
	//stream<ap_uint<16> >&	remcrc_lengthFifo,
	stream<ap_uint<16> >&	exh_lengthFifo,
	stream<ExHeader<WIDTH> >& exHeaderOutput
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	ipUdpMeta meta;
	ExHeader<WIDTH> header;
	fwdPolicy policy;

	if (!input.empty() && !exHeaderInput.empty() && !dropMetaIn.empty())
	{
		input.read(meta);
		exHeaderInput.read(header);
		dropMetaIn.read(policy);
		if (!policy.isDrop) //TODO clean this up
		{
			if (!policy.ackOnly)
			{
				//remcrc_lengthFifo.write(meta.length - (8 + 12 + 4)); //UDP + BTH + CRC
				exh_lengthFifo.write(meta.length);
				exHeaderOutput.write(header);
			}
			//output.write(dstTuple(meta.their_address, meta.their_port));
		}
	}
}
