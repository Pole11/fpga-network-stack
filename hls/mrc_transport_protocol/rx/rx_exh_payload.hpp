#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void rx_exh_payload(
	stream<pkgSplit>& metaIn,
	stream<net_axis<WIDTH> >& input,
	stream<net_axis<WIDTH> >& rx_exh2rethShiftFifo,
	stream<net_axis<WIDTH> >& rx_exh2rethImmShiftFifo, 
	stream<net_axis<WIDTH> >& rx_exhNoShiftFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, PKG};
	static fsmStateType rep_state = META;
	static pkgSplit meta;

	net_axis<WIDTH> currWord;

	switch (rep_state)
	{
	case META:
		if (!metaIn.empty())
		{
			metaIn.read(meta);
			rep_state = PKG;
		}
		break;
	case PKG:
		if (!input.empty())
		{
			input.read(currWord);

			if (checkIfImmDt(meta.op_code))          // RETH + ImmDt: 20 bytes
			{
				rx_exh2rethImmShiftFifo.write(currWord);
			}
			else if (checkIfRethHeader(meta.op_code)) // RETH: 16 bytes
			{
				rx_exh2rethShiftFifo.write(currWord);
			}
			else
			{
				rx_exhNoShiftFifo.write(currWord);
			}

			if (currWord.last)
			{
				rep_state = META;
			}
		}
		break;
	} //switch
}
