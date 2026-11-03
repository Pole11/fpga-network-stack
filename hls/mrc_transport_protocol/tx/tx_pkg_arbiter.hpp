#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void tx_pkg_arbiter(
	stream<pkgInfo>& tx_pkgInfoFifo,
	stream<net_axis<WIDTH> >& s_axis_mem_read_data,
	stream<net_axis<WIDTH> >& remoteReadData,
	stream<net_axis<WIDTH> >& localReadData,
	stream<net_axis<WIDTH> >& rawPayFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum mrpStateType{IDLE, FWD_AETH, FWD_RETH, FWD_RAW};
	static mrpStateType state = IDLE;
	static ap_uint<8> wordCounter = 0;

	static pkgInfo info;
	net_axis<WIDTH> currWord;

	switch (state)
	{
	case IDLE:
		if (!tx_pkgInfoFifo.empty())
		{
			tx_pkgInfoFifo.read(info);
			wordCounter = 0;

			if (info.type == AETH)
			{
				state = FWD_AETH;
			}
			else if (info.type == RETH)
			{
				state = FWD_RETH;
			}
			else
			{
				state = FWD_RAW;
			}
		}
		break;
	case FWD_AETH:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);

			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			remoteReadData.write(currWord);
		}
		break;
	case FWD_RETH:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);

			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			localReadData.write(currWord);
		}
		break;
	case FWD_RAW:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);
			
			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			rawPayFifo.write(currWord);
		}
		break;
	}//switch

}
