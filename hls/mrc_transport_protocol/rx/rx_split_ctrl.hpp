#pragma once

#include "../mrc_headers.hpp"

// routes the words of each packet to the data path or the control path
// input starts right after the BTH (output of the BTH shifter)
template <int WIDTH, int INSTID = 0>
void rx_split_ctrl(
	stream<net_axis<WIDTH> >& input,
	stream<bool>&             splitIn,
	stream<net_axis<WIDTH> >& dataOut,
	stream<net_axis<WIDTH> >& ctrlOut
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, FWD};
	static fsmStateType state = META;
	static bool toCtrl = false;

	net_axis<WIDTH> currWord;

	switch (state)
	{
	case META:
		if (!splitIn.empty())
		{
			splitIn.read(toCtrl);
			state = FWD;
		}
		break;
	case FWD:
		if (!input.empty())
		{
			input.read(currWord);
			if (toCtrl)
				ctrlOut.write(currWord);
			else
				dataOut.write(currWord);
			if (currWord.last)
				state = META;
		}
		break;
	}
}
