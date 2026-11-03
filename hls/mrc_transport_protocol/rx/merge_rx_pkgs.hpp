#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void merge_rx_pkgs(	
	stream<pkgShift>& rx_pkgShiftTypeFifo,
	stream<net_axis<WIDTH> >& rx_rethSift2mergerFifo,
	stream<net_axis<WIDTH> >& rx_rethImmSift2mergerFifo,
	stream<net_axis<WIDTH> >& rx_NoSift2mergerFifo,
	stream<net_axis<WIDTH> >& m_axis_mem_write_data
) {
#pragma HLS inline off
#pragma HLS pipeline II=1
	// SHIFT_RETH = 0, SHIFT_NONE = 1, SHIFT_RETH_IMM = 2
	// FWD_RETH = 0,   FWD_NONE = 1,   FWD_RETH_IMM = 2,   IDLE = 3
	enum mrpStateType{FWD_RETH, FWD_NONE, FWD_RETH_IMM, IDLE}; // DO NOT REORDER
	static mrpStateType state = IDLE;

	pkgShift shift;
	net_axis<WIDTH> currWord;

	switch (state)
	{
	case IDLE:
		// if (!rx_pkgShiftTypeFifo.empty())
		// {
		// 	rx_pkgShiftTypeFifo.read(shift);
		// 	if (shift.type == SHIFT_RETH)     		state = FWD_RETH;
		// 	else if (shift.type == SHIFT_RETH_IMM) 	state = FWD_RETH_IMM;
		// 	else                                   	state = FWD_NONE;
		// }
		// break;
		if (!rx_pkgShiftTypeFifo.empty())
		{
			rx_pkgShiftTypeFifo.read(shift);
			state = (mrpStateType) shift.type; // it is alligned with the enum
		}
		break;
	case FWD_RETH:
		if (!rx_rethSift2mergerFifo.empty())
		{
			rx_rethSift2mergerFifo.read(currWord);
			m_axis_mem_write_data.write(currWord);
			if (currWord.last) state = IDLE;
		}
		break;
	case FWD_RETH_IMM:
		if (!rx_rethImmSift2mergerFifo.empty())
		{
			rx_rethImmSift2mergerFifo.read(currWord);
			m_axis_mem_write_data.write(currWord);
			if (currWord.last) state = IDLE;
		}
		break;
	case FWD_NONE:
		if (!rx_NoSift2mergerFifo.empty())
		{
			rx_NoSift2mergerFifo.read(currWord);
			m_axis_mem_write_data.write(currWord);
			if (currWord.last) state = IDLE;
		}
		break;
	}//switch
}
