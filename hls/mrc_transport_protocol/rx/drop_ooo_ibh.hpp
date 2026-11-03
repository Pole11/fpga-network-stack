#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void drop_ooo_ibh(	
	stream<net_axis<WIDTH> >& input,
	stream<bool>& metaIn,
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmType {META, FWD, DROP};
	static fsmType state = META;
    static ap_uint<32> cntPacketsFwd = 0;
    static ap_uint<32> cntPacketsDrop = 0;
    static ap_uint<32> cntBeatsFwd = 0;
    static ap_uint<32> cntBeatsDrop = 0;

	bool drop;
	net_axis<WIDTH> currWord;

	switch (state)
	{
		case META:
			if (!metaIn.empty())
			{
				metaIn.read(drop);
                
				if (drop)
				{
					state = DROP;
				}
				else
				{
					state = FWD;
				}
			}
			break;
		case FWD:
			if (!input.empty())
			{
				input.read(currWord);
				output.write(currWord);
                cntBeatsFwd++;
                //m_cnt_dbg_bf = cntBeatsFwd;
				if (currWord.last)
				{
                    cntPacketsFwd++;
                    //m_cnt_dbg_pf = cntPacketsFwd;
					state = META;
				}
			}
			break;
		case DROP:
			if (!input.empty())
			{
				input.read(currWord);
                cntBeatsDrop++;
                //m_cnt_dbg_bd = cntBeatsDrop;
				if (currWord.last)
				{
                    cntPacketsDrop++;
                    //m_cnt_dbg_pd = cntPacketsDrop;
					state = META;
				}
			}
			break;
	} //switch
}
