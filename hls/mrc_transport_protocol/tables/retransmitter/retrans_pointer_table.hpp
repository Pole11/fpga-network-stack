/************************************************
Copyright (c) 2019, Systems Group, ETH Zurich.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************/
#pragma once

#include "retransmitter_types.hpp"

template <int INSTID = 0>
void retrans_pointer_table(	
    stream<pointerReq>&			    pointerReqFifo,
	stream<pointerUpdate>&			pointerUpdFifo,
	stream<retransPointerEntry>& 	pointerRspFifo)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	static retransPointerEntry ptr_table[MAX_QPS];
#if defined( __VITIS_HLS__)
	#pragma HLS bind_storage variable=ptr_table type=RAM_1P impl=LUTRAM
#else
	#pragma HLS RESOURCE variable=ptr_table core=RAM_T2P_BRAM
#endif

	static ap_uint<16> pt_lockedQP;
	static bool pt_isLocked = false;
	static bool pt_wait = false;
	static pointerReq pt_req;

	pointerUpdate upd;

	if (!pointerUpdFifo.empty())
	{
		pointerUpdFifo.read(upd);
		ptr_table[upd.qpn] = upd.entry;
		if (pt_lockedQP == upd.qpn)
		{
			// clear up lock
			pt_isLocked = false;
		}
	}
	else if (!pointerReqFifo.empty() && !pt_wait)
	{
		pointerReqFifo.read(pt_req);
		if (pt_req.lock && pt_isLocked)
		{
			// another lock queing up, waiting for an upd to clear prev lock
			pt_wait = true;
		}
		else
		{
			pointerRspFifo.write(ptr_table[pt_req.qpn]);
			if (pt_req.lock)
			{
				// lock is claim, can only read, cannot write
				pt_isLocked = true;
				pt_lockedQP = pt_req.qpn;
			}
		}
	}
	else if (pt_wait && !pt_isLocked)
	{
		// prev lock cleared
		pointerRspFifo.write(ptr_table[pt_req.qpn]);
		pt_isLocked = true;
		pt_lockedQP = pt_req.qpn;
		pt_wait = false;
	}
}
