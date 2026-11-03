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
#include "retrans_pointer_table.hpp"
#include "retrans_meta_table.hpp"
#include "process_retransmissions.hpp"
#include "freelist_handler.hpp"

template <int INSTID = 0>
void retransmitter(	
	stream<retransUpdate>&	rx2retrans_upd,
    stream<retransRdInit>&  retrans2rx_init,
	stream<retransmission>& rx2retrans_req,
	stream<retransmission>& timer2retrans_req,
	stream<retransEntry>&	tx2retrans_insertRequest,
	stream<retransEvent>&	retrans2event
) {
//#pragma HLS DATAFLOW
//#pragma HLS INTERFACE ap_ctrl_none register port=return
#pragma HLS INLINE

	static stream<pointerReq>				rt_pointerReqFifo("rt_pointerReqFifo");
	static stream<pointerUpdate>			rt_pointerUpdFifo("rt_pointerUpdFifo");
	static stream<retransPointerEntry> 		rt_pointerRspFifo("rt_pointerRspFifo"); //TODO reorder
	#pragma HLS STREAM depth=2 variable=rt_pointerReqFifo
	#pragma HLS STREAM depth=2 variable=rt_pointerUpdFifo
	#pragma HLS STREAM depth=2 variable=rt_pointerRspFifo

	static stream<retransMetaReq>				rt_metaReqFifo("rt_metaReqFifo");
	static stream<retransMetaEntry>			rt_metaRspFifo("rt_metaRspFifo");
	#pragma HLS STREAM depth=2 variable=rt_metaReqFifo
	#pragma HLS STREAM depth=2 variable=rt_metaRspFifo

	static stream<ap_uint<16> > rt_freeListFifo("rt_freeListFifo");
	#pragma HLS STREAM depth=META_TABLE_SIZE variable=rt_freeListFifo
	static stream<ap_uint<16> > rt_releaseFifo("rt_releaseFifo");
	#pragma HLS STREAM depth=2 variable=rt_releaseFifo

	freelist_handler<INSTID>(rt_releaseFifo, rt_freeListFifo);

	retrans_pointer_table<INSTID>(rt_pointerReqFifo, rt_pointerUpdFifo, rt_pointerRspFifo);

	retrans_meta_table<INSTID>( rt_metaReqFifo,
						rt_metaRspFifo);

	process_retransmissions<INSTID>(
		rx2retrans_upd,
        retrans2rx_init,
		rx2retrans_req,
		timer2retrans_req,
		tx2retrans_insertRequest,
		rt_pointerReqFifo,
		rt_pointerUpdFifo,
		rt_pointerRspFifo,
		rt_metaReqFifo,
		rt_metaRspFifo,
		rt_freeListFifo,
		rt_releaseFifo,
		retrans2event
	);
}
