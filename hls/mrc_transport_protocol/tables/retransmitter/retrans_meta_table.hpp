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
void retrans_meta_table(stream<retransMetaReq>&		meta_upd_req,
						stream<retransMetaEntry>&		meta_rsp)
{
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	static retransMetaEntry meta_table[META_TABLE_SIZE];
#if defined( __VITIS_HLS__)
	#pragma HLS bind_storage variable=meta_table type=RAM_T2P impl=BRAM
#else
	#pragma HLS RESOURCE variable=meta_table core=RAM_T2P_BRAM
#endif
	#pragma HLS DEPENDENCE variable=meta_table inter false

	retransMetaReq req;

    if (!meta_upd_req.empty())
    {
        meta_upd_req.read(req);
        if (req.write)
        {
            meta_table[req.idx] = req.entry;
        }
        else if (req.append)
        {
            meta_table[req.idx].next = req.entry.next;
            meta_table[req.idx].isTail = false;
        }
        else
        {
            meta_rsp.write(meta_table[req.idx]);
        }
    }
}
