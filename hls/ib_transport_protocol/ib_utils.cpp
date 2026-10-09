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

#include "ib_transport_protocol.hpp"

bool checkIfResponse(mrcOpCode code)
{
	return (code == MRC_ACK);
}

// Any MRC request: carries a METH (and a TSETH if BTH.ts)
bool checkIfMrcRequest(mrcOpCode code)
{
	return (code == MRC_RDMA_WRITE_FIRST || code == MRC_RDMA_WRITE_MIDDLE ||
			code == MRC_RDMA_WRITE_LAST  || code == MRC_RDMA_WRITE_LAST_IMM ||
			code == MRC_RDMA_WRITE_ONLY  || code == MRC_RDMA_WRITE_ONLY_IMM);
}

bool checkIfWrite(mrcOpCode code)
{
	return (code == MRC_RDMA_WRITE_ONLY  	|| code == MRC_RDMA_WRITE_FIRST ||
			code == MRC_RDMA_WRITE_MIDDLE	|| code == MRC_RDMA_WRITE_LAST 	||
			code == MRC_RDMA_WRITE_LAST_IMM || code == MRC_RDMA_WRITE_ONLY_IMM);
}

bool checkIfAethHeader(mrcOpCode code)
{
	return (code == MRC_ACK);
}

bool checkIfRethHeader(mrcOpCode code)
{
	return (code == MRC_RDMA_WRITE_ONLY  	|| code == MRC_RDMA_WRITE_FIRST ||
			code == MRC_RDMA_WRITE_MIDDLE	|| code == MRC_RDMA_WRITE_LAST 	||
			code == MRC_RDMA_WRITE_LAST_IMM || code == MRC_RDMA_WRITE_ONLY_IMM);
}

// MRC: every request (write) packet carries a METH after the BTH
bool checkIfMethHeader(mrcOpCode code)
{
	return checkIfWrite(code);
}

// MRC control packets: header-only, routed to the control path by rx_process_ibh
bool checkIfMrcCtrl(mrcOpCode code)
{
	return (code == MRC_SACK || code == MRC_NACK || code == MRC_PROBE_REQ ||
			code == MRC_EP_REQ || code == MRC_EP_RSP);
}

// Write with Immediate: an ImmDt follows the RETH
bool checkIfImmDt(mrcOpCode code)
{
	return (code == MRC_RDMA_WRITE_LAST_IMM || code == MRC_RDMA_WRITE_ONLY_IMM);
}