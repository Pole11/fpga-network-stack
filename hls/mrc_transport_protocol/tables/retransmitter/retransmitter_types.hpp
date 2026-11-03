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

#include "../../../axi_utils.hpp"
#include "../../mrc_transport_protocol.hpp"
using namespace hls;

const uint32_t META_TABLE_SIZE = 2000;

struct retransUpdate
{
	ap_uint<16> qpn;
	ap_uint<24> latest_acked_req; // cack_psn: release every entry with psn <= this
    mrcOpCode op_code;
	retransUpdate() {}
	retransUpdate(ap_uint<16> qpn, ap_uint<24> psn, mrcOpCode op_code)
		:qpn(qpn), latest_acked_req(psn), op_code(op_code) {}
};

// retransmitter -> rx_ack_cpl: one entry released by a transport ACK, in posted order
struct retransRdInit
{
	ap_uint<16> qpn;
	ap_uint<24> psn;
	ap_uint<64> laddr;
	ap_uint<1>  lst;
	ap_uint<1>  host;
	retransRdInit() {}
	retransRdInit(ap_uint<16> qpn, ap_uint<24> psn, ap_uint<64> laddr, ap_uint<1> lst, ap_uint<1> host)
		:qpn(qpn), psn(psn), laddr(laddr), lst(lst), host(host) {}
};

struct retransmission
{
	ap_uint<16> qpn;
	ap_uint<24> psn;
	bool        single; // true: resend only psn (MRC NACK); false: go-back-N from psn (timer)
	retransmission() : single(false) {}
	retransmission(ap_uint<16> qpn)
		:qpn(qpn), psn(0), single(false) {}
	retransmission(ap_uint<16> qpn, ap_uint<24> psn)
		:qpn(qpn), psn(psn), single(false) {}
	retransmission(ap_uint<16> qpn, ap_uint<24> psn, bool single)
		:qpn(qpn), psn(psn), single(single) {}
};

struct retransMeta
{
	ap_uint<16> qpn;
	ap_uint<24> psn;
	mrcOpCode	opCode;
	retransMeta() {}
	retransMeta(ap_uint<16> qpn, ap_uint<24> psn, mrcOpCode op)
		:qpn(qpn), psn(psn), opCode(op){}
};

struct retransAddrLen
{
	ap_uint<64> localAddr;
	ap_uint<64> remoteAddr;
	ap_uint<32> length;
    ap_uint<1>  lst;
    ap_uint<4>  offs;
    ap_uint<1>  host;
	retransAddrLen() {}
	retransAddrLen(ap_uint<64> laddr, ap_uint<64> raddr, ap_uint<32> len, ap_uint<1> lst, ap_uint<4> offs, ap_uint<1> host)
		:localAddr(laddr), remoteAddr(raddr), length(len), lst(lst), offs(offs), host(host) {}
};

struct retransEntry
{
	ap_uint<16> qpn;
	ap_uint<24> psn;
	mrcOpCode	opCode;
	ap_uint<64> localAddr;
	ap_uint<64> remoteAddr;
	ap_uint<32> length;
    ap_uint<1>  lst;
    ap_uint<4>  offs;
    ap_uint<1>  host;
	retransEntry() {}
	retransEntry(ap_uint<16> qpn, ap_uint<24> psn, mrcOpCode op, ap_uint<64> laddr, ap_uint<64> raddr, ap_uint<32> len, ap_uint<1> lst, ap_uint<4> offs, ap_uint<1> host)
		:qpn(qpn), psn(psn), opCode(op), localAddr(laddr), remoteAddr(raddr), length(len), lst(lst), offs(offs), host(host) {}
	retransEntry(retransMeta meta, retransAddrLen addrlen)
		:qpn(meta.qpn), psn(meta.psn), opCode(meta.opCode), localAddr(addrlen.localAddr), remoteAddr(addrlen.remoteAddr), length(addrlen.length), lst(addrlen.lst), offs(addrlen.offs), host(addrlen.host) {}
};

struct retransPointerEntry
{
	ap_uint<16>	head;
	ap_uint<16> tail;
	bool valid;
};

typedef enum {INSERT, RELEASE, RX_RETRANS, TIMER_RETRANS} retransOperation ;

struct pointerMeta
{
	retransOperation	op;
	//ap_uint<16>			qpn;
	ap_uint<24>			psn;
	retransPointerEntry entry;
	pointerMeta() {}
	pointerMeta(retransOperation op, ap_uint<24> psn, retransPointerEntry e)
		:op(op), psn(psn), entry(e) {}
};

struct pointerUpdate
{
	ap_uint<16>	qpn;
	retransPointerEntry entry;
	pointerUpdate() {}
	pointerUpdate(ap_uint<16> qpn, retransPointerEntry e)
		:qpn(qpn), entry(e) {}
};

struct retransMetaEntry
{
	ap_uint<24> psn;
	ap_uint<16> next;
	mrcOpCode	opCode;
	ap_uint<64> localAddr;
	ap_uint<64> remoteAddr;
	ap_uint<32> length;
    ap_uint<1>  lst;
    ap_uint<4>  offs;
    ap_uint<1>  host;
	bool valid;
	bool isTail;
	retransMetaEntry() {}
	retransMetaEntry(retransEntry& e)
		:psn(e.psn), next(0), opCode(e.opCode), localAddr(e.localAddr), remoteAddr(e.remoteAddr), length(e.length), lst(e.lst), offs(e.offs), host(e.host), valid(true), isTail(true) {}
	retransMetaEntry(ap_uint<16> next)
		:next(next) {}
};

struct retransMetaReq
{
	ap_uint<16> idx;
	retransMetaEntry entry;
	bool write;
	bool append;
	retransMetaReq() {}
	retransMetaReq(ap_uint<16> idx)
		:idx(idx), write(false),  append(false) {}
	retransMetaReq(ap_uint<16> idx, ap_uint<16> next)
		:idx(idx), entry(retransMetaEntry(next)), write(false), append(true) {}
	retransMetaReq(ap_uint<16> idx, retransMetaEntry e)
		:idx(idx), entry(e), write(true), append(false) {}
};

struct pointerReq
{
	ap_uint<16>	qpn;
	bool		lock;
	pointerReq() {}
	pointerReq(ap_uint<16> qpn)
		:qpn(qpn), lock(false) {}
	pointerReq(ap_uint<16> qpn, bool l)
		:qpn(qpn), lock(l) {}
};
