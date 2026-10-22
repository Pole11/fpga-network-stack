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

#include "../../axi_utils.hpp"
#include "../mrc_transport_protocol.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS

#define MRC_MPR 128

// take a look at section 7.5, table 7-5
struct stateTableEntry
{
	//window
	ap_uint<24> resp_epsn;
	ap_uint<24> resp_old_outstanding;

	// bit i representes resp_epsn + 1
	ap_uint<MRC_MPR> resp_bitmap; // packets arrived
	ap_uint<MRC_MPR> resp_lastmap; // the packet is the last of its message (LAST or ONLY), used for the MSN inside the ACK (means how many messages are completed)
	//ap_uint<MRC_MPR> resp_ackmap; // the packet had BTH.A = 1, used for the delayed ACK
	bool resp_prev_ar; // an AR arrived and the bitmap has not been empty since (there are still holes)
	
	ap_uint<24> req_next_psn;
	ap_uint<24> req_old_unack;
	ap_uint<24> req_old_valid; //required? can be computed?

	ap_uint<3>	retryCounter;   // responder: RC NAK logic in rx_ibh_fsm
	// requestor reliability state, written only by rx_req_fsm (and qp_interface at setup)
	ap_uint<3>  req_retries;    // NACK retries since req_old_unack last advanced
	bool        req_error;      // QP in ERROR on the requestor side (Class B)
};

struct ifStateReq
{
	ap_uint<16> qpn;
	qpState		newState;
	ap_uint<24> remote_psn;
	ap_uint<24> local_psn;
	bool 		selective_update_remote_psn;
	bool		write;
	ifStateReq() {}
	ifStateReq(ap_uint<24> qpn)
		:qpn(qpn), write(false) {}
	ifStateReq(ap_uint<16> qpn, qpState s, ap_uint<24> rpsn, ap_uint<24> lpsn)
		:qpn(qpn), newState(s), remote_psn(rpsn), local_psn(lpsn), selective_update_remote_psn(false), write(true) {}
	ifStateReq(ap_uint<16> qpn, ap_uint<24> rpsn, bool selective_update_rpsn)
		:qpn(qpn), remote_psn(rpsn), selective_update_remote_psn(selective_update_rpsn), write(true) {}
};

struct rxStateReq
{
	ap_uint<16> qpn;
	ap_uint<24> epsn;
	ap_uint<3>	retryCounter;
	bool		isResponse;
	bool		write;
	ap_uint<MRC_MPR> bitmap;
	ap_uint<MRC_MPR> lastmap;
	//ap_uint<MRC_MPR> ackmap;
	bool prevAr;

	rxStateReq() :isResponse(false), write(false) {}
	rxStateReq(ap_uint<24> qpn, bool isRsp)
		:qpn(qpn), isResponse(isRsp), write(false) {}
	//rxStateReq(ap_uint<16> qpn, ap_uint<24> psn, ap_uint<MRC_MPR> bitmap, ap_uint<MRC_MPR> lastmap, ap_uint<MRC_MPR> ackmap, bool isRsp)
	//	:qpn(qpn), epsn(psn), retryCounter(0x7), isResponse(isRsp), write(true), bitmap(bitmap), lastmap(lastmap), ackmap(ackmap) {}
	rxStateReq(ap_uint<16> qpn, ap_uint<24> psn, ap_uint<MRC_MPR> bitmap, ap_uint<MRC_MPR> lastmap, bool prevAr, bool isRsp)
		:qpn(qpn), epsn(psn), retryCounter(0x7), isResponse(isRsp), write(true), bitmap(bitmap), lastmap(lastmap), prevAr(prevAr) {}
};

// ------------------------------------------------------------------------------------------------
// Requestor reliability state: rx_req_fsm (MRC control path) <-> state_table
//
// rx_req_fsm is the ONLY writer (qp_interface only initialises it).
//   unack_psn    = oldest unacknowledged PSN = last cumulatively acknowledged PSN + 1
//                  (left edge of the retransmission window, Table 7-4), stored in req_old_unack.
//                  Advanced by the cack_psn of SACKs and transport ACKs.
//   max_psn_sent = req_next_psn - 1 (req_next_psn is written by TX)
//   retries      = NACK retries since unack_psn last advanced (Table 7-3)
//   error        = QP in ERROR on the requestor side (Class B, Table 6-12)
// ------------------------------------------------------------------------------------------------
struct reqStateReq
{
	ap_uint<16> qpn;
	ap_uint<24> unack_psn;
	ap_uint<3>  retries;
	bool        error;
	bool        write;
	reqStateReq() : write(false) {}
	reqStateReq(ap_uint<16> qpn)
		: qpn(qpn), unack_psn(0), retries(0), error(false), write(false) {}
	reqStateReq(ap_uint<16> qpn, ap_uint<24> unack, ap_uint<3> retries, bool error)
		: qpn(qpn), unack_psn(unack), retries(retries), error(error), write(true) {}
};

struct reqStateRsp
{
	ap_uint<24> unack_psn;
	ap_uint<24> max_psn_sent;
	ap_uint<3>  retries;
	bool        error;
	reqStateRsp() {}
	reqStateRsp(ap_uint<24> unack, ap_uint<24> maxSent, ap_uint<3> retries, bool error)
		: unack_psn(unack), max_psn_sent(maxSent), retries(retries), error(error) {}
};

struct rxStateRsp
{
	//window
	ap_uint<24> epsn;
	ap_uint<24> oldest_outstanding_psn;
	ap_uint<24> max_forward; //used for reponses, page 346

	ap_uint<3>	retryCounter;

	ap_uint<MRC_MPR> bitmap;
	ap_uint<MRC_MPR> lastmap;
	//ap_uint<MRC_MPR> ackmap;
	bool prevAr;

	rxStateRsp() {}
	rxStateRsp(ap_uint<24> epsn, ap_uint<24> old)
		:epsn(epsn), oldest_outstanding_psn(old), max_forward(0), retryCounter(0) {}
	rxStateRsp(ap_uint<24> epsn, ap_uint<24> old, ap_uint<24> maxf)
		:epsn(epsn), oldest_outstanding_psn(old), max_forward(maxf), retryCounter(0) {}
	rxStateRsp(ap_uint<24> epsn, ap_uint<24> old, ap_uint<24> maxf, ap_uint<3> rc)
		:epsn(epsn), oldest_outstanding_psn(old), max_forward(maxf), retryCounter(rc) {}
};

struct txStateReq
{
	ap_uint<16> qpn;
	ap_uint<24> psn;
	bool		write;
	txStateReq() :write(false) {}
	txStateReq(ap_uint<24> qpn)
		:qpn(qpn), write(false) {}
	txStateReq(ap_uint<16> qpn, ap_uint<24> psn)
		:qpn(qpn), psn(psn), write(true) {}
};

template <int INSTID>
void state_table(	hls::stream<rxStateReq>& rxIbh2stateTable_upd_req,
					hls::stream<reqStateReq>& rxCtrl2stateTable_upd_req,
					hls::stream<txStateReq>& txIbh2stateTable_upd_req,
					hls::stream<ifStateReq>& qpi2stateTable_upd_req,
					hls::stream<rxStateRsp>& stateTable2rxIbh_rsp,
					hls::stream<reqStateRsp>& stateTable2rxCtrl_rsp,
					hls::stream<stateTableEntry>& stateTable2txIbh_rsp,
					hls::stream<stateTableEntry>& stateTable2qpi_rsp);


template <int INSTID = 0>
void state_table(		
	hls::stream<rxStateReq>& rxIbh2stateTable_upd_req,
	hls::stream<reqStateReq>& rxCtrl2stateTable_upd_req,
	hls::stream<txStateReq>& txIbh2stateTable_upd_req,
	hls::stream<ifStateReq>& qpi2stateTable_upd_req,
	hls::stream<rxStateRsp>& stateTable2rxIbh_rsp,
	hls::stream<reqStateRsp>& stateTable2rxCtrl_rsp,
	hls::stream<stateTableEntry>& stateTable2txIbh_rsp,
	hls::stream<stateTableEntry>& stateTable2qpi_rsp
	//hls::stream<ap_uint<16> >& stateTable2rxIbh_inv // invalidate cache line
) {
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	static stateTableEntry state_table[MAX_QPS];
#if defined( __VITIS_HLS__)
	#pragma HLS bind_storage variable=state_table type=RAM_2P impl=BRAM
#else
	#pragma HLS RESOURCE variable=state_table core=RAM_2P_BRAM
#endif

	rxStateReq rxRequest;
	reqStateReq ctrlRequest;
	txStateReq txRequest;
	ifStateReq ifRequest;

	if (!rxIbh2stateTable_upd_req.empty())
	{
		rxIbh2stateTable_upd_req.read(rxRequest);
		// Since step B rx_ibh_fsm only handles requests: the isResponse branches below are
		// unreachable (cleanup)
		// todo cleanup !!! not all, but just the isResponse branches
		if (rxRequest.write)
		{
			if (rxRequest.isResponse)
			{
				state_table[rxRequest.qpn].req_old_unack = rxRequest.epsn;
			}
			else
			{
				state_table[rxRequest.qpn].resp_epsn = rxRequest.epsn;
				state_table[rxRequest.qpn].retryCounter = rxRequest.retryCounter;
				//state_table[rxRequest.qpn].sendNAK = rxRequest.epsn;
				state_table[rxRequest.qpn].resp_bitmap  = rxRequest.bitmap;
				state_table[rxRequest.qpn].resp_lastmap = rxRequest.lastmap;
				//state_table[rxRequest.qpn].resp_ackmap  = rxRequest.ackmap;
				state_table[rxRequest.qpn].resp_prev_ar = rxRequest.prevAr;
			}
		}
		else
		{
			stateTableEntry entry = state_table[rxRequest.qpn(15,0)];
			if (rxRequest.isResponse)
			{
				stateTable2rxIbh_rsp.write(rxStateRsp(entry.req_old_unack, entry.req_old_valid, entry.req_next_psn-1));
			}
			else
			{
				// here there is the mapping of resp_old_outstanding and oldest_outstanding_psn
				rxStateRsp rsp(entry.resp_epsn, entry.resp_old_outstanding, entry.resp_epsn, entry.retryCounter);
				rsp.bitmap  = entry.resp_bitmap;
				rsp.lastmap = entry.resp_lastmap;
				//rsp.ackmap  = entry.resp_ackmap;
				rsp.prevAr = entry.resp_prev_ar;
				stateTable2rxIbh_rsp.write(rsp);
			}
		}
	}
	else if (!rxCtrl2stateTable_upd_req.empty())
	{
		rxCtrl2stateTable_upd_req.read(ctrlRequest);
		if (ctrlRequest.write)
		{
			state_table[ctrlRequest.qpn].req_old_unack = ctrlRequest.unack_psn;
			state_table[ctrlRequest.qpn].req_retries   = ctrlRequest.retries;
			state_table[ctrlRequest.qpn].req_error     = ctrlRequest.error;
		}
		else
		{
			stateTableEntry entry = state_table[ctrlRequest.qpn];
			stateTable2rxCtrl_rsp.write(reqStateRsp(entry.req_old_unack, entry.req_next_psn - 1,
			                                        entry.req_retries, entry.req_error));
		}
	}
	else if (!txIbh2stateTable_upd_req.empty())
	{
		txIbh2stateTable_upd_req.read(txRequest);
		if (txRequest.write)
		{
			state_table[txRequest.qpn].req_next_psn = txRequest.psn;
		}
		else
		{
			stateTable2txIbh_rsp.write(state_table[txRequest.qpn]);
		}
	}
	else if (!qpi2stateTable_upd_req.empty())
	{
		qpi2stateTable_upd_req.read(ifRequest);
		if (ifRequest.write)
		{
			std::cout << std::hex << "[STATE TABLE " << INSTID << "]: setup new connection, psn " << ifRequest.remote_psn << std::endl;
			//state_table[ifRequest.qpn].state = ifRequest.newState;
			//state_table[ifRequest.qpn].prevOpCode = RC_RDMA_WRITE_LAST;
			if(!ifRequest.selective_update_remote_psn) { // ful setup
				state_table[ifRequest.qpn].resp_epsn = ifRequest.local_psn;
				state_table[ifRequest.qpn].resp_old_outstanding = ifRequest.local_psn;
				state_table[ifRequest.qpn].req_next_psn = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_old_unack = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_old_valid = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_retries = 0;
				state_table[ifRequest.qpn].req_error   = false;
				state_table[ifRequest.qpn].retryCounter = 0xF;
				
				state_table[ifRequest.qpn].resp_bitmap  = 0;
				state_table[ifRequest.qpn].resp_lastmap = 0;
				//state_table[ifRequest.qpn].resp_ackmap  = 0;
				state_table[ifRequest.qpn].resp_prev_ar = false;
				//stateTable2rxIbh_inv.write(ifRequest.qpn);
			} else {
				state_table[ifRequest.qpn].req_next_psn = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_old_unack = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_old_valid = ifRequest.remote_psn;
				state_table[ifRequest.qpn].req_retries = 0;
				state_table[ifRequest.qpn].req_error   = false;
			}
			//state_table[ifRequest.qpn].sendNAK = false;

			//state_table[ifRequest.qpn].r_key = ifRequest.r_key;
			//state_table[ifRequest.qpn].virtual_address = ifRequest.virtual_address;
		}
		else
		{
			stateTable2qpi_rsp.write(state_table[ifRequest.qpn(15,0)]);
		}
	}

}
