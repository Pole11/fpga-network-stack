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
void process_retransmissions(
	stream<retransUpdate>&	        rx2retrans_upd,
    stream<retransRdInit>&          retrans2rx_init,
	stream<retransmission>&         rx2retrans_req,
	stream<retransmission>&         timer2retrans_req,
	stream<retransEntry>&	        tx2retrans_insertRequest,
	stream<pointerReq>&				pointerReqFifo,
	stream<pointerUpdate>&			pointerUpdFifo,
	stream<retransPointerEntry>& 	pointerRspFifo, //TODO reorder
	stream<retransMetaReq>&			metaReqFifo,
	stream<retransMetaEntry>&		metaRspFifo,
	stream<ap_uint<16> >&			freeListFifo,
	stream<ap_uint<16> >&			releaseFifo,
	stream<retransEvent>&			retrans2event
) {
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	enum retransStateType {MAIN, INSERT_0, INSERT_1, UPDATE_0, UPDATE_1, UPDATE_2, RETRANS_0, RETRANS_1, RETRANS_2, TIMER_RETRANS_0};	static retransStateType rt_state = MAIN;
	static retransUpdate update;
	static ap_uint<16> curr;
	static ap_uint<16> newMetaIdx;
	static retransEntry insert;
	static retransmission retrans;
	static retransMetaEntry meta; //TODO register needed??
	static retransPointerEntry ptrMeta;

	switch (rt_state)
	{
	case MAIN:
		if (!rx2retrans_upd.empty())
		{
			rx2retrans_upd.read(update);
			pointerReqFifo.write(pointerReq(update.qpn, true)); // enquire whether we have previous req for this qpn
			rt_state = UPDATE_0;
			std::cout << std::hex << "[PROCESS RETRANSMISSION " << INSTID << "]: updating " << update.latest_acked_req << std::endl;
		}
		else if (!rx2retrans_req.empty())
		{
			rx2retrans_req.read(retrans);
			std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: RX Retransmit triggered!!" << std::endl;
			pointerReqFifo.write(pointerReq(retrans.qpn)); 		// enquire whether we have previous req for this qpn
			rt_state = RETRANS_0;
		}
		else if (!timer2retrans_req.empty())
		{
			std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: TIMER Retransmit triggered!!" << std::endl;
			timer2retrans_req.read(retrans);
			// Uses always head psn
			pointerReqFifo.write(pointerReq(retrans.qpn)); 		// enquire whether we have previous req for this qpn
			rt_state = TIMER_RETRANS_0;
		}
		else if (!tx2retrans_insertRequest.empty() && !freeListFifo.empty())
		{
			newMetaIdx = freeListFifo.read();					// check whether we still have place to insert into `retrans meta table`
			tx2retrans_insertRequest.read(insert);
			pointerReqFifo.write(pointerReq(insert.qpn, true));	// enquire whether we have previous req for this qpn
			rt_state = INSERT_0;
			std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: inserting new meta, psn " << std::hex << insert.psn << ", occupying pointer " << newMetaIdx << std::endl;
		}
		break;
	case INSERT_0:
		if (!pointerRspFifo.empty())
		{
			pointerRspFifo.read(ptrMeta);
			if (!ptrMeta.valid)
			{
				// First request for this qpn, write into both `Retrans Pointer Table`, `Retrans Meta Table`
				ptrMeta.valid = true;
				ptrMeta.head = newMetaIdx;
				ptrMeta.tail = newMetaIdx;
				metaReqFifo.write(retransMetaReq(newMetaIdx, retransMetaEntry(insert)));
				pointerUpdFifo.write(pointerUpdate(insert.qpn, ptrMeta));
				rt_state = MAIN;
				std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: inserting new entry at qpn " << insert.qpn << ", last " << insert.lst << std::endl;
			}
			else
			{
				// Update the existing "tail" entry in `Retrans Meta Table` with next=newMetaIdx
				metaReqFifo.write(retransMetaReq(ptrMeta.tail, newMetaIdx));
				// Update the tail pointer to the next index in `Retrans Meta Table`
				ptrMeta.tail = newMetaIdx;
				rt_state = INSERT_1;
				std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: appending entry at qpn " << insert.qpn << std::endl;
			}
		}
		break;
	case INSERT_1:
		// Add new entry into `Retrans Meta Table`
		metaReqFifo.write(retransMetaReq(newMetaIdx, retransMetaEntry(insert)));
		// updates `Retrans Pointer Table` with new "tail"
		pointerUpdFifo.write(pointerUpdate(insert.qpn, ptrMeta));
		rt_state = MAIN;
		break;
	case UPDATE_0:
		if (!pointerRspFifo.empty())
		{
            std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: state UPDATE_0 init entry" << std::endl;
			pointerRspFifo.read(ptrMeta);
			if (ptrMeta.valid)
			{
                std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: state UPDATE_0 valid meta entry" << std::endl;
				// Enquire the first uncleared index (head) in `Retrans Meta Table`
				metaReqFifo.write(retransMetaReq(ptrMeta.head));
				curr = ptrMeta.head;
				rt_state = UPDATE_1;
			}
			else
			{
				// this should not happen
				std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: state UPDATE_0 invalid meta entry" << std::endl;
				// Release lock
				pointerUpdFifo.write(pointerUpdate(update.qpn, ptrMeta));
				rt_state = MAIN;
			}
		}
		break;
	case UPDATE_1:
		if (!metaRspFifo.empty())
		{
			metaRspFifo.read(meta);
			rt_state = UPDATE_2;
		}
		break;
	case UPDATE_2:
		// Release every entry with psn <= cack_psn, reporting each one (posted order) as a completion
		if (meta.valid && mrcPsnLE(meta.psn, update.latest_acked_req))
		{
			retrans2rx_init.write(retransRdInit(update.qpn, meta.psn, meta.localAddr, meta.lst, meta.host));
			std::cout << std::hex << "[PROCESS RETRANSMISSION " << INSTID << "]: released psn " << meta.psn
			          << " (cack " << update.latest_acked_req << "), last " << meta.lst << std::dec << std::endl;

			releaseFifo.write(curr);
			ptrMeta.head  = meta.next;
			ptrMeta.valid = !meta.isTail;
			curr          = meta.next;

			if (!meta.isTail)
			{
				// next entry may be covered too: keep the lock and request it
				metaReqFifo.write(retransMetaReq(meta.next));
				rt_state = UPDATE_1;
				break;
			}
		}
		// first entry beyond cack_psn, end of list, or invalid entry: write back and unlock
		pointerUpdFifo.write(pointerUpdate(update.qpn, ptrMeta));
		rt_state = MAIN;
		break;
	case RETRANS_0:
		if (!pointerRspFifo.empty())
		{
			std::cout << "[PROCESS RETRANSMISSION " << INSTID << "]: NAK, retransmitting qpn " << retrans.qpn << std::endl;
			pointerRspFifo.read(ptrMeta);
			rt_state = MAIN;
			if (ptrMeta.valid)
			{
				// Enquire the first uncleared index (head) in `Retrans Meta Table`
				metaReqFifo.write(retransMetaReq(ptrMeta.head));
				curr = ptrMeta.head;
				rt_state = RETRANS_1;
			}
		}
		break;
	case RETRANS_1:
		// walk the list until the requested psn
		if (!metaRspFifo.empty())
		{
			metaRspFifo.read(meta);
			rt_state = MAIN;
			if (meta.valid)
			{
				if (meta.psn == retrans.psn)
				{
					std::cout << std::hex << "[PROCESS RETRANSMISSION " << INSTID << "]: retransmitting psn " << meta.psn
					          << (retrans.single ? " (single)" : "") << std::dec << std::endl;
					retrans2event.write(retransEvent(meta.opCode, retrans.qpn, meta.localAddr, meta.remoteAddr, meta.length, meta.psn, meta.lst, meta.offs));
					if (!retrans.single && !meta.isTail)
					{
						metaReqFifo.write(retransMetaReq(meta.next));
						rt_state = RETRANS_2;   // go-back-N: everything up to the tail
					}
				}
				else if (!meta.isTail)
				{
					metaReqFifo.write(retransMetaReq(meta.next));
					rt_state = RETRANS_1;
				}
				else
				{
					std::cout << std::hex << "[PROCESS RETRANSMISSION " << INSTID << "]: psn " << retrans.psn
					          << " not in the list (already acknowledged), nothing to retransmit" << std::dec << std::endl;
				}
			}
		}
		break;
	case RETRANS_2:
		// Retransmit everything until reach tail
		if (!metaRspFifo.empty())
		{
			metaRspFifo.read(meta);
			rt_state = MAIN;
			if (meta.valid)
			{
				if (!meta.isTail)
				{
					// keep retransmitting
					metaReqFifo.write(retransMetaReq(meta.next));
					rt_state = RETRANS_2;
				}
				std::cout << std::hex << "[PROCESS RETRANSMISSION " << INSTID << "]: retransmitting psn " << meta.psn << std::endl;
				retrans2event.write(retransEvent(meta.opCode, retrans.qpn, meta.localAddr, meta.remoteAddr, meta.length, meta.psn, meta.lst, meta.offs));
			}
		}
		break;
	case TIMER_RETRANS_0:
		if (!pointerRspFifo.empty())
		{
			std::cout << std::hex << "[PROCESS RETRANSMISSION " << INSTID << "]: timed out, retransmitting qpn " << retrans.qpn << std::endl;
			pointerRspFifo.read(ptrMeta);
			if (ptrMeta.valid)
			{
				// Get continuous stream of meta entries
				metaReqFifo.write(retransMetaReq(ptrMeta.head));
				rt_state = RETRANS_2;
			}
			else
			{
				rt_state = MAIN;
			}
		}
		break;
	} // switch
}
