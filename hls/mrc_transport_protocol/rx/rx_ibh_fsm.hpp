#pragma once

#include "../mrc_headers.hpp"
#include "../tables/state_table.hpp"

template <int INSTID = 0>
void rx_ibh_fsm(
#ifdef DBG_IBV
    stream<psnPkg>&	m_axis_dbg,
#endif
	stream<ibhMeta>& metaIn, 					// contains op_code, partition_key, dest_qp, psn, validPSN, numPkg, ackReq, rtx, ts, msnInc, cack
	stream<ap_uint<16> >& udpLenIn,
	stream<exhMeta>& exhMetaFifo,				// only numPkg, can be removed
	stream<rxStateRsp>& stateTable_rsp,			// contains epsn, oldest_outstanding_psn (is it useful?), max_forward (is it useful?), retryCounter, bitmap, lastmap, prevAr
	
	stream<rxStateReq>& stateTable_upd_req,		// contains qpn, epsn, retryCounter, isResponse, write, bitmap, lastmap, prevAr
	stream<ibhMeta>& metaOut,					// contains op_code, partition_key, dest_qp, psn, validPSN, numPkg, ackReq, rtx, ts, msnInc, cack
	stream<ackEvent>& ibhEventFifo,				// contains qpn, psn, validPsn, isNak
	stream<bool>& ibhDropFifo,
	stream<fwdPolicy>& ibhDropMetaFifo,			// contains isDrop, ackOnly
	//stream<ackMeta>& m_axis_rx_ack_meta,
	//stream<ap_uint<16> >& stateTableInv,		// qpn of every QP set up again: its cached state is stale
	ap_uint<32>& regInvalidPsnDropCount
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType{LOAD, PROCESS};
	static fsmStateType fsmState = LOAD;

	static ibhMeta meta;
	static exhMeta emeta;
	static bool isResponse;
	static ap_uint<32> droppedPackets = 0;
	rxStateRsp qpState;

	// trying to implement a cache, probably I should have tested the latency before
	//static bool        cacheValid = false; 
	//static ap_uint<16> cacheQp;
	//static rxStateRsp  cacheState;
	static bool        hit;

	switch(fsmState)
	{
	case LOAD:
		//if (!stateTableInv.empty())
		//{
		//	if (stateTableInv.read() == cacheQp)
		//		cacheValid = false;
		//}
		//else if (!metaIn.empty() && !exhMetaFifo.empty())
		if (!metaIn.empty() && !exhMetaFifo.empty())
		{
			metaIn.read(meta);
			exhMetaFifo.read(emeta);
			isResponse = checkIfResponse(meta.op_code);
			//hit = cacheValid && cacheQp == meta.dest_qp(15, 0);
			//if (!hit)
				stateTable_upd_req.write(rxStateReq(meta.dest_qp, isResponse));
			fsmState = PROCESS;
		}
		break;
	case PROCESS:
	{
		if (/*hit || */!stateTable_rsp.empty() && !udpLenIn.empty())
		{
			//if (hit) 
			//	qpState = cacheState;
			//else 
				stateTable_rsp.read(qpState);

			ap_uint<16> udpLen = udpLenIn.read();
			     

			ap_uint<24> d = meta.psn - qpState.epsn; // distance from epsn
			bool accepted    = d < MRC_MPR && !qpState.bitmap[d];
			bool outOfWindow = d >= MRC_MPR && d <= 0x800000;

			std::cout << std::hex << "[RX IBH FSM " << INSTID << "]: epsn " << qpState.epsn
						<< ", psn " << meta.psn << ", bitmap " << qpState.bitmap << std::dec << std::endl;
#ifdef DBG_IBV
			m_axis_dbg.write(psnPkg(meta.op_code, meta.psn, qpState.epsn, 0));
#endif

			// UDP + BTH + METH + TSETH? + RETH + ImmDt? + ICRC
			ap_uint<16> hdrLen = 8 + 12 + 4 + (meta.ts ? 4 : 0) + 16 + (checkIfImmDt(meta.op_code) ? 4 : 0) + 4;
			// isn’t meant to count real wire bytes
			// it’s an accounting value shared with the requestor’s congestion control, and what matters is that both ends compute exactly the same number per packet
			qpState.rcvdBytes      += udpLen + 40; // nominal_pktsize (8.3.1)
			qpState.sackTriggerCnt += (udpLen > hdrLen + 1024) ? (ap_uint<16>)(udpLen - hdrLen) : (ap_uint<16>)1024; // max(payload, MIN_ACK_PACKET_SIZE), at least 1024

			if (accepted) // packet is inside the window
			{
				ibhDropFifo.write(false);
				ibhDropMetaFifo.write(fwdPolicy(false, false));

				if (d >= (ap_uint<24>)(qpState.maxRcvPsn - qpState.epsn + 1))
					qpState.maxRcvPsn = meta.psn;

				qpState.bitmap[d(MRC_MPR_BITS - 1, 0)]  = 1;
				qpState.lastmap[d(MRC_MPR_BITS - 1, 0)] = (meta.op_code == MRC_RDMA_WRITE_LAST     || meta.op_code == MRC_RDMA_WRITE_ONLY ||
											meta.op_code == MRC_RDMA_WRITE_LAST_IMM || meta.op_code == MRC_RDMA_WRITE_ONLY_IMM);
				qpState.immmap[d(MRC_MPR_BITS - 1, 0)]  = checkIfImmDt(meta.op_code);

				ap_uint<MRC_MPR> mask = qpState.bitmap & ~(qpState.bitmap + 1); // bits below the lowest hole
				ap_uint<MRC_MPR_BITS + 1> n = 0;
				meta.msnInc = 0;
				meta.immInc = 0;
				meta.oooCount = 0;
				for (int i = 0; i < MRC_MPR; i++)
				{
				#pragma HLS unroll
					n           	+= mask[i];
					meta.msnInc 	+= (mask[i] & qpState.lastmap[i]);
					meta.immInc 	+= (mask[i] & qpState.immmap[i]);
					meta.oooCount 	+= (qpState.bitmap[i] & !mask[i]);
				}

				qpState.epsn    += n;
				qpState.bitmap  >>= n;
				qpState.lastmap >>= n;
				qpState.immmap  >>= n;
				meta.cack = qpState.epsn - 1;

				qpState.prevAr = qpState.prevAr || meta.ackReq;
				meta.ackReq    = meta.ackReq || meta.rtx || (qpState.prevAr && meta.cack == qpState.maxRcvPsn)
				                 || qpState.sackTriggerCnt > MRC_SACK_GEN_THRESHOLD; // also add || n != 0 so that a SACK is generated on every cum_ack increment?
				qpState.prevAr = qpState.prevAr && meta.cack != qpState.maxRcvPsn;

				if ((ap_int<24>)(qpState.lowestUnsackedPsn - meta.cack) < 0) // cack has overtaken the previous lowestUnsackedPsn so update it
					qpState.lowestUnsackedPsn = meta.cack;
				else if (n == 0 && !meta.ackReq && (ap_int<24>)(meta.psn - qpState.lowestUnsackedPsn) < 0) // a lower PSN packet arrived
					qpState.lowestUnsackedPsn = meta.psn;
			}
			else if (!outOfWindow) // duplicate (bit is already set or are behind the window by less than 0x800000)
			{
				std::cout << std::hex << "[RX IBH FSM " << INSTID << "]: duplicate psn " << meta.psn << std::dec << std::endl;
				// drop counter increased
				droppedPackets++;
				regInvalidPsnDropCount = droppedPackets;
				ibhEventFifo.write(ackEvent(meta.dest_qp, qpState.epsn - 1, false));
				// drop payload
				ibhDropFifo.write(true);
				ibhDropMetaFifo.write(fwdPolicy(false, true)); // no drop, yes ack

				meta.cack   = qpState.epsn - 1;
				meta.ackReq = true; // meta is not forwarded: reused as "a SACK is sent"
			}
			else // out of window, silently drop it
			{
				std::cout << std::hex << "[RX IBH FSM " << INSTID << "]: out of window psn " << meta.psn << std::dec << std::endl;
				droppedPackets++;
				regInvalidPsnDropCount = droppedPackets;
				ibhDropFifo.write(true);
				ibhDropMetaFifo.write(fwdPolicy(true, false));
			}

			if (!outOfWindow)
			{
				if (meta.ackReq)
				{
					if ((ap_int<24>)(qpState.maxRcvPsn - qpState.lowestUnsackedPsn) <= 64)
						qpState.lowestUnsackedPsn = ((ap_int<24>)(qpState.maxRcvPsn - 64 - meta.cack) > 0) ?
						                            (ap_uint<24>)(qpState.maxRcvPsn - 64) : meta.cack;
					meta.sackOffset = qpState.lowestUnsackedPsn - meta.cack;
					qpState.lowestUnsackedPsn += 64;
					qpState.sackTriggerCnt = 0;
				}
				if (accepted)
					metaOut.write(meta);
				stateTable_upd_req.write(rxStateReq(meta.dest_qp, qpState)); // probably not the cleanest way to wrap data
			}

			//cacheState = qpState;
			//cacheQp    = meta.dest_qp(15, 0);
			//cacheValid = true;
			fsmState   = LOAD;
		}
		break;
	}
	} // switch
}
