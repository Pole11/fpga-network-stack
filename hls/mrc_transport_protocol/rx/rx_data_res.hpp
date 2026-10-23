// receive WRITEs

#pragma once

#include "../mrc_headers.hpp"
#include "../tables/state_table.hpp"
#include "../tables/msn_table.hpp"

// joins the tail of 'lo' and the head of 'hi', dropping the first 'offset' bytes of 'lo'.
// a word here is 64 bits, 8 bytes
/*
input word 0 (lo):  M0 M1 M2 M3 R0 R1 R2 R3
input word 1 (hi):  R4 R5 R6 R7 R8 R9 Ra Rb

output word 0:      R0 R1 R2 R3 R4 R5 R6 R7
                     lo[4..7]  |  hi[0..3]

The four METH bytes have disappeared, and the RETH now starts at byte 0. That's what rx_process_exh expects.

It is only used if METH (4 Bytes) is present and TSETH not (4 Bytes too)
*/
template <int W>
net_axis<W> mrcJoin(net_axis<W>& lo, net_axis<W>& hi, bool hasTseth)
{
#pragma HLS inline
	net_axis<W> w;
	if (hasTseth)
	{
		w.data = (hi.data(63, 0), lo.data(W-1, 64));
		w.keep = (hi.keep(7, 0),  lo.keep(W/8-1, 8));
	}
	else
	{
		w.data = (hi.data(31, 0), lo.data(W-1, 32));
		w.keep = (hi.keep(3, 0),  lo.keep(W/8-1, 4));
	}
	w.last = 0;
	return w;
}

// RX strip MRC extension headers
// input starts right after the BTH
// for MRC requests: parses METH (+ TSETH if BTH.ts) from the first word and removes those 4/8 bytes, so rx_process_exh sees the RETH at byte 0
// every other packet passes through unchanged
// requires WIDTH >= 64 (METH + TSETH = 8 bytes).
template <int WIDTH, int INSTID = 0>
void rx_strip_mrc_eth(
	stream<net_axis<WIDTH> >& input,
	stream<mrcEthMeta>&       metaIn,
	stream<exhOpMeta>&        metaOut,
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1 

	enum fsmStateType {META, FIRST, BODY, REMAINDER};
	static fsmStateType state = META;
	static mrcOpCode     opCode;
	static bool         hasTseth;
	static ap_uint<4>   offset;      // bytes to strip: 0, 4 or 8
	static net_axis<WIDTH> prevWord;

	mrcEthMeta      inMeta;
	net_axis<WIDTH> currWord;
	net_axis<WIDTH> sendWord;
	net_axis<WIDTH> emptyWord;
	emptyWord.data = 0;
	emptyWord.keep = 0;
	emptyWord.last = 0;

	switch (state)
	{
	case META: // obtain the correct offest (4 or 8)
		if (!metaIn.empty())
		{
			metaIn.read(inMeta);
			opCode   = inMeta.op_code;
			hasTseth = inMeta.req && inMeta.ts; // ts is only meaningful on requests
			offset   = inMeta.req ? (hasTseth ? 8 : 4) : 0;
			state    = FIRST;

			std::cout << "[RX STRIP MRC ETH " << INSTID << "] offset:" << offset << std::endl;
		}
		break;

	case FIRST:
		if (!input.empty())
		{
			input.read(currWord);
			DBG_WORD("STRIP MRC headers IN", currWord);

			exhOpMeta out;
			out.op_code = opCode;
			if (offset != 0)
			{
				// METH: bytes 0-1 RQMSN, bytes 2-3 MSN (big-endian on the wire)
				// METH is always present on a request packet
				// it gets set also if the packet has no header, fix this!
				out.mrc.hasMeth = true; // TODO: somehow bind it to req bit?
				out.mrc.rqmsn = reverse((ap_uint<16>) currWord.data(15, 0));
				out.mrc.msn   = reverse((ap_uint<16>) currWord.data(31, 16));
				if (hasTseth)
				{
					// TSETH: bytes 4-5 tx_timestamp, byte 6 bit 7 = tsr
					out.mrc.hasTseth    = true;
					out.mrc.txTimestamp = reverse((ap_uint<16>) currWord.data(47, 32));
					out.mrc.tsr         = currWord.data[55];
				}
			}
			metaOut.write(out);

			if (offset == 0)                      // not an MRC request: pass through
			{
				output.write(currWord);
				state = currWord.last ? META : BODY;
				// we just read one word, and we must not remove anything, simply forward the packet
			}
			else if (currWord.last)               // whole packet in one word
			{
				sendWord = mrcJoin(currWord, emptyWord, hasTseth);
				sendWord.last = 1;
				output.write(sendWord);
				DBG_WORD("STRIP MRC headers OUT", sendWord);
				state = META;
				// we must remove the first 4 bytes of the METH header
				// but there is no following packet to borrow bits from
				// so add an empty word in the end and drop the first 
				// "offset" Bytes
				// this should never happen because the bus word is like 512, but can change
			}
			else                                  // hold it, the next word completes the output
			{
				prevWord = currWord;
				state = BODY;
				// the next word to borrow bits from hasn't arrived yet
			}
		}
		break;

	case BODY:
		if (!input.empty())
		{
			input.read(currWord);
			DBG_WORD("STRIP MRC headers IN", currWord);
			if (offset == 0)
			{
				// propagate word as is
				output.write(currWord);
				if (currWord.last)
					state = META;
			}
			else
			{
				sendWord = mrcJoin(prevWord, currWord, hasTseth);
				if (currWord.last)
				{
					// bytes of currWord beyond 'offset' don't fit and need one more output word
					// bool remainder = hasTseth 
					// 		? (currWord.keep(WIDTH/8-1, 8) != 0)
                    //       	: (currWord.keep(WIDTH/8-1, 4) != 0);
					// you probably don't need to read all keep values, 1 is enough 
					bool remainder = hasTseth ? currWord.keep[8] : currWord.keep[4];
					sendWord.last = !remainder;
					state = remainder ? REMAINDER : META;
				}
				DBG_WORD("STRIP MRC headers OUT", sendWord);
				output.write(sendWord);
				prevWord = currWord;
			}
		}
		break;

	case REMAINDER:
		sendWord = mrcJoin(prevWord, emptyWord, hasTseth);
		sendWord.last = 1;
		output.write(sendWord);
		DBG_WORD("STRIP MRC headers OUT", sendWord);
		state = META;
		break;
	}
}

template <int WIDTH, int INSTID = 0>
void rx_process_exh( // probably it is useless now ... 
	stream<net_axis<WIDTH> >& input, // packet without BTH, without METH and without TSETH, it starts with RETH
	stream<exhOpMeta>& metaIn,
	stream<exhMeta>& exhMetaFifo, // probably useless ?!
	stream<ExHeader<WIDTH> >& metaOut,
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, RETH_HEADER, NO_HEADER};

	static fsmStateType state = META;

	static RdmaExHeader<WIDTH> rdmaHeader;
	static packetHeader<WIDTH, RETH_SIZE + IMMDT_SIZE> rethImmHeader; // NEW: RETH + ImmDt, *_IMM only

	static bool metaWritten = false;
	static bool rethParsed  = false;
	static bool hasImmDt    = false;

	net_axis<WIDTH> currWord;
	static exhOpMeta opMeta;
	static mrcOpCode opCode;

	switch (state)
	{
	case META:
		if (!metaIn.empty())
		{
			metaIn.read(opMeta);
			opCode = opMeta.op_code;
			metaWritten = false;
			rethParsed  = false;
			hasImmDt    = checkIfImmDt(opCode);

			state = opMeta.mrc.hasMeth ? RETH_HEADER : NO_HEADER; // every MRC request carries a RETH
			std::cout << "[RX PROCESS EXH " << INSTID << "] state:" << state << std::endl;
		}
		break;
	case RETH_HEADER:
		if (!input.empty())
		{
			input.read(currWord);
			rdmaHeader.parseWord(currWord.data);
			rethImmHeader.parseWord(currWord.data);

			if (rethParsed && WIDTH <= RETH_SIZE)
			{
				output.write(currWord);
			}

			if (rdmaHeader.isReady())
			{
				rethParsed = true;
				if (checkIfWrite(opCode) && WIDTH > RETH_SIZE)
				{
					output.write(currWord);
				}
			}

			// non-IMM: same word as before. IMM: wait for the ImmDt (or the end of a malformed packet)
			if (!metaWritten && rdmaHeader.isReady() && (!hasImmDt || rethImmHeader.isReady() || currWord.last))
			{
				exhMetaFifo.write(exhMeta()); 

				ExHeader<WIDTH> exh(rdmaHeader);
				exh.mrc = opMeta.mrc;
				if (hasImmDt)
				{
					// ImmDt: the 4 bytes right after the RETH, big-endian
					// on a malformed IMM packet that ends before the ImmDt is complete, the meta is still written
					exh.mrc.hasImmDt = true;
					exh.mrc.immDt    = reverse((ap_uint<IMMDT_SIZE>) rethImmHeader.header(RETH_SIZE + IMMDT_SIZE - 1, RETH_SIZE));
				}
				metaOut.write(exh);
				metaWritten = true;
			}
			if (currWord.last)
			{
				rdmaHeader.clear();
				rethImmHeader.clear();
				state = META;
			}
		}
		break;
	case NO_HEADER:
		if (!input.empty())
		{
			input.read(currWord);
#ifdef DBG_FULL
			std::cout << "\t";
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			output.write(currWord);
			if (!metaWritten)
			{
				exhMetaFifo.write(exhMeta()); 
				metaOut.write(ExHeader<WIDTH>());
				metaWritten = true;
			}
			if (currWord.last)
			{
				state = META;
			}
		}
		break;
	}//switch
}

template <int INSTID = 0>
void rx_ibh_fsm(
#ifdef DBG_IBV
    stream<psnPkg>&	m_axis_dbg,
#endif
	stream<ibhMeta>& metaIn, 					// contains op_code, partition_key, dest_qp, psn, validPSN, numPkg, ackReq, rtx, ts, msnInc, cack
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
		if (/*hit || */!stateTable_rsp.empty())
		{
			//if (hit) 
			//	qpState = cacheState;
			//else 
				stateTable_rsp.read(qpState);
			     

			ap_uint<24> d = meta.psn - qpState.epsn; // distance from epsn

			std::cout << std::hex << "[RX IBH FSM " << INSTID << "]: epsn " << qpState.epsn
						<< ", psn " << meta.psn << ", bitmap " << qpState.bitmap << std::dec << std::endl;
#ifdef DBG_IBV
			m_axis_dbg.write(psnPkg(meta.op_code, meta.psn, qpState.epsn, 0));
#endif

			if (d < MRC_MPR && !qpState.bitmap[d]) // packet is inside the window
			{
				ibhDropFifo.write(false);
				ibhDropMetaFifo.write(fwdPolicy(false, false));

				qpState.bitmap[d(6, 0)]  = 1;
				qpState.lastmap[d(6, 0)] = (meta.op_code == MRC_RDMA_WRITE_LAST     || meta.op_code == MRC_RDMA_WRITE_ONLY ||
											meta.op_code == MRC_RDMA_WRITE_LAST_IMM || meta.op_code == MRC_RDMA_WRITE_ONLY_IMM);

				ap_uint<MRC_MPR> mask = qpState.bitmap & ~(qpState.bitmap + 1); // bits below the lowest hole
				ap_uint<8> n = 0;
				meta.msnInc = 0;
				for (int i = 0; i < MRC_MPR; i++)
				{
				#pragma HLS unroll
					n           += mask[i];
					meta.msnInc += (mask[i] & qpState.lastmap[i]);
				}

				qpState.prevAr = qpState.prevAr || meta.ackReq;
				meta.ackReq    = qpState.prevAr && qpState.bitmap[0];
				qpState.prevAr = qpState.prevAr && (qpState.bitmap & ~mask) != 0;

				qpState.epsn    += n;
				qpState.bitmap  >>= n;
				qpState.lastmap >>= n;
				meta.cack = qpState.epsn - 1;

				metaOut.write(meta);
				stateTable_upd_req.write(rxStateReq(meta.dest_qp, qpState.epsn, qpState.bitmap, qpState.lastmap, qpState.prevAr, isResponse));
			}
			else if (d < MRC_MPR || d > 0x800000) // duplicate ( bit is already set or are behind the window by less than 0x800000)
			{
				std::cout << std::hex << "[RX IBH FSM " << INSTID << "]: duplicate psn " << meta.psn << std::dec << std::endl;
				// drop counter increased
				droppedPackets++;
				regInvalidPsnDropCount = droppedPackets;
				ibhEventFifo.write(ackEvent(meta.dest_qp, qpState.epsn - 1, false));
				// drop payload
				ibhDropFifo.write(true);
				ibhDropMetaFifo.write(fwdPolicy(false, true)); // no drop, yes ack
			}
			else // out of window, silently drop it
			{
				std::cout << std::hex << "[RX IBH FSM " << INSTID << "]: out of window psn " << meta.psn << std::dec << std::endl;
				droppedPackets++;
				regInvalidPsnDropCount = droppedPackets;
				ibhDropFifo.write(true);
				ibhDropMetaFifo.write(fwdPolicy(true, false));
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

template <int WIDTH, int INSTID = 0>
void rx_exh_fsm(
#ifdef DBG_IBV
    stream<psnPkg>&	m_axis_dbg,
#endif
	stream<ibhMeta>& metaIn,
	stream<ap_uint<16> >& udpLengthFifo,
	stream<dmaState>& msnTable2rxExh_rsp,
	//stream<ap_uint<64> >& rx_readReqAddr_pop_rsp,
	stream<ExHeader<WIDTH> >& headerInput,
	stream<memCmd>& memoryWriteCmd,
	stream<rxMsnReq>& rxExh2msnTable_upd_req,
	//stream<mqPopReq>& rx_readReqAddr_pop_req,
	stream<ackEvent>& rx_exhEventMetaFifo,
	stream<pkgSplit>& rx_pkgSplitTypeFifo,
	stream<pkgShift>& rx_pkgShiftTypeFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum pe_fsmStateType {META, DMA_META, DATA};
	static pe_fsmStateType pe_fsmState = META;
	static ibhMeta meta;
	net_axis<WIDTH> currWord;
	static ExHeader<WIDTH> exHeader;
	static dmaState dmaMeta;
	static ap_uint<16> udpLength;
	ap_uint<32> payLoadLength;
	//static rxReadReqRsp readReqMeta;

	switch (pe_fsmState)
	{
	case META:
		if (!metaIn.empty() && !headerInput.empty())
		{
			metaIn.read(meta);
			headerInput.read(exHeader);

			rxExh2msnTable_upd_req.write(rxMsnReq(meta.dest_qp));   // must stay
			pe_fsmState = DMA_META;
		}
		break;
	case DMA_META:
		if (!msnTable2rxExh_rsp.empty() && !udpLengthFifo.empty())
		{
			msnTable2rxExh_rsp.read(dmaMeta);
			udpLengthFifo.read(udpLength);
			pe_fsmState = DATA;
		}
		break;
	case DATA: // TODO merge with DMA_META
        #ifdef DBG_IBV
            m_axis_dbg.write(psnPkg(meta.op_code, meta.psn, meta.dest_qp, 0));
        #endif

		switch(meta.op_code) {
		case MRC_RDMA_WRITE_FIRST:
		case MRC_RDMA_WRITE_MIDDLE:
		case MRC_RDMA_WRITE_LAST:
		case MRC_RDMA_WRITE_ONLY:
		case MRC_RDMA_WRITE_LAST_IMM:
		case MRC_RDMA_WRITE_ONLY_IMM:
		{
			// [BTH][METH][TSETH?][RETH][ImmDt?][PayLd]. METH/TSETH already stripped; every packet has a RETH
			RdmaExHeader<WIDTH> rdmaHeader = exHeader.getRdmaHeader();
			bool lastPkt = (meta.op_code == MRC_RDMA_WRITE_LAST     || meta.op_code == MRC_RDMA_WRITE_ONLY ||
			                meta.op_code == MRC_RDMA_WRITE_LAST_IMM || meta.op_code == MRC_RDMA_WRITE_ONLY_IMM);
			bool immPkt  = checkIfImmDt(meta.op_code);

			// UDP + BTH + METH/TSETH/ImmDt + RETH + ICRC
			payLoadLength = udpLength - (8 + 12 + exHeader.mrc.length() + 16 + 4);

			// The RETH va is this packet's own address, so no msn-table vaddr tracking is needed
			memoryWriteCmd.write(memCmd(meta.op_code, meta.dest_qp, lastPkt ? PKG_F : PKG_NF,
			                            rdmaHeader.getVirtualAddress(), payLoadLength, PKG_NR, PKG_HOST, 0));
			if (immPkt)
			{
				// Parsed and available: exHeader.mrc.immDt, exHeader.mrc.rqmsn
				// TODO WriteIMM (6.3.3): stash immDt (keyed by rqmsn) until the message completes,
				//      then generate the RQ completion carrying it.
				// TODO more than max_wimm_inflight outstanding -> NAK-Invalid Request (Table 6-15)
			}
			if (meta.msnInc != 0)
				rxExh2msnTable_upd_req.write(rxMsnReq(meta.dest_qp, dmaMeta.msn + meta.msnInc));
			if (meta.ackReq)
				rx_exhEventMetaFifo.write(ackEvent(meta.dest_qp, meta.cack, false)); // todo: don't generate an ACK, but a SACK request one all previous packets arrived
			
			rx_pkgSplitTypeFifo.write(pkgSplit(meta.op_code));
			rx_pkgShiftTypeFifo.write(pkgShift(immPkt ? SHIFT_RETH_IMM : SHIFT_RETH, meta.dest_qp));
			pe_fsmState = META;
			break;
		}
		default:
			break;
		} //switch meta_Opcode
		break;
	} //switch
}

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

template <int WIDTH, int INSTID = 0>
void rx_exh_payload(
	stream<pkgSplit>& metaIn,
	stream<net_axis<WIDTH> >& input,
	stream<net_axis<WIDTH> >& rx_exh2rethShiftFifo,
	stream<net_axis<WIDTH> >& rx_exh2rethImmShiftFifo, 
	stream<net_axis<WIDTH> >& rx_exhNoShiftFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, PKG};
	static fsmStateType rep_state = META;
	static pkgSplit meta;

	net_axis<WIDTH> currWord;

	switch (rep_state)
	{
	case META:
		if (!metaIn.empty())
		{
			metaIn.read(meta);
			rep_state = PKG;
		}
		break;
	case PKG:
		if (!input.empty())
		{
			input.read(currWord);

			if (checkIfImmDt(meta.op_code))          // RETH + ImmDt: 20 bytes
			{
				rx_exh2rethImmShiftFifo.write(currWord);
			}
			else if (checkIfRethHeader(meta.op_code)) // RETH: 16 bytes
			{
				rx_exh2rethShiftFifo.write(currWord);
			}
			else
			{
				rx_exhNoShiftFifo.write(currWord);
			}

			if (currWord.last)
			{
				rep_state = META;
			}
		}
		break;
	} //switch
}

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

	enum mrpStateType{IDLE, FWD_RETH, FWD_RETH_IMM, FWD_NONE};
	static mrpStateType state = IDLE;

	pkgShift shift;
	net_axis<WIDTH> currWord;

	switch (state)
	{
	case IDLE:
		if (!rx_pkgShiftTypeFifo.empty())
		{
			rx_pkgShiftTypeFifo.read(shift);
			if (shift.type == SHIFT_RETH)     		state = FWD_RETH;
			else if (shift.type == SHIFT_RETH_IMM) 	state = FWD_RETH_IMM;
			else                                   	state = FWD_NONE;
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

// UDP meta handler
// TODO maybe all ACKS should be triggered by ibhFSM?? what is the guarantee we should/have to give
// TODO this should become a BRAM, storage type of thing
template <int WIDTH, int INSTID = 0>
void ipUdpMetaHandler(	
	stream<ipUdpMeta>&		input,
	stream<ExHeader<WIDTH> >& exHeaderInput,
	stream<fwdPolicy>&			dropMetaIn,
	//stream<dstTuple>&		output,
	//stream<ap_uint<16> >&	remcrc_lengthFifo,
	stream<ap_uint<16> >&	exh_lengthFifo,
	stream<ExHeader<WIDTH> >& exHeaderOutput
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	ipUdpMeta meta;
	ExHeader<WIDTH> header;
	fwdPolicy policy;
	bool isDrop;

	if (!input.empty() && !exHeaderInput.empty() && !dropMetaIn.empty())
	{
		input.read(meta);
		exHeaderInput.read(header);
		dropMetaIn.read(policy);
		if (!policy.isDrop) //TODO clean this up
		{
			if (!policy.ackOnly)
			{
				//remcrc_lengthFifo.write(meta.length - (8 + 12 + 4)); //UDP + BTH + CRC
				exh_lengthFifo.write(meta.length);
				exHeaderOutput.write(header);

			}
			//output.write(dstTuple(meta.their_address, meta.their_port));
		}
	}
}

