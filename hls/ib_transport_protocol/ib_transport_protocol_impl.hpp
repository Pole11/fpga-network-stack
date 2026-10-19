#pragma once

#include <rocev2_config.hpp>
#include "conn_table.hpp"
#include "state_table.hpp"
#include "msn_table.hpp"
#include "transport_timer.hpp"
#include "retransmitter/retransmitter.hpp"
#include "read_req_table.hpp"
#include "multi_queue/multi_queue.hpp"
#include "mrc_headers.hpp"
#include "mrc_rx_ctrl.hpp"
#include "mrc_rx_req.hpp"

// ------------------------------------------------------------------------------------------------
// RX path
// ------------------------------------------------------------------------------------------------

/** 
 * RX process IBH
 */
template <int WIDTH, int INSTID = 0>
void rx_process_ibh(	
#ifdef DBG_IBV
	stream<psnPkg>& m_axis_dbg,
#endif
	stream<net_axis<WIDTH> >& input,
	stream<ibhMeta>& metaToIbhFsmOut,              	// data path only (requests)
	stream<mrcEthMeta>& metaToStripCtrlOut,        	// data path only, you only need to know if it has METH and TSETH
	stream<mrcCtrlBthMeta>& metaToProcCtrlOut,   	// control path only, you must have all the possible information of BTH needed for ctrl
	stream<bool>& isCtrlDataOut,            // every packet, true = control
	stream<bool>& isCtrlUdpOut,             // every packet, true = control
	stream<net_axis<WIDTH> >& forwardInputOut,
	ap_uint<32>&		regIbvCountRx
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	static BaseTransportHeader<WIDTH> bth;
	static bool metaWritten = false;
	static ap_uint<32> validRx = 0;

	net_axis<WIDTH> currWord;

	if (!input.empty()) {
		input.read(currWord);
		bth.parseWord(currWord.data);

		if (bth.isReady()) {
			forwardInputOut.write(currWord);
			
			if (!metaWritten) {
				mrcOpCode opCode = bth.getOpCode();
				bool isCtrl = checkIfMrcCtrl(opCode);
				std::cout << "[RX PROCESS IBH " << INSTID << "]: input psn " << std::hex << bth.getPsn() << std::endl;

				isCtrlDataOut.write(isCtrl);
				isCtrlUdpOut.write(isCtrl);
				if (isCtrl)
				{
					// SACK/NACK/PROBE/EP: dqp and psn have an opcode-dependent meaning (Table 7-6),
					// these packets never go through the RC PSN checks of rx_ibh_fsm
					metaToProcCtrlOut.write(mrcCtrlBthMeta(opCode, bth.getPartitionKey(), bth.getDstQP(), bth.getPsn(),
					                                 bth.getAckReq(), bth.getRtx()));
				}
				else
				{
					ibhMeta m(opCode, bth.getPartitionKey(), bth.getDstQP(), bth.getPsn(), true, bth.getAckReq(), bth.getRtx(), bth.getTs());
					metaToIbhFsmOut.write(m);
					metaToStripCtrlOut.write(mrcEthMeta(opCode, bth.getTs()));
				}
				metaWritten = true;

                validRx++;
		        regIbvCountRx = validRx;

				std::cout << "[RX PROCESS IBH " << INSTID << "]: process IBH opcode: " << opCode << (isCtrl ? " (ctrl)" : "") << std::endl;
			}
		}
		
		if (currWord.last) {
			bth.clear();
			metaWritten = false;

#ifdef DBG_IBV
		m_axis_dbg.write(psnPkg(bth.getOpCode(), bth.getPsn(), bth.getDstQP(), currWord.last));
#endif
		}
	}
}

// Joins the tail of 'lo' and the head of 'hi', dropping the first 'offset' bytes of 'lo'.
// A word here is 64 bits, 8 bytes
/*

input word 0 (lo):  M0 M1 M2 M3 R0 R1 R2 R3
input word 1 (hi):  R4 R5 R6 R7 R8 R9 Ra Rb

output word 0:      R0 R1 R2 R3 R4 R5 R6 R7
                     lo[4..7]  |  hi[0..3]

The four METH bytes have disappeared, and the RETH now starts at byte 0. That's what rx_process_exh expects.

It is only used if METH (4 Bytes) is present and TSETH not (4 Bytes too)
*/
template <int W>
net_axis<W> mrcJoin(net_axis<W>& lo, net_axis<W>& hi, ap_uint<4> offset)
{
#pragma HLS inline
	ap_uint<2*W>   data;
	ap_uint<2*W/8> keep;
	data(W-1, 0)       = lo.data;
	data(2*W-1, W)     = hi.data;
	keep(W/8-1, 0)     = lo.keep;
	keep(2*W/8-1, W/8) = hi.keep;
	data >>= (8 * offset);
	keep >>= offset;

	net_axis<W> w;
	w.data = data(W-1, 0);
	w.keep = keep(W/8-1, 0);
	w.last = 0;
	return w;
}

/**
 * RX strip MRC extension headers
 *
 * Input starts right after the BTH. For MRC requests: parses METH (+ TSETH if BTH.ts)
 * from the first word and removes those 4/8 bytes, so rx_process_exh sees the RETH at byte 0.
 * Every other packet passes through unchanged. Requires WIDTH >= 64 (METH + TSETH = 8 bytes).
 */
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
			bool req = checkIfMrcRequest(inMeta.op_code);
			hasTseth = req && inMeta.ts;          // ts is only meaningful on requests
			offset   = req ? (hasTseth ? 8 : 4) : 0;
			state    = FIRST;
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
				sendWord = mrcJoin(currWord, emptyWord, offset);
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
				sendWord = mrcJoin(prevWord, currWord, offset);
				if (currWord.last)
				{
					// bytes of currWord beyond 'offset' don't fit and need one more output word
					bool remainder = ((currWord.keep >> offset) != 0);
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
		sendWord = mrcJoin(prevWord, emptyWord, offset);
		sendWord.last = 1;
		output.write(sendWord);
		DBG_WORD("STRIP MRC headers OUT", sendWord);
		state = META;
		break;
	}
}

/**
 * RX process EXH
 */
template <int WIDTH, int INSTID = 0>
void rx_process_exh(
	stream<net_axis<WIDTH> >& input, // packet without BTH, without METH and without TSETH, it starts with RETH
	stream<exhOpMeta>& metaIn,
	stream<exhMeta>& exhMetaFifo, // the isNak parameter must be removed
	stream<ExHeader<WIDTH> >& metaOut,
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, RETH_HEADER, NO_HEADER};

	static fsmStateType state = META;

	static RdmaExHeader<WIDTH> rdmaHeader;
	static AckExHeader<WIDTH> ackHeader;
	static packetHeader<WIDTH, RETH_SIZE + IMMDT_SIZE> rethImmHeader; // NEW: RETH + ImmDt, *_IMM only

	static bool metaWritten = false;
	static bool rethParsed  = false; // NEW
	static bool hasImmDt    = false; // NEW

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

			if (checkIfRethHeader(opCode))
			{
				state = RETH_HEADER;
			}
			else
			{
				state = NO_HEADER;
			}
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

/**
 * RX IBH fsm
 * 
 * PSN handling page
 * page 298, responser receiving requests
 * page 346, requester receiving responses
 *
 * NAK processing, NAK has to be in-order
 * Responder: - send NAK do not update PSN
 * page 336   - do not send futher ACKs or NAKs until valid PKG is received
 * Requester: - retransmit data
 * page 349   - do not increment epsn
 */
//TODO check if MRC_ACK is a NAK
//TODO validate response is consistent with request
//TODO actually any response in Unack region is valid, not just the next one.
template <int INSTID = 0>
void rx_ibh_fsm(
#ifdef DBG_IBV
    stream<psnPkg>&	m_axis_dbg,
#endif
	stream<ibhMeta>& metaIn, 					// contains op_code, partition_key, dest_qp, psn, validPSN, numPkg, ackReq, rtx, ts, msnInc, cack
	stream<exhMeta>& exhMetaFifo,				// only numPkg, can be removed
	stream<rxStateRsp>& stateTable_rsp,			// contains epsn, oldest_outstanding_psn (is it useful?), max_forward (is it useful?), retryCounter, bitmap, lastmap, ackmap
	
	stream<rxStateReq>& stateTable_upd_req,		// contains qpn, epsn, retryCounter, isResponse, write, bitmap, lastmap, ackmap
	stream<ibhMeta>& metaOut,					// contains op_code, partition_key, dest_qp, psn, validPSN, numPkg, ackReq, rtx, ts, msnInc, cack
	stream<ackEvent>& ibhEventFifo,				// contains qpn, psn, validPsn, isNak
	stream<bool>& ibhDropFifo,
	stream<fwdPolicy>& ibhDropMetaFifo,			// contains isDrop, ackOnly
	//stream<ackMeta>& m_axis_rx_ack_meta,
	ap_uint<32>&		regInvalidPsnDropCount
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
	static bool        cacheValid = false; // todo: clear this flag when setting up new connections
	static ap_uint<16> cacheQp;
	static rxStateRsp  cacheState;

	switch(fsmState)
	{
	case LOAD:
		if (!metaIn.empty() && !exhMetaFifo.empty())
		{
			metaIn.read(meta);
			exhMetaFifo.read(emeta);
			isResponse = checkIfResponse(meta.op_code); // always false since step B: ACKs go to rx_req_fsm			
			// this is a read, you write on the FIFO a request
			// it is a read because it has only two parameters, not three
			if (!(cacheValid && cacheQp == meta.dest_qp(15, 0)))
				stateTable_upd_req.write(rxStateReq(meta.dest_qp, isResponse));
			fsmState = PROCESS;
		}
		break;
		case PROCESS:
		bool hit = cacheValid && cacheQp == meta.dest_qp(15, 0);
		if (hit || !stateTable_rsp.empty())
		{
			if (hit) qpState = cacheState;
			else     stateTable_rsp.read(qpState);

			ap_uint<24> d = meta.psn - qpState.epsn; // distance from epsn

			std::cout << std::hex << "[RX IBH FSM " << INSTID << "]: epsn " << qpState.epsn
			          << ", psn " << meta.psn << ", bitmap " << qpState.bitmap << std::dec << std::endl;
#ifdef DBG_IBV
			m_axis_dbg.write(psnPkg(meta.op_code, meta.psn, qpState.epsn, 0));
#endif

			if (d < MRC_MPR && !qpState.bitmap[d]) // packet is inside the window
			{
				// drop drop, simply forward the payload to memory!
				ibhDropFifo.write(false);
				ibhDropMetaFifo.write(fwdPolicy(false, false));

				// love this:
				qpState.bitmap[d]  = 1; // bitmap track who arrived
				qpState.lastmap[d] = (meta.op_code == MRC_RDMA_WRITE_LAST     || meta.op_code == MRC_RDMA_WRITE_ONLY ||
				                      meta.op_code == MRC_RDMA_WRITE_LAST_IMM || meta.op_code == MRC_RDMA_WRITE_ONLY_IMM); // lastmap tracks who is last/only
				qpState.ackmap[d]  = meta.ackReq; // ackmap tracks who has an ack req

				ap_uint<8> n = MRC_MPR; // n is the index of the lowest zero in bitmap, the lowest hole
				for (int i = MRC_MPR - 1; i >= 0; i--)
				{
#pragma HLS unroll
					if (!qpState.bitmap[i]) n = i;
				}

				// inspect the bits that you are shifting out,
				meta.msnInc = 0;
				meta.ackReq = false;
				for (int i = 0; i < MRC_MPR; i++)
				{
#pragma HLS unroll
					if (i < n && qpState.lastmap[i]) meta.msnInc++; // count how many message ended
					if (i < n && qpState.ackmap[i])  meta.ackReq = true; // set meta.ackReq to true in the case that any of those asked for an ACK
				}

				qpState.epsn    += n;
				qpState.bitmap  >>= n;
				qpState.lastmap >>= n;
				qpState.ackmap  >>= n;
				meta.cack = qpState.epsn - 1;

				metaOut.write(meta);
				stateTable_upd_req.write(rxStateReq(meta.dest_qp, qpState.epsn, qpState.bitmap, qpState.lastmap, qpState.ackmap, isResponse));
			}
			else if (d < MRC_MPR || d > 0x800000) // duplicate ( bit is already set or are behind the window by less than 0x80000000)
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

			cacheState = qpState;
			cacheQp    = meta.dest_qp(15, 0);
			cacheValid = true;
			fsmState   = LOAD;
		}
		break;
	}
}

/**
 * RX EXH fsm
 * 
 * For reliable connections, page 246, 266, 269
 * SEND ONLY: PayLd
 * SEND FIRST: PayLd
 * SEND MIDDLE: PayLd
 * SEND LAST: PayLd
 * RDMA WRITE ONLY: RETH, PayLd
 * RDMA WRITE FIRST: RETH, PayLd
 * RDMA WRITE MIDDLE: PayLd
 * RDMA WRITE LAST: PayLd
 * RDMA READ REQUEST: RETH
 * RDMA READ RESPONSE ONLY: AETH, PayLd
 * RDMA READ RESPONSE FIRST: AETH, PayLd
 * RDMA READ RESPONSE MIDDLE: PayLd
 * RDMA READ RESPONSE LAST: AETH, PayLd
 * ACK: AETH
 */
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
	stream<readRequest>& readRequestFifo,
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

/**
 * Drop out of order
 */
//Currently not used!! ... ehm why? I guess it is
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

/**
 * RX EXH payload
 */
template <int WIDTH, int INSTID = 0>
void rx_exh_payload(
	stream<pkgSplit>& metaIn,
	stream<net_axis<WIDTH> >& input,
	stream<net_axis<WIDTH> >& rx_exh2rethShiftFifo,
	stream<net_axis<WIDTH> >& rx_exh2rethImmShiftFifo, 
	stream<net_axis<WIDTH> >& rx_exh2aethShiftFifo,
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

/**
 * Handling of the read requests
 */
template <int INSTID = 0>
void handle_read_requests(	
	stream<readRequest>& requestIn,
	stream<memCmdInternal>&	memoryReadCmd,
	stream<event>& readEventFifo
) {

}

template <int WIDTH, int INSTID = 0>
void merge_rx_pkgs(	
	stream<pkgShift>& rx_pkgShiftTypeFifo,
	stream<net_axis<WIDTH> >& rx_aethSift2mergerFifo,
	stream<net_axis<WIDTH> >& rx_rethSift2mergerFifo,
	stream<net_axis<WIDTH> >& rx_rethImmSift2mergerFifo,
	stream<net_axis<WIDTH> >& rx_NoSift2mergerFifo,
	stream<net_axis<WIDTH> >& m_axis_mem_write_data
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum mrpStateType{IDLE, FWD_AETH, FWD_RETH, FWD_RETH_IMM, FWD_NONE};
	static mrpStateType state = IDLE;

	pkgShift shift;
	net_axis<WIDTH> currWord;

	switch (state)
	{
	case IDLE:
		if (!rx_pkgShiftTypeFifo.empty())
		{
			rx_pkgShiftTypeFifo.read(shift);
			if (shift.type == SHIFT_AETH)          state = FWD_AETH;
			else if (shift.type == SHIFT_RETH)     state = FWD_RETH;
			else if (shift.type == SHIFT_RETH_IMM) state = FWD_RETH_IMM;
			else                                   state = FWD_NONE;
		}
		break;
	case FWD_AETH:
		if (!rx_aethSift2mergerFifo.empty())
		{
			rx_aethSift2mergerFifo.read(currWord);
			m_axis_mem_write_data.write(currWord);
			if (currWord.last) state = IDLE;
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

// ------------------------------------------------------------------------------------------------
// TX path
// ------------------------------------------------------------------------------------------------

/**
 * Local request handler
 */
template <int INSTID = 0>
void local_req_handler(
	stream<txMeta>& s_axis_sq_meta,
#ifdef RETRANS_EN
	stream<retransEvent>& retransEventFifo,
	stream<retransAddrLen>& tx2retrans_insertAddrLen,
#endif
	stream<memCmdInternal>&	tx_local_memCmdFifo, //TODO rename
	//stream<mqInsertReq<ap_uint<64> > >&	tx_localReadAddrFifo,
	stream<event>&tx_localTxMeta,
    ap_uint<32>& regRetransCount)
{
#pragma HLS inline off
#pragma HLS pipeline II=1

	//enum fsmStateType {META, GENERATE};
	//static fsmStateType lrh_state;
	static txMeta meta;

	event ev;
	retransEvent rev;
    static ap_uint<32> retransCount = 0;

	//switch (lrh_state)
	//{
	//case META:
#ifdef RETRANS_EN
	if (!retransEventFifo.empty())
	{
		retransEventFifo.read(rev);
		tx_localTxMeta.write(event(rev.op_code, rev.qpn, rev.remoteAddr, rev.length, rev.psn));

        retransCount++;
        regRetransCount = retransCount;        

		std::cout << std::dec << "[LOCAL REQ HANDLER " << INSTID << "]: retransmission: length: " << rev.length << ", local addr: " << std::hex << rev.localAddr << ", remote addres: " << rev.remoteAddr << ", psn: " << rev.psn << ", op code: " << rev.op_code << ", qpn: " << rev.qpn << std::endl;

		std::cout << "[LOCAL REQ HANDLER " << INSTID << "]: retranmission writing into memCmd with lengh " << rev.length << std::endl;
		tx_local_memCmdFifo.write(memCmdInternal(rev.op_code, rev.qpn, rev.localAddr, rev.length, PKG_INT, PKG_R, rev.offs));
		

	}
	else if (!s_axis_sq_meta.empty())
#else

	if (!s_axis_sq_meta.empty())
#endif
	{
		s_axis_sq_meta.read(meta); 

		std::cout << std::dec << "[LOCAL REQ HANDLER " << INSTID << "]: opcode: " << meta.op_code << 
			", transmission: length: " << std::dec << meta.len << ", local addr: " << std::hex << meta.laddr << ", remote addres: " << meta.raddr << ", last: " << meta.lst << std::endl;

		if(meta.op_code == MRC_RDMA_WRITE_MIDDLE || meta.op_code == MRC_RDMA_WRITE_FIRST ||
			meta.op_code == MRC_RDMA_WRITE_LAST || meta.op_code == MRC_RDMA_WRITE_ONLY)
		{
			tx_localTxMeta.write(event(meta.op_code, meta.qpn, meta.raddr, meta.len));
			tx_local_memCmdFifo.write(memCmdInternal(meta.op_code, meta.qpn, meta.laddr, meta.len, meta.host, PKG_NR, meta.offs));	
		}
#ifdef RETRANS_EN
		tx2retrans_insertAddrLen.write(retransAddrLen(meta.laddr, meta.raddr, meta.len, meta.lst, meta.offs, meta.host));
#endif
	}
}

/**
 * Local memory command merger
 */
template <int WIDTH, int INSTID = 0>
void mem_cmd_merger(
	stream<memCmdInternal>& remoteReadRequests,
	stream<memCmdInternal>& localReadRequests,
	stream<memCmd>& out,
	stream<pkgInfo>& pkgInfoFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	memCmdInternal cmd;

	if (!remoteReadRequests.empty())
	{
        std::cout << "[MEM CMD MERGER " << INSTID << "]: reading remote request" << std::endl;
		remoteReadRequests.read(cmd);
	}
	else if (!localReadRequests.empty())
	{
		localReadRequests.read(cmd);

		std::cout << "[MEM CMD MERGER " << INSTID << "]: reading local request, opcode - " << std::hex << cmd.op_code 
            << ", sync - " << cmd.sync << ", offs - " << cmd.offs << ", vaddr - " << cmd.addr << ", len - " << cmd.len << std::endl;

		if(cmd.op_code == MRC_RDMA_WRITE_ONLY)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_F, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RETH, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
		if(cmd.op_code == MRC_RDMA_WRITE_FIRST)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_NF, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RETH, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
		if(cmd.op_code == MRC_RDMA_WRITE_MIDDLE)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_NF, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RAW, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
		if(cmd.op_code == MRC_RDMA_WRITE_LAST)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_F, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RAW, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
	}

}

/**
 * TX pkg arbitration
 */
template <int WIDTH, int INSTID = 0>
void tx_pkg_arbiter(
	stream<pkgInfo>& tx_pkgInfoFifo,
	stream<net_axis<WIDTH> >& s_axis_mem_read_data,
	stream<net_axis<WIDTH> >& remoteReadData,
	stream<net_axis<WIDTH> >& localReadData,
	stream<net_axis<WIDTH> >& rawPayFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum mrpStateType{IDLE, FWD_AETH, FWD_RETH, FWD_RAW};
	static mrpStateType state = IDLE;
	static ap_uint<8> wordCounter = 0;

	static pkgInfo info;
	net_axis<WIDTH> currWord;

	switch (state)
	{
	case IDLE:
		if (!tx_pkgInfoFifo.empty())
		{
			tx_pkgInfoFifo.read(info);
			wordCounter = 0;

			if (info.type == AETH)
			{
				state = FWD_AETH;
			}
			else if (info.type == RETH)
			{
				state = FWD_RETH;
			}
			else
			{
				state = FWD_RAW;
			}
		}
		break;
	case FWD_AETH:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);

			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			remoteReadData.write(currWord);
		}
		break;
	case FWD_RETH:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);

			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			localReadData.write(currWord);
		}
		break;
	case FWD_RAW:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);
			
			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			rawPayFifo.write(currWord);
		}
		break;
	}//switch

}

/**
 * TX meta merger
 * 
 * rx_ackEventFifo MRC_ACK from ibh and exh
 * rx_readEvenFifo READ events from RX side
 * tx_appMetaFifo, retransmission events, WRITEs and READ_REQ only
 */
template <int INSTID = 0>
void meta_merger(	
	stream<ackEvent>&	rx_ackEventFifo,
	stream<event>&		rx_readEvenFifo,
	stream<event>&		tx_appMetaFifo,
	//stream<event>&		timer2exhFifo,
	stream<ap_uint<16> >&	tx_connTable_req,
	stream<ibhMeta>&	tx_ibhMetaFifo,
	stream<event>&		tx_exhMetaFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	ackEvent aev;
	event ev;
	ap_uint<16> key = 0; //TODO hack

	if (!rx_ackEventFifo.empty())
	{
		rx_ackEventFifo.read(aev);

		tx_connTable_req.write(aev.qpn(15, 0));
		// PSN used for read response
		tx_ibhMetaFifo.write(ibhMeta(MRC_ACK, key, aev.qpn, aev.psn, aev.validPsn));
		tx_exhMetaFifo.write(event(aev));
		std::cout << "[META MERGER " << INSTID << "]: reading from ack event with qpn " << aev.qpn << ", psn " << aev.psn << std::endl;
	}
	else if (!rx_readEvenFifo.empty())
	{
		rx_readEvenFifo.read(ev);
		tx_connTable_req.write(ev.qpn(15, 0));
		// PSN used for read response
		tx_ibhMetaFifo.write(ibhMeta(ev.op_code, key, ev.qpn, ev.psn, ev.validPsn));
		tx_exhMetaFifo.write(ev);
		std::cout << "[META MERGER " << INSTID << "]: reading from read event with qpn " << ev.qpn << ", psn " << ev.psn << std::endl;
	}
	else if (!tx_appMetaFifo.empty()) //TODO rename
	{
		tx_appMetaFifo.read(ev);

		ap_uint<22> numPkg = 1;

		tx_connTable_req.write(ev.qpn(15, 0));
		if (ev.validPsn) //retransmit
		{
			tx_ibhMetaFifo.write(ibhMeta(ev.op_code, key, ev.qpn, ev.psn, ev.validPsn));
		}
		else //local
		{
			tx_ibhMetaFifo.write(ibhMeta(ev.op_code, key, ev.qpn, numPkg));
		}
		tx_exhMetaFifo.write(ev);
		std::cout << "[META MERGER " << INSTID << "]: reading from app event with qpn " << ev.qpn << ", psn " << ev.psn << std::endl;
	}
	/*else if (!timer2exhFifo.empty())
	{
		timer2exhFifo.read(ev);

		tx_connTable_req.write(ev.qpn(15, 0));
		// PSN used for retransmission
		tx_ibhMetaFifo.write(ibhMeta(ev.op_code, key, ev.qpn, ev.psn, ev.validPsn));
		tx_exhMetaFifo.write(ev);
	}*/
}

/**
 * Generate IBH
 * 
 * For everything, except READ_RSP, we get PSN from state_table
 */
template <int WIDTH, int INSTID = 0>
void generate_ibh(	
	stream<ibhMeta>& metaIn,
	stream<ap_uint<24> >& dstQpIn,
	stream<stateTableEntry>& stateTable2txIbh_rsp,
	stream<txStateReq>&	txIbh2stateTable_upd_req,
#ifdef RETRANS_EN
	stream<retransMeta>& tx2retrans_insertMeta,
#endif
	stream<BaseTransportHeader<WIDTH> >& tx_ibhHeaderFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {META, GET_PSN};
	static fsmStateType gi_state = META;
	static BaseTransportHeader<WIDTH> header;

	static ibhMeta meta;
	net_axis<WIDTH> currWord;
	stateTableEntry qpState; //TODO what is really required
	ap_uint<24> dstQp;

	switch(gi_state)
	{
	case META:
		if (!metaIn.empty() && !dstQpIn.empty())
		{
			metaIn.read(meta);
			dstQpIn.read(dstQp);
			meta.partition_key = 0xFFFF; //TODO this is hard-coded, where does it come from??
			header.clear();

			header.setOpCode(meta.op_code);
			header.setPartitionKey(meta.partition_key);
			//PSN only valid for READ_RSP, otherwise we get it in state GET_PSN
			header.setPsn(meta.psn);
			header.setDstQP(dstQp); //TODO ist meta.dest_qp required??
			if (meta.validPSN)
			{
				tx_ibhHeaderFifo.write(header);
				std::cout << "[GENERATE IBH " << INSTID << "]: input meta, opcode 0x" << meta.op_code  << " valid psn " << std::hex << meta.psn << std::endl;
			}
			else
			{
				txIbh2stateTable_upd_req.write(txStateReq(meta.dest_qp)); //TODO this is actually our qp
				gi_state = GET_PSN;
			}
		}
		break;
	case GET_PSN:
		if (!stateTable2txIbh_rsp.empty())
		{
			stateTable2txIbh_rsp.read(qpState);
			if (meta.op_code == MRC_ACK)
			{
				header.setPsn(qpState.resp_epsn-1); //TODO -1 necessary??
				std::cout << "[GENERATE IBH " << INSTID << "]: MRC_ACK psn " << std::hex << qpState.resp_epsn-1 << std::endl;
			}
			else
			{
				header.setPsn(qpState.req_next_psn);
				header.setAckReq(true);
				//Update PSN
				ap_uint<24> nextPsn = qpState.req_next_psn + meta.numPkg;
				txIbh2stateTable_upd_req.write(txStateReq(meta.dest_qp, nextPsn));

				//Store Packet descirptor in retransmitter table
#ifdef RETRANS_EN
				tx2retrans_insertMeta.write(retransMeta(meta.dest_qp, qpState.req_next_psn, meta.op_code));
#endif
				std::cout << "[GENERATE IBH " << INSTID << "]: generate header opcode 0x" << std::hex << meta.op_code << " psn " << qpState.req_next_psn << std::endl;
			}
			tx_ibhHeaderFifo.write(header);
			gi_state = META;
		}
		break;
	}
}

/**
 * Generate EXH
 * 
 * Types currently supported: DETH, RETH, AETH, ImmDt, IETH
 *
 * For reliable connections, page 246, 266, 269
 * SEND ONLY: PayLd
 * RDMA WRITE ONLY: RETH, PayLd
 * RDMA WRITE FIRST: RETH, PayLd
 * RDMA WRITE MIDDLE: PayLd
 * RDMA WRITE LAST: PayLd
 * RDMA READ REQUEST: RETH
 * RDMA READ RESPONSE ONLY: AETH, PayLd
 * RDMA READ RESPONSE FIRST: AETH, PayLd
 * RDMA READ RESPONSE MIDDLE: PayLd
 * RDMA READ RESPONSE LAST: AETH, PayLd
 * ACK: AETH
 */
template <int WIDTH, int INSTID = 0>
void generate_exh(	
	stream<event>& metaIn,
	stream<txMsnRsp>& msnTable2txExh_rsp,
	stream<ap_uint<16> >& txExh2msnTable_req,
	//stream<txReadReqUpdate>& tx_readReqTable_upd,
	stream<ap_uint<16> >& lengthFifo,
	stream<txPacketInfo>& packetInfoFifo,
#ifdef RETRANS_EN
	stream<ap_uint<24> >&	txSetTimer_req,
	//stream<retransAddrLen>&		tx2retrans_insertAddrLen,
#endif
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum ge_fsmStateType {META, GET_MSN, PROCESS};
	static ge_fsmStateType ge_state = META;
	static event meta;
	net_axis<WIDTH> sendWord;
	static RdmaExHeader<WIDTH> rdmaHeader;
	static AckExHeader<WIDTH>  ackHeader;
	static bool metaWritten;
	static txMsnRsp msnMeta;
	ap_uint<16> udpLen;
	txPacketInfo info;

	switch(ge_state)
	{
	case META:
		if (!metaIn.empty())
		{
			rdmaHeader.clear();
			ackHeader.clear();

			metaIn.read(meta);
			metaWritten = false;
			//if (meta.op_code == MRC_RDMA_READ_RESP_ONLY || meta.op_code == MRC_RDMA_READ_RESP_FIRST || meta.op_code == MRC_RDMA_READ_RESP_MIDDLE || meta.op_code == MRC_RDMA_READ_RESP_LAST || meta.op_code == MRC_ACK)
			{
				txExh2msnTable_req.write(meta.qpn);
				ge_state = GET_MSN;
			}
			//else
			{
				// Start Timer for RDMW_WRITE_* & RDMA_READ_REQUEST

				//txSetTimer_req.write(meta.qpn);
				//ge_state = PROCESS;
			}
#ifdef RETRANS_EN
			//TODO PART HIST
			if (meta.op_code == MRC_RDMA_WRITE_ONLY || meta.op_code == MRC_RDMA_WRITE_FIRST || meta.op_code == MRC_RDMA_WRITE_MIDDLE || meta.op_code == MRC_RDMA_WRITE_LAST)
			{
				txSetTimer_req.write(meta.qpn);
			}
#endif
		}
		break;
	case GET_MSN:
		if (!msnTable2txExh_rsp.empty())
		{
			msnTable2txExh_rsp.read(msnMeta);
			ge_state = PROCESS;
		}
		break;
	case PROCESS:
		{
			sendWord.last = 0;
			switch(meta.op_code)
			{
			case MRC_RDMA_WRITE_ONLY:
			case MRC_RDMA_WRITE_FIRST:
			{
				// [BTH][RETH][PayLd]
				rdmaHeader.setVirtualAddress(meta.addr);
				rdmaHeader.setLength(meta.length); //TODO Move up??
				rdmaHeader.setRemoteKey(msnMeta.r_key);
				ap_uint<8> remainingLength = rdmaHeader.consumeWord(sendWord.data);
				sendWord.keep = ~0;
				sendWord.last = (remainingLength == 0);
				std::cout << "[GENERATE EXH " << INSTID << "]: RDMA_WRITE_ONLY/FIRST" << std::endl;
#ifdef DBG_FULL
				print(std::cout, sendWord);
				std::cout << std::endl;
#endif
				output.write(sendWord);
				if (remainingLength == 0)
				{
					//TODO
				}
				if (!metaWritten) //TODO we are losing 1 cycle here
				{
					info.isAETH = false;
					info.hasHeader = true;
					info.hasPayload = (meta.length != 0); //TODO should be true
					packetInfoFifo.write(info);

					//BTH: 12, RETH: 16, PayLd: x, ICRC: 4
					ap_uint<32> payloadLen = meta.length;
					if (meta.op_code == MRC_RDMA_WRITE_FIRST)
					{
						payloadLen = PMTU;
					}
					udpLen = 12+16+payloadLen+4; //TODO dma_len can be much larger, for multiple packets we need to split this into multiple packets
					lengthFifo.write(udpLen);
					//Store meta for retransmit
					metaWritten = true;
				}
				break;
			}
			case MRC_RDMA_WRITE_MIDDLE:
			case MRC_RDMA_WRITE_LAST:
				// [BTH][PayLd]
				info.isAETH = false;
				info.hasHeader = false;
				info.hasPayload = (meta.length != 0); //TODO should be true
				packetInfoFifo.write(info);
				//BTH: 12, PayLd: x, ICRC: 4
				udpLen = 12+meta.length+4;
				lengthFifo.write(udpLen);
				//Store meta for retransmit
				ge_state = META;
				break;
			case MRC_ACK:
			{
				// [BTH][AETH]
				//Check if ACK or NAK
				if (!meta.isNak)
				{
					ackHeader.setSyndrome(0x1f);
				}
				else
				{
					//PSN SEQ error
					std::cout << "[GENERATE EXH " << INSTID << "]: psn seq error" << std::endl;
					ackHeader.setSyndrome(0x60);
				}
				std::cout << "[GENERATE EXH " << INSTID << "]: MRC_ACK with qpn " << meta.qpn << ", psn " << meta.psn << std::endl;
				ackHeader.setMsn(msnMeta.msn);
				ackHeader.consumeWord(sendWord.data); //TODO
				{
					info.isAETH = true;
					info.hasHeader = true;
					info.hasPayload = false;
					packetInfoFifo.write(info);
					sendWord.keep(AETH_SIZE/8-1, 0) = 0xFF;
					sendWord.keep(WIDTH/8-1, (AETH_SIZE/8)) = 0;
					sendWord.last = 1;
#ifdef DBG_FULL
					std::cout << "[GENERATE EXH " << INSTID << "]: MRC_ACK ";
					print(std::cout, sendWord);
					std::cout << std::endl;
#endif
					output.write(sendWord);
					//BTH: 12, AETH: 4, ICRC: 4
					lengthFifo.write(12+4+4);
				}
				break;
			}
			default:
				break;
			} //switch
		} //if empty
		if (sendWord.last)
		{
			ge_state = META;
		}
		break;
	}//switch
}

/**
 * Append the payload
 */
template <int WIDTH, int INSTID = 0>
void append_payload(
	stream<txPacketInfo>&	packetInfoFifo,
	stream<net_axis<WIDTH> >&	tx_headerFifo,
	stream<net_axis<WIDTH> >&	tx_aethPayloadFifo,
	stream<net_axis<WIDTH> >&	tx_rethPayloadFifo,
	stream<net_axis<WIDTH> >&	tx_rawPayloadFifo,
	stream<net_axis<WIDTH> >&	tx_packetFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmState {INFO, HEADER, AETH_PAYLOAD, RETH_PAYLOAD, RAW_PAYLOAD};
	static fsmState state = INFO;
	static net_axis<WIDTH> prevWord;
	net_axis<WIDTH> currWord;
	net_axis<WIDTH> sendWord;
	static bool firstPayload = true;

	static txPacketInfo info;

	// TODO align this stuff!!
	switch (state)
	{
	case INFO:
		if (!packetInfoFifo.empty())
		{
			firstPayload = true;
			packetInfoFifo.read(info);

			if (info.hasHeader)
			{
				state = HEADER;
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state HEADER" << std::endl;
			}
			else
			{
				state = RAW_PAYLOAD;
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state RAW_PAYLOAD" << std::endl;
			}
		}
		break;
	case HEADER:
		if (!tx_headerFifo.empty())
		{
			tx_headerFifo.read(prevWord);
			//TODO last is not necessary
			if (!prevWord.last)
			{
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: writing header" << std::endl;
				tx_packetFifo.write(prevWord);
			}
			else //last
			{
				if (!info.hasPayload)
				{
					state = INFO;
					tx_packetFifo.write(prevWord);
					std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state INFO" << std::endl;
				}
				else // hasPayload
				{
					prevWord.last = 0;
					if (info.isAETH)
					{
						state = AETH_PAYLOAD;
						std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state AETH_PAYLOAD" << std::endl;
					}
					else //RETH
					{
						if (WIDTH <= RETH_SIZE)
						{
							tx_packetFifo.write(prevWord);
						}
						state = RETH_PAYLOAD;
						std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving to state RETH_PAYLOAD" << std::endl;
					}
				}
			}
		}
		break;
	case AETH_PAYLOAD:
		if (!tx_aethPayloadFifo.empty())
		{
			tx_aethPayloadFifo.read(currWord);
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD" << INSTID << "]: PAYLOAD WORD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			sendWord = currWord;
			if (firstPayload)
			{
				sendWord.data(31, 0) = prevWord.data(31, 0);
				firstPayload = false;
			}
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD" << INSTID << "]: AETH PAYLOAD" << std::endl;
			print(std::cout, sendWord);
			std::cout << std::endl;
#endif
			tx_packetFifo.write(sendWord);
			if (currWord.last)
			{
				state = INFO;
			}
		}
		break;
	case RETH_PAYLOAD:
		if (!tx_rethPayloadFifo.empty())
		{
			std::cout << "[APPEND PAYLOAD " << INSTID << "]: reading rethPayloadFifo" << std::endl;
			tx_rethPayloadFifo.read(currWord);
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD " << INSTID << "]: PAYLOAD WORD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			sendWord = currWord;
			if (firstPayload && WIDTH > RETH_SIZE)
			{
				sendWord.data(127, 0) = prevWord.data(127, 0);
				firstPayload = false;
			}
#ifdef DBG_FULL
			std::cout << "[APPEND PAYLOAD" << INSTID << "]: RETH PAYLOAD" << std::endl;
			print(std::cout, sendWord);
			std::cout << std::endl;
#endif
			tx_packetFifo.write(sendWord);
			if (currWord.last)
			{
				state = INFO;
				std::cout << "[APPEND PAYLOAD " << INSTID << "]: moving state INFO" << std::endl;
			}
		}
		break;
	case RAW_PAYLOAD:
		if (!tx_rawPayloadFifo.empty())
		{
			tx_rawPayloadFifo.read(currWord);
			tx_packetFifo.write(currWord);
			if (currWord.last)
			{
				state = INFO;
			}
		}
		break;
	}
}

/**
 * Prepend the header
 */
//TODO this introduces 1 cycle for WIDTH > 64
template <int WIDTH, int INSTID = 0>
void prepend_ibh_header(
	stream<BaseTransportHeader<WIDTH> >& tx_ibhHeaderFifo,
	stream<net_axis<WIDTH> >& tx_ibhPayloadFifo,
	stream<net_axis<WIDTH> >& m_axis_tx_data,
    ap_uint<32>&		regIbvCountTx
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum pihStatea {GET_HEADER, HEADER, PARTIAL_HEADER, BODY};
	static pihStatea state = GET_HEADER;
	static BaseTransportHeader<WIDTH> header;
	static ap_uint<WIDTH> headerData;
	net_axis<WIDTH> currWord;
    static ap_uint<32> validTx = 0;

	switch (state)
	{
	case GET_HEADER:
		if (!tx_ibhHeaderFifo.empty())
		{
			tx_ibhHeaderFifo.read(header);
			if (BTH_SIZE >= WIDTH)
			{
				state = HEADER;
				std::cout << "[PREPEND IBH HEADER " << INSTID << "]: going to state HEADER" << std::endl;
			}
			else
			{
				state = PARTIAL_HEADER;
				std::cout << "[PREPEND IBH HEADER " << INSTID << "]: going to state PARTIAL_HEADER" << std::endl;
			}
		}
		break;
	case HEADER:
	{
		ap_uint<8> remainingLength = header.consumeWord(currWord.data);
		if (remainingLength < (WIDTH/8))
		{
			state = PARTIAL_HEADER;
		}
		currWord.keep = ~0;
		currWord.last = 0;
		m_axis_tx_data.write(currWord);

		break;
	}
		break;
	case PARTIAL_HEADER:
		if (!tx_ibhPayloadFifo.empty())
		{
			tx_ibhPayloadFifo.read(currWord);
#ifdef DBG_FULL
			std::cout << "[PREPEND IBH HEADER " << INSTID << "]: IBH PARTIAL PAYLOAD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			header.consumeWord(currWord.data);
			m_axis_tx_data.write(currWord);

#ifdef DBG_FULL
			std::cout << "[PREPEND IBH HEADER " << INSTID << "]: IBH PARTIAL HEADER" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			state = BODY;
			if (currWord.last)
			{
                validTx++;
		        regIbvCountTx = validTx;

				state = GET_HEADER;
			}
		}
		break;
	case BODY:
		if (!tx_ibhPayloadFifo.empty())
		{
			tx_ibhPayloadFifo.read(currWord);
			m_axis_tx_data.write(currWord);

#ifdef DBG_FULL
			std::cout << "[PREPEND IBH HEADER " << INSTID << "]: IBH PAYLOAD WORD" << std::endl;
			print(std::cout, currWord);
			std::cout << std::endl;
#endif
			if (currWord.last)
			{
                validTx++;
		        regIbvCountTx = validTx;
                
				state = GET_HEADER;
			}
		}
		break;
	}
}

// ------------------------------------------------------------------------------------------------
// UDP
// ------------------------------------------------------------------------------------------------

/**
 * UDP meta handler
 */
//TODO maybe all ACKS should be triggered by ibhFSM?? what is the guarantee we should/have to give
//TODO this should become a BRAM, storage type of thing
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

/**
 * UDP meta merger TX
 */
template <int INSTID = 0>
void tx_ipUdpMetaMerger(	
	stream<connTableEntry>& tx_connTable2ibh_rsp,
	stream<ap_uint<16> >&	tx_lengthFifo,
	stream<ipUdpMeta>&		m_axis_tx_meta,
	stream<ap_uint<24> >&	tx_dstQpFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	connTableEntry connMeta;
	ap_uint<16> len;

	if (!tx_connTable2ibh_rsp.empty() && !tx_lengthFifo.empty())
	{
		tx_connTable2ibh_rsp.read(connMeta);
		tx_lengthFifo.read(len);
		std::cout << "[TX IP UDP META MERGER " << INSTID << "]: port " << connMeta.remote_udp_port << std::endl;
		m_axis_tx_meta.write(ipUdpMeta(connMeta.remote_ip_address, RDMA_DEFAULT_PORT, connMeta.remote_udp_port, len));
		tx_dstQpFifo.write(connMeta.remote_qpn);
	}
}

// ------------------------------------------------------------------------------------------------
// QP interface
// ------------------------------------------------------------------------------------------------

/**
 * QP table
 */
template <int INSTID = 0>
void qp_interface(	
	stream<qpContext>& 			contextIn,
	stream<stateTableEntry>&	stateTable2qpi_rsp,
	stream<ifStateReq>&			qpi2stateTable_upd_req,
	stream<ifMsnReq>&			if2msnTable_init
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fstStateType{GET_STATE, UPD_STATE};
	static fstStateType qp_fsmState = GET_STATE;
	static qpContext context;
	stateTableEntry state;

	switch(qp_fsmState)
	{
	case GET_STATE:
		if (!contextIn.empty())
		{
			contextIn.read(context);
			qpi2stateTable_upd_req.write(context.qp_num);
			qp_fsmState = UPD_STATE;
		}
		break;
	case UPD_STATE:
		if (!stateTable2qpi_rsp.empty())
		{
			stateTable2qpi_rsp.read(state);
			//TODO check if valid transition

			// Diversify this interface: Based on the vaddr, call either one of the following: 
			if(context.virtual_address == 0xDEADBEEF) {
				// Update of the rkey: Just call the if2msnTable_init
				if2msnTable_init.write(ifMsnReq(context.qp_num, context.r_key));
			} else if (context.virtual_address == 0xFEEDBEEF) {
				// Update of the remote PSN only: Special call to the qpi2stateTable_upd_req
				qpi2stateTable_upd_req.write(ifStateReq(context.qp_num, context.remote_psn, true));
			} else {
				// Default case: Call both, conventional call 
				qpi2stateTable_upd_req.write(ifStateReq(context.qp_num, context.newState, context.remote_psn, context.local_psn));
				if2msnTable_init.write(ifMsnReq(context.qp_num, context.r_key)); //TODO store virtual address somewhere??
			}

			// qpi2stateTable_upd_req.write(ifStateReq(context.qp_num, context.newState, context.remote_psn, context.local_psn));
			// if2msnTable_init.write(ifMsnReq(context.qp_num, context.r_key)); //TODO store virtual address somewhere??
			qp_fsmState = GET_STATE;
		}
		break;
	}
}
