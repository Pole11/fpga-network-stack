/************************************************
 * MRC RX requestor path
 *
 * Step B: the transport ACK (AETH, 0xD1) no longer travels on the data path. It is routed by
 * rx_process_ibh to the control path like SACK/NACK, and rx_ctrl_fsm forwards it here.
 *
 *  rx_ctrl_fsm --(mrcCtrlMeta)--> rx_req_fsm --(ackCplReq)--> rx_ack_cpl --> m_axis_rx_ack_meta
 *                                   |     ^                       ^
 *                       reqStateReq |     | reqStateRsp           | retrans2rx_init
 *                                   v     |                       |
 *                                  state_table               retransmitter
 *                                   
 *                rx_req_fsm also drives: rx2retrans_upd, rx2retrans_req, rxClearTimer_req
 *
 * Step B reproduces exactly what rx_ibh_fsm + rx_exh_fsm did for an ACK (RC semantics), with one
 * fix: a NAK no longer waits for a retrans2rx_init that the retransmitter never sends.
 *
 * The three seams step C is built on:
 *   1. rx_ctrl_fsm decides which opcodes reach rx_req_fsm.
 *      C: forward MRC_SACK and MRC_NACK too.
 *   2. rx_req_fsm is the ONLY writer of the requestor reliability state (state_table via
 *      reqStateReq), of the retransmitter release/retransmit requests and of the timer.
 *      C: replace the RC logic in the MRC_ACK case, fill in the MRC_SACK / MRC_NACK cases.
 *   3. rx_ack_cpl is the ONLY producer of completions (m_axis_rx_ack_meta).
 *      C: replace the 1:1 pairing with retrans2rx_init by "complete every WR up to AETH.MSN"
 *         (6.2.1, in posted order) and error completions for semantic NAKs (Table 6-12).
 *
 * Include this file in ib_transport_protocol.cpp right after mrc_rx_ctrl.hpp: it relies on the
 * same headers (state_table.hpp, retransmitter and timer types) being included before it.
 ************************************************/
#pragma once

#include "mrc_headers.hpp"

// ------------------------------------------------------------------------------------------------
// rx_req_fsm -> rx_ack_cpl: "a transport ACK arrived, produce the completion(s) it implies"
//
// Step B only uses qpn. The other fields are carried already so that step C can change the
// completion rule without touching the interface.
// ------------------------------------------------------------------------------------------------
struct ackCplReq
{
	ap_uint<24> qpn;
	ap_uint<24> psn;      // BTH.psn of the ACK. B: RC acked psn. C: cack_psn (Table 7-6)
	ap_uint<24> msn;      // AETH.MSN. C: complete every outstanding WR with msn <= this (6.2.1)
	ap_uint<8>  syndrome; // AETH syndrome. C: semantic NAK -> error completion, Class B (Table 6-12)
	ackCplReq() {}
	ackCplReq(ap_uint<24> qpn, ap_uint<24> psn, ap_uint<24> msn, ap_uint<8> syn)
		: qpn(qpn), psn(psn), msn(msn), syndrome(syn) {}
};

/**
 * Requestor reliability engine.
 *
 * Owns the requestor-side acknowledgement state of every QP. Two-phase like rx_ibh_fsm:
 * LOAD issues the state read, PROCESS consumes it and issues the writes. The read of the next
 * packet goes through the same FIFO as the write of the previous one, so back-to-back ACKs of
 * the same QP always see the updated state.
 */
template <int WIDTH, int INSTID = 0>
void rx_req_fsm(
	stream<mrcCtrlMeta<WIDTH> >& metaIn,
	stream<reqStateRsp>&         stateTable_rsp,
	stream<reqStateReq>&         stateTable_req,
#ifdef RETRANS_EN
	stream<rxTimerUpdate>&       rxClearTimer_req,
	stream<retransUpdate>&       rx2retrans_upd,
	stream<retransmission>&      rx2retrans_req,
#endif
	stream<ackCplReq>&           cplOut
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {LOAD, PROCESS};
	static fsmStateType state = LOAD;
	static mrcCtrlMeta<WIDTH> m;

	reqStateRsp qpState;

	switch (state)
	{
	case LOAD:
		if (!metaIn.empty())
		{
			metaIn.read(m);
			stateTable_req.write(reqStateReq(m.dest_qp(15, 0)));
			state = PROCESS;
		}
		break;

	case PROCESS:
		if (!stateTable_rsp.empty())
		{
			stateTable_rsp.read(qpState);

			switch (m.op_code)
			{
			case MRC_ACK:
			{
				// ---------------------------------------------------------------------------------
				// Step B: RC behaviour, moved verbatim from rx_ibh_fsm (state update, retransmitter
				// release, timer) and rx_exh_fsm (NAK -> retransmit, completion).
				//
				// C replaces this block (see the TODOs in rx_ctrl_fsm and the step C notes):
				//   - no PSN window check: cack_psn is monotonic, stale ACKs are ignored (modular max)
				//   - ACK releases the retransmitter up to cack_psn, not "one entry per ACK"
				//   - NAKs are semantic (Class B), never a retransmission
				// ---------------------------------------------------------------------------------
				AckExHeader<WIDTH> aeth = m.getAckHeader();
				bool isNak = aeth.isNAK();

				std::cout << std::hex << "[RX REQ FSM " << INSTID << "]: AETH qp " << m.dest_qp
				          << " psn " << m.psn << " type " << aeth.getAckType() << " value " << aeth.getAckValue()
				          << " msn " << aeth.getMsn() << ", unack " << qpState.unack_psn
				          << " max sent " << qpState.max_psn_sent << std::dec << std::endl;

				// Same expression as the old rx_ibh_fsm: unack_psn <= psn <= max_psn_sent, with wrap
				bool inWindow = (qpState.unack_psn <= m.psn && m.psn <= qpState.max_psn_sent)
				             || ((qpState.unack_psn <= m.psn || m.psn <= qpState.max_psn_sent)
				                 && qpState.max_psn_sent < qpState.unack_psn);

				if (!isNak && inWindow)
				{
					stateTable_req.write(reqStateReq(m.dest_qp(15, 0), (ap_uint<24>) (m.psn + 1)));
				}
#ifdef RETRANS_EN
				if (!isNak)
				{
					// the retransmitter treats the psn as cumulative and answers with retrans2rx_init
					rx2retrans_upd.write(retransUpdate(m.dest_qp, m.psn, MRC_ACK));
					cplOut.write(ackCplReq(m.dest_qp, m.psn, aeth.getMsn(), aeth.getSyndrome()));
				}
				else
				{
					// RC behaviour kept for step B. Wrong for MRC (Table 6-11), removed in C.
					rx2retrans_req.write(retransmission(m.dest_qp, m.psn));
				}
				rxClearTimer_req.write(rxTimerUpdate(m.dest_qp, m.psn == qpState.max_psn_sent));
#else
				if (!isNak)
				{
					cplOut.write(ackCplReq(m.dest_qp, m.psn, aeth.getMsn(), aeth.getSyndrome()));
				}
#endif
				break;
			}
			// C: case MRC_SACK, case MRC_NACK (moved here from rx_ctrl_fsm)
			default:
				break;
			}

			state = LOAD;
		}
		break;
	}
}

/**
 * Completion generation for transport ACKs.
 *
 * Step B: one completion per non-NAK ACK, built from the retransmitter entry the ACK released
 * (same pairing rx_exh_fsm did). Isolated in its own process so that waiting for the
 * retransmitter can never stall rx_req_fsm, and never the data path.
 */
template <int INSTID = 0>
void rx_ack_cpl(
	stream<ackCplReq>&     cplIn,
#ifdef RETRANS_EN
	stream<retransRdInit>& retrans2rx_init,
#endif
	stream<ackMeta>&       m_axis_rx_ack_meta
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	ackCplReq req;

#ifdef RETRANS_EN
	retransRdInit init;

	if (!cplIn.empty() && !retrans2rx_init.empty())
	{
		cplIn.read(req);
		retrans2rx_init.read(init);

		ap_uint<4> dst  = init.host ? (ap_uint<4>) init.laddr(51, 48) : (ap_uint<4>) 0;
		ap_uint<2> strm = init.host ? (ap_uint<2>) init.laddr(53, 52) : (ap_uint<2>) 0;
		m_axis_rx_ack_meta.write(ackMeta(MRC_ACK, req.qpn(15, 0), init.host, dst, strm, init.lst));

		std::cout << std::hex << "[RX ACK CPL " << INSTID << "]: completion qp " << req.qpn
		          << " lst " << init.lst << std::dec << std::endl;
	}
#else
	// Without the retransmitter there is no per-WR information to complete with
	if (!cplIn.empty())
	{
		cplIn.read(req);
		m_axis_rx_ack_meta.write(ackMeta(MRC_ACK, req.qpn(15, 0), 0, 0, 0, 1));
	}
#endif
}