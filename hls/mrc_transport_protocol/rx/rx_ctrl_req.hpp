// receive ACK/NACK/SACK/EP_RSP

#pragma once

#include "../mrc_headers.hpp"
#include "../tables/state_table.hpp"
#include "../tables/retransmitter.hpp"

typedef enum {
	REQ_ERR_REMOTE_NAK     = 0, // AETH NAK: Invalid Request / Remote Access / Remote Op Error
	REQ_ERR_REMOTE_OP      = 1, // NACK UNEXP_EVENT ("Remote Operation Error")
	REQ_ERR_RETRY_EXCEEDED = 2  // NACK retries exhausted ("Retry Counter Exceeded")
} reqErrType;

struct reqErrEvent
{
	ap_uint<16> qpn;
	ap_uint<24> psn;   // PSN the error refers to
	ap_uint<8>  code;  // AETH syndrome value or NACK reason
	ap_uint<2>  type;  // reqErrType
	reqErrEvent() {}
	reqErrEvent(ap_uint<16> qpn, ap_uint<24> psn, ap_uint<8> code, reqErrType t)
		: qpn(qpn), psn(psn), code(code), type(t) {}
};

// NACK reasons the requestor retries (Table 7-3); everything else is not retriable
inline bool mrcNackRetriable(ap_uint<8> reason)
{
#pragma HLS inline
	return reason == NACK_TRIMMED   || reason == NACK_TRIMMED_LASTHOP || reason == NACK_NO_BITMAP ||
	       reason == NACK_NO_PKT_BUFFER || reason == NACK_NO_RESOURCE || reason == NACK_PSN_OOR_WINDOW;
}

// requestor reliability engine: the only writer of the requestor state, of retransmitter release / retransmit requests and of the Local ACK timer
// LOAD issues the state read
// PROCESS decides and writes back at most once
// the read of the next packet goes through the same FIFO as the write of the previous one, so back-to-back packets of the same QP always see the updated state
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
	stream<reqErrEvent>&         errOut
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fsmStateType {LOAD, PROCESS};
	static fsmStateType state = LOAD;
	static mrcCtrlMeta<WIDTH> m;

	reqStateRsp qp;

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
			stateTable_rsp.read(qp);

			ap_uint<16> qpn      = m.dest_qp(15, 0);
			ap_uint<24> unack    = qp.unack_psn;
			ap_uint<3>  retries  = qp.retries;
			bool        error    = qp.error;
			bool        advanced = false;  // unack_psn moved forward
			bool        dirty    = false;  // requestor state must be written back
			bool        hasCack  = false;  // this packet carries a cumulative ack (ACK or SACK)
			ap_uint<24> cack     = 0;

			// TODO (all opcodes) validate: dest_qp is a valid QP; spdcid == dest_qp[15:0] (SACK/NACK)

			switch (m.op_code)
			{
			case MRC_ACK:
			{
				AckExHeader<WIDTH> aeth = m.getAckHeader();
				std::cout << std::hex << "[RX REQ FSM " << INSTID << "]: ACK qp " << m.dest_qp
				          << " psn " << m.psn << " type " << aeth.getAckType() << " value " << aeth.getAckValue()
				          << " msn " << aeth.getMsn() << ", unack " << unack << " max sent " << qp.max_psn_sent
				          << std::dec << std::endl;
				if (error) break;

				if (aeth.getAckType() == AETH_ACK)
				{
					// BTH.psn of a transport ACK is the responder's cack_psn (Table 7-6).
					// AETH.MSN is only logged: completion is decided by PSN, which does not depend
					// on the peer's MSN bookkeeping. The credit field is always 0x1F in MRC (6.4).
					cack    = m.psn;
					hasCack = true;
				}
				else if (aeth.getAckType() == AETH_NAK)
				{
					// Class B (Table 6-12): no retransmission, the QP goes to ERROR
					std::cout << std::hex << "[RX REQ FSM " << INSTID << "]: NAK code " << aeth.getAckValue()
					          << " on qp " << m.dest_qp << " -> QP error" << std::dec << std::endl;
					errOut.write(reqErrEvent(qpn, m.psn, aeth.getAckValue(), REQ_ERR_REMOTE_NAK));
					error = true;
					dirty = true;
				}
				else
				{
					// RNR NAK and reserved: not applicable in MRC (Table 6-11)
					std::cout << "[RX REQ FSM " << INSTID << "]: RNR/reserved AETH type, ignored" << std::endl;
				}
				break;
			}
			case MRC_SACK:
			{
				SackExHeader<WIDTH> h = m.getSackHeader();
				std::cout << std::hex << "[RX REQ FSM " << INSTID << "]: SACK qp " << m.dest_qp
				          << " cack_psn " << h.getCackPsn() << " ack_psn " << h.getAckPsn()
				          << " pr " << h.isProbeResponse() << " m " << h.getMark()
				          << " sack_base " << h.getSackBasePsn() << " bitmap " << h.getSackBitmap()
				          << " ooo " << h.getOooCount() << " rcvd_bytes " << h.getRcvdBytes()
				          << ", unack " << unack << " max sent " << qp.max_psn_sent << std::dec << std::endl;
				if (error) break;

				// SETH.cack_psn (BTH.psn is reserved in probe responses, 7.4.6)
				cack    = h.getCackPsn();
				hasCack = true;

				// TODO pr == 1: probe response, match getProbeId() against the outstanding probe
				//      (ack_psn_offset is NOT a PSN offset then)
				// TODO fast loss detection from sack_bitmap anchored at getSackBasePsn() (6.2.3, 7.4.5)
				// TODO getMark(): path skipping for getEntropy() (9.3.1)
				// TODO getMpr() != 0: max_psn_range = 128 * mpr, limits TX (7.4.4)
				// TODO CC OnACK() if getCcType() == MRC_CC_TYPE_DEFAULT (8.4)
				break;
			}
			case MRC_NACK:
			{
				NackExHeader<WIDTH> h = m.getNackHeader();
				ap_uint<24> npsn   = h.getNackPsn();
				ap_uint<8>  reason = h.getNackReason();
				std::cout << std::hex << "[RX REQ FSM " << INSTID << "]: NACK qp " << m.dest_qp
				          << " reason " << reason << " nack_psn " << npsn << " ts " << h.getTxTimestamp()
				          << ", unack " << unack << " max sent " << qp.max_psn_sent
				          << " retries " << retries << std::dec << std::endl;
				if (error) break;

				// TODO CC OnNACK() with getTxTimestamp() (8.5)
				if (!(mrcPsnLE(unack, npsn) && mrcPsnLE(npsn, qp.max_psn_sent)))
				{
					std::cout << "[RX REQ FSM " << INSTID << "]: NACK psn not outstanding "
					          << "(already acknowledged or never sent), ignored" << std::endl;
				}
				else if (reason == NACK_UNEXP_EVENT)
				{
					errOut.write(reqErrEvent(qpn, npsn, reason, REQ_ERR_REMOTE_OP));
					error = true;
					dirty = true;
				}
				else if (mrcNackRetriable(reason))
				{
					if (retries == MRC_REQ_MAX_RETRIES)
					{
						errOut.write(reqErrEvent(qpn, npsn, reason, REQ_ERR_RETRY_EXCEEDED));
						error = true;
					}
					else
					{
						retries++;
#ifdef RETRANS_EN
						rx2retrans_req.write(retransmission(qpn, npsn, true));
#endif
					}
					dirty = true;
				}
				else
				{
					std::cout << "[RX REQ FSM " << INSTID << "]: unknown NACK reason, ignored" << std::endl;
				}
				break;
			}
			default:
				break;
			}

			if (hasCack)
			{
				if (!mrcPsnLE(cack, qp.max_psn_sent))
				{
					std::cout << std::hex << "[RX REQ FSM " << INSTID << "]: cack_psn " << cack
					          << " acknowledges a PSN never sent, ignored" << std::dec << std::endl;
				}
				else
				{
					if (mrcPsnLE(unack, cack))   // cack_psn + 1 is after unack_psn: progress
					{
						unack    = cack + 1;
						retries  = 0;
						advanced = true;
						dirty    = true;
					}
#ifdef RETRANS_EN
					// Only the transport ACK releases and completes (7.2). Sent even without progress:
					// a SACK may already have moved unack_psn past this cack_psn. A stale ACK releases
					// nothing, the retransmitter compares modulo 2^24.
					if (m.op_code == MRC_ACK)
						rx2retrans_upd.write(retransUpdate(qpn, cack, MRC_ACK));
#endif
				}
			}

			if (dirty)
				stateTable_req.write(reqStateReq(qpn, unack, retries, error));

#ifdef RETRANS_EN
			if (advanced)
			{
				// everything sent is acknowledged -> stop, otherwise restart (7.2)
				bool allAcked = (unack == (ap_uint<24>) (qp.max_psn_sent + 1));
				rxClearTimer_req.write(rxTimerUpdate(qpn, allAcked));
			}
			else if (error && !qp.error)
			{
				rxClearTimer_req.write(rxTimerUpdate(qpn, true));   // QP just entered ERROR
			}
#endif
			state = LOAD;
		}
		break;
	}
}

// completions for the host
// every retransmitter entry released by a transport ACK becomes one ackMeta, in posted order, carrying that packet's lst: the same stream the host received before, now driven by cumulative ACKs 
// kept in its own process so a slow host interface never stalls rx_req_fsm
template <int INSTID = 0>
void rx_ack_cpl(
#ifdef RETRANS_EN
	stream<retransRdInit>& retrans2rx_init,
#endif
	stream<reqErrEvent>&   errIn,
	stream<ackMeta>&       m_axis_rx_ack_meta
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	bool done = false;

#ifdef RETRANS_EN
	if (!retrans2rx_init.empty())
	{
		retransRdInit init;
		retrans2rx_init.read(init);

		ap_uint<4> dst  = init.host ? (ap_uint<4>) init.laddr(51, 48) : (ap_uint<4>) 0;
		ap_uint<2> strm = init.host ? (ap_uint<2>) init.laddr(53, 52) : (ap_uint<2>) 0;
		m_axis_rx_ack_meta.write(ackMeta(MRC_ACK, init.qpn, init.host, dst, strm, init.lst));

		std::cout << std::hex << "[RX ACK CPL " << INSTID << "]: completion qp " << init.qpn
		          << " psn " << init.psn << " lst " << init.lst << std::dec << std::endl;
		done = true;
	}
#endif

	if (!done && !errIn.empty())
	{
		reqErrEvent err;
		errIn.read(err);
		std::cout << std::hex << "[RX ACK CPL " << INSTID << "]: QP " << err.qpn << " in ERROR, type "
		          << err.type << " code " << err.code << " psn " << err.psn << std::dec << std::endl;
		// TODO error completion to the host: needs a status field in ackMeta (top-level interface)
		// TODO flush outstanding WRs of this QP with "flushed in error" status
	}
}