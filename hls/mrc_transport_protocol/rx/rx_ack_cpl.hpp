#pragma once

#include "../mrc_headers.hpp"
#include "../tables/retransmitter/retransmitter_types.hpp"

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
