#pragma once

#include "../mrc_headers.hpp"

// control packet dispatch
// this process must keep reading metaIn every cycle so it never backpressures RX
// reqOut is the exception: it is written with a blocking write, and rx_req_fsm needs two cycles plus a state table round trip per packet, so size its FIFO for ACK bursts 
template <int WIDTH, int INSTID = 0>
void rx_ctrl_fsm(
	stream<mrcCtrlMeta<WIDTH> >& metaIn,
    stream<mrcCtrlMeta<WIDTH> >& reqOut   // requestor-side packets -> rx_req_fsm
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	mrcCtrlMeta<WIDTH> m;

	if (!metaIn.empty())
	{
		metaIn.read(m);

		if (!m.lengthOk)
		{
			// TODO: count malformed control packets (register) and drop
			std::cout << "[RX CTRL FSM " << INSTID << "]: short control packet, opcode 0x" << std::hex
			          << m.op_code << " udp length " << std::dec << m.udpLength << ", dropped" << std::endl;
			return;
		}

		switch (m.op_code)
		{
		case MRC_ACK:  // transport ACK / NAK   (6.2.4, Table 7-6)
		case MRC_SACK: // reliability SACK      (7.4, 7.5.5.5)
		case MRC_NACK: // reliability NACK      (7.5.3, 7.5.5.6, Table 7-3)
		{
			// requestor side: rx_req_fsm owns the requestor reliability state
			reqOut.write(m);
			break;
		}
		case MRC_PROBE_REQ: // responder side (6.5.1)
		{
			ProbeExHeader<WIDTH> h = m.getProbeHeader();
			std::cout << std::hex << "[RX CTRL FSM " << INSTID << "]: PROBE qp " << m.dest_qp
			          << " probe_id " << h.getProbeId() << std::dec << std::endl;
			// TODO answer with a SACK: pr = 1, ack_psn_offset = getProbeId(), current responder
			//      reliability state (cack_psn, bitmap). Probes do not consume PSNs.
			break;
		}
		case MRC_EP_REQ: // 7.3, 6.5.2, 6.6
		{
			EpReqExHeader<WIDTH> h = m.getEpReqHeader();
			std::cout << std::hex << "[RX CTRL FSM " << INSTID << "]: EP REQ op " << h.getOp()
			          << " id " << (ap_uint<16>) m.psn(15, 0) << " mask " << h.getPortStatusMask() << std::dec << std::endl;
			// TODO EP_OP_PORT_STATUS: store peer port reachability from getPortStatusMask()
			// TODO EP_OP_EV_PROBE: answer with an EETH to m.srcAddress, BTH.psn[15:0] = m.psn[15:0],
			//      tx_timestamp per Table 7-23 (reflected / service-time adjusted / zero)
			break;
		}
		case MRC_EP_RSP: // 6.5.2
		{
			EpRspExHeader<WIDTH> h = m.getEpRspHeader();
			std::cout << std::hex << "[RX CTRL FSM " << INSTID << "]: EP RSP op " << h.getOp()
			          << " id " << (ap_uint<16>) m.psn(15, 0) << " ts " << h.getTxTimestamp() << std::dec << std::endl;
			// TODO match m.psn[15:0] against the outstanding EV probe, compute return-trip latency
			break;
		}
		default:
			break;
		}
	}
}
