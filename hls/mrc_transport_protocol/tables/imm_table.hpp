#pragma once

#include "../mrc_transport_protocol.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS

// rx_exh_gsm reads data from msn_table and sends it to imm_table
// rx_exh_fsm -> imm_table, at most one per packet
struct rxImmReq
{
	ap_uint<16> qpn;
	bool        hasImmDt; // LAST_IMM / ONLY_IMM: store immDt at rqmsn
	ap_uint<16> rqmsn;
	ap_uint<32> immDt;
	ap_uint<16> head;     // RQMSN of the oldest WriteIMM not completed yet
	ap_uint<8>  immInc;   // how many WriteIMMs completed with this packet, starting from head
	rxImmReq() :immInc(0) {}
	rxImmReq(ap_uint<16> qpn, bool hasImmDt, ap_uint<16> rqmsn, ap_uint<32> immDt, ap_uint<16> head, ap_uint<8> immInc)
		:qpn(qpn), hasImmDt(hasImmDt), rqmsn(rqmsn), immDt(immDt), head(head), immInc(immInc) {}
};

// per QP ring of MRC_MAX_WIMM ImmDt, written OOO at their RQMSN slot and read in order from head (6.3.3)
// one completion per cycle, so a packet completing many WriteIMMs never stalls rx_exh_fsm
template <int INSTID = 0>
void imm_table(
	stream<rxImmReq>& rxExh2immTable_req,
	stream<cplMeta>&  immTable2ackCpl
) {
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off

	static ap_uint<32> imm_table[MAX_QPS][MRC_MAX_WIMM];
#if defined( __VITIS_HLS__)
	#pragma HLS bind_storage variable=imm_table type=RAM_2P impl=BRAM
#endif
	static rxImmReq req;

    // read immInc values starting from head

	if (req.immInc != 0)
	{
		ap_uint<32> imm = imm_table[req.qpn][req.head(MRC_MAX_WIMM_BITS - 1, 0)];
		immTable2ackCpl.write(cplMeta(MRC_RDMA_WRITE_ONLY_IMM, req.qpn, PKG_HOST, 0, 0, PKG_F, imm));
		std::cout << std::hex << "[IMM TABLE " << INSTID << "]: completion qp " << req.qpn << " rqmsn " << req.head
		          << " imm " << imm << std::dec << std::endl;
		req.head++;
		req.immInc--; // emit one completion per cycle
	}
	else if (!rxExh2immTable_req.empty())
	{
		rxExh2immTable_req.read(req);
		if (req.hasImmDt)
			imm_table[req.qpn][req.rqmsn(MRC_MAX_WIMM_BITS - 1, 0)] = req.immDt;
	}
}
