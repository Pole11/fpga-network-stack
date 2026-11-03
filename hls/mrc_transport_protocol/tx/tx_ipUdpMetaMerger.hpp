#pragma once

#include "../mrc_headers.hpp"
#include "../tables/conn_table.hpp"

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
