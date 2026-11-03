#pragma once

#include "../mrc_headers.hpp"

// join data (event) and ctrl (ackEvent)  
// counterpart of rx_split_ctrl
// rx_ackEventFifo MRC_ACK from ibh and exh
// tx_appMetaFifo, retransmission events, WRITEs and READ_REQ only
template <int INSTID = 0>
void meta_merger(	 
	stream<ackEvent>&	rx_ackEventFifo,
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
