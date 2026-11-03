#pragma once

#include "../mrc_headers.hpp"
#include "../tables/state_table.hpp"
#include "../tables/retransmitter/retransmitter_types.hpp"

// still mixes data and ctrl
// for everything we get PSN from state_table
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
