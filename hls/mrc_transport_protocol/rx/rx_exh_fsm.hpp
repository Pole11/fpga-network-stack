#pragma once

#include "../mrc_headers.hpp"
#include "../tables/msn_table.hpp"
#include "../tables/imm_table.hpp"

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
	stream<rxImmReq>& rxExh2immTable_req,
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
			payLoadLength = udpLength - (8 + 12 + exHeader.mrc.length() + 16 + 4) - meta.padCnt;

			// lst marks the packet that completes WriteIMMs (6.3.2): once its write is in memory, so are all the
			// packets before it, and its ImmDt (immMeta.last = 1 on the last one) can be delivered
			memoryWriteCmd.write(memCmd(meta.op_code, meta.dest_qp, (meta.immInc != 0) ? PKG_F : PKG_NF,
			                            rdmaHeader.getVirtualAddress(), payLoadLength, PKG_NR, PKG_HOST, 0));
			
			if (immPkt || meta.immInc != 0)
				rxExh2immTable_req.write(rxImmReq(meta.dest_qp, immPkt, exHeader.mrc.rqmsn, exHeader.mrc.immDt, meta.rqmsnHead, meta.immInc));
			if (meta.msnInc != 0)
				rxExh2msnTable_upd_req.write(rxMsnReq(meta.dest_qp, dmaMeta.msn + meta.msnInc));
			if (meta.ackReq)
				rx_exhEventMetaFifo.write(ackEvent(meta.dest_qp, meta.cack, false)); // todo: don't generate an ACK, but a SACK request one all previous packets arrived
			
			rx_pkgSplitTypeFifo.write(pkgSplit(meta.op_code));
			rx_pkgShiftTypeFifo.write(pkgShift(immPkt ? SHIFT_RETH_IMM : SHIFT_RETH, meta.dest_qp, meta.padCnt));
			pe_fsmState = META;
			break;
		}
		default:
			break;
		} //switch meta_Opcode
		break;
	} //switch
}
