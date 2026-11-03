#pragma once

#include "../mrc_headers.hpp"
#include "../tables/msn_table.hpp"

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

			// last packet is wrong, completions are different in MRC, but keep for now, until write IMM are done 6.3.2
			memoryWriteCmd.write(memCmd(meta.op_code, meta.dest_qp, lastPkt ? PKG_F : PKG_NF,
			                            rdmaHeader.getVirtualAddress(), payLoadLength, PKG_NR, PKG_HOST, 0));
			
			// probably the lastmap should be used to send the signal when a message completes
			// note that this line is coalescing message completions, which is wrong for MRC!
			// completions is done by write immediates, so you need to keep track of them
			//memoryWriteCmd.write(memCmd(meta.op_code, meta.dest_qp, (meta.immInc != 0) ? PKG_F : PKG_NF,
            //                			rdmaHeader.getVirtualAddress(), payLoadLength, PKG_NR, PKG_HOST, 0));
			
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
