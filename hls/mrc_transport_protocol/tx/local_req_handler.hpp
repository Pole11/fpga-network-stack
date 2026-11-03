#pragma once

#include "../mrc_headers.hpp"
#include "../tables/retransmitter/retransmitter_types.hpp"

template <int INSTID = 0>
void local_req_handler(
	stream<txMeta>& s_axis_sq_meta,
#ifdef RETRANS_EN
	stream<retransEvent>& retransEventFifo,
	stream<retransAddrLen>& tx2retrans_insertAddrLen,
#endif
	stream<memCmdInternal>&	tx_local_memCmdFifo, //TODO rename
	//stream<mqInsertReq<ap_uint<64> > >&	tx_localReadAddrFifo,
	stream<event>&tx_localTxMeta,
    ap_uint<32>& regRetransCount)
{
#pragma HLS inline off
#pragma HLS pipeline II=1

	//enum fsmStateType {META, GENERATE};
	//static fsmStateType lrh_state;
	static txMeta meta;

	event ev;
	retransEvent rev;
    static ap_uint<32> retransCount = 0;

	//switch (lrh_state)
	//{
	//case META:
#ifdef RETRANS_EN
	if (!retransEventFifo.empty())
	{
		retransEventFifo.read(rev);
		tx_localTxMeta.write(event(rev.op_code, rev.qpn, rev.remoteAddr, rev.length, rev.psn));

        retransCount++;
        regRetransCount = retransCount;        

		std::cout << std::dec << "[LOCAL REQ HANDLER " << INSTID << "]: retransmission: length: " << rev.length << ", local addr: " << std::hex << rev.localAddr << ", remote addres: " << rev.remoteAddr << ", psn: " << rev.psn << ", op code: " << rev.op_code << ", qpn: " << rev.qpn << std::endl;

		std::cout << "[LOCAL REQ HANDLER " << INSTID << "]: retranmission writing into memCmd with lengh " << rev.length << std::endl;
		tx_local_memCmdFifo.write(memCmdInternal(rev.op_code, rev.qpn, rev.localAddr, rev.length, PKG_INT, PKG_R, rev.offs));
		

	}
	else if (!s_axis_sq_meta.empty())
#else

	if (!s_axis_sq_meta.empty())
#endif
	{
		s_axis_sq_meta.read(meta); 

		std::cout << std::dec << "[LOCAL REQ HANDLER " << INSTID << "]: opcode: " << meta.op_code << 
			", transmission: length: " << std::dec << meta.len << ", local addr: " << std::hex << meta.laddr << ", remote addres: " << meta.raddr << ", last: " << meta.lst << std::endl;

		if(meta.op_code == MRC_RDMA_WRITE_MIDDLE || meta.op_code == MRC_RDMA_WRITE_FIRST ||
			meta.op_code == MRC_RDMA_WRITE_LAST || meta.op_code == MRC_RDMA_WRITE_ONLY)
		{
			tx_localTxMeta.write(event(meta.op_code, meta.qpn, meta.raddr, meta.len));
			tx_local_memCmdFifo.write(memCmdInternal(meta.op_code, meta.qpn, meta.laddr, meta.len, meta.host, PKG_NR, meta.offs));	
		}
#ifdef RETRANS_EN
		tx2retrans_insertAddrLen.write(retransAddrLen(meta.laddr, meta.raddr, meta.len, meta.lst, meta.offs, meta.host));
#endif
	}
}
