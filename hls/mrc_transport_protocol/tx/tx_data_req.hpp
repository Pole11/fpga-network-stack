// send WRITEs

#pragma once

#include "../mrc_headers.hpp"

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

template <int WIDTH, int INSTID = 0>
void mem_cmd_merger(
	stream<memCmdInternal>& remoteReadRequests,
	stream<memCmdInternal>& localReadRequests,
	stream<memCmd>& out,
	stream<pkgInfo>& pkgInfoFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	memCmdInternal cmd;

	if (!remoteReadRequests.empty())
	{
        std::cout << "[MEM CMD MERGER " << INSTID << "]: reading remote request" << std::endl;
		remoteReadRequests.read(cmd);
	}
	else if (!localReadRequests.empty())
	{
		localReadRequests.read(cmd);

		std::cout << "[MEM CMD MERGER " << INSTID << "]: reading local request, opcode - " << std::hex << cmd.op_code 
            << ", sync - " << cmd.sync << ", offs - " << cmd.offs << ", vaddr - " << cmd.addr << ", len - " << cmd.len << std::endl;

		if(cmd.op_code == MRC_RDMA_WRITE_ONLY)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_F, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RETH, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
		if(cmd.op_code == MRC_RDMA_WRITE_FIRST)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_NF, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RETH, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
		if(cmd.op_code == MRC_RDMA_WRITE_MIDDLE)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_NF, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RAW, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
		if(cmd.op_code == MRC_RDMA_WRITE_LAST)
		{
			out.write(memCmd(cmd.op_code, cmd.qpn, PKG_F, cmd.addr, cmd.len, cmd.sync, 0, cmd.offs));
			pkgInfoFifo.write(pkgInfo(RAW, ((cmd.len+(WIDTH/8)-1)/(WIDTH/8))));
		}
	}

}

template <int WIDTH, int INSTID = 0>
void tx_pkg_arbiter(
	stream<pkgInfo>& tx_pkgInfoFifo,
	stream<net_axis<WIDTH> >& s_axis_mem_read_data,
	stream<net_axis<WIDTH> >& remoteReadData,
	stream<net_axis<WIDTH> >& localReadData,
	stream<net_axis<WIDTH> >& rawPayFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum mrpStateType{IDLE, FWD_AETH, FWD_RETH, FWD_RAW};
	static mrpStateType state = IDLE;
	static ap_uint<8> wordCounter = 0;

	static pkgInfo info;
	net_axis<WIDTH> currWord;

	switch (state)
	{
	case IDLE:
		if (!tx_pkgInfoFifo.empty())
		{
			tx_pkgInfoFifo.read(info);
			wordCounter = 0;

			if (info.type == AETH)
			{
				state = FWD_AETH;
			}
			else if (info.type == RETH)
			{
				state = FWD_RETH;
			}
			else
			{
				state = FWD_RAW;
			}
		}
		break;
	case FWD_AETH:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);

			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			remoteReadData.write(currWord);
		}
		break;
	case FWD_RETH:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);

			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			localReadData.write(currWord);
		}
		break;
	case FWD_RAW:
		if (!s_axis_mem_read_data.empty())
		{
			s_axis_mem_read_data.read(currWord);
			
			wordCounter++;
			if (currWord.last)
			{
				state = IDLE;
			}
			if (wordCounter == PMTU_WORDS)
			{
				currWord.last = 1;
				state = IDLE;
			}
			rawPayFifo.write(currWord);
		}
		break;
	}//switch

}

void merge_retrans_request(	
	stream<retransMeta>&		tx2retrans_insertMeta,
	stream<retransAddrLen>&		tx2retrans_insertAddrLen,
	stream<retransEntry>&		tx2retrans_insertRequest
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	retransMeta meta;
	retransAddrLen addrlen;

	if (!tx2retrans_insertMeta.empty() && !tx2retrans_insertAddrLen.empty())
	{
		tx2retrans_insertMeta.read(meta);
		tx2retrans_insertAddrLen.read(addrlen);
		tx2retrans_insertRequest.write(retransEntry(meta, addrlen));
	}
}