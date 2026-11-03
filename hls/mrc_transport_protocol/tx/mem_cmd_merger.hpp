#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void mem_cmd_merger(
	stream<memCmdInternal>& localReadRequests,
	stream<memCmd>& out,
	stream<pkgInfo>& pkgInfoFifo
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	memCmdInternal cmd;

	if (!localReadRequests.empty())
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
