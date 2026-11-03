#pragma once

#include "../mrc_headers.hpp"
#include "../tables/msn_table.hpp"

// still mixes data and ctrl
template <int WIDTH, int INSTID = 0>
void generate_exh(	
	stream<event>& metaIn,
	stream<txMsnRsp>& msnTable2txExh_rsp,
	stream<ap_uint<16> >& txExh2msnTable_req,
	//stream<txReadReqUpdate>& tx_readReqTable_upd,
	stream<ap_uint<16> >& lengthFifo,
	stream<txPacketInfo>& packetInfoFifo,
#ifdef RETRANS_EN
	stream<ap_uint<24> >&	txSetTimer_req,
	//stream<retransAddrLen>&		tx2retrans_insertAddrLen,
#endif
	stream<net_axis<WIDTH> >& output
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum ge_fsmStateType {META, GET_MSN, PROCESS};
	static ge_fsmStateType ge_state = META;
	static event meta;
	net_axis<WIDTH> sendWord;
	static RdmaExHeader<WIDTH> rdmaHeader;
	static AckExHeader<WIDTH>  ackHeader;
	static bool metaWritten;
	static txMsnRsp msnMeta;
	ap_uint<16> udpLen;
	txPacketInfo info;

	switch(ge_state)
	{
	case META:
		if (!metaIn.empty())
		{
			rdmaHeader.clear();
			ackHeader.clear();

			metaIn.read(meta);
			metaWritten = false;
			//if (meta.op_code == MRC_RDMA_READ_RESP_ONLY || meta.op_code == MRC_RDMA_READ_RESP_FIRST || meta.op_code == MRC_RDMA_READ_RESP_MIDDLE || meta.op_code == MRC_RDMA_READ_RESP_LAST || meta.op_code == MRC_ACK)
			{
				txExh2msnTable_req.write(meta.qpn);
				ge_state = GET_MSN;
			}
			//else
			{
				// Start Timer for RDMW_WRITE_* & RDMA_READ_REQUEST

				//txSetTimer_req.write(meta.qpn);
				//ge_state = PROCESS;
			}
#ifdef RETRANS_EN
			//TODO PART HIST
			if (meta.op_code == MRC_RDMA_WRITE_ONLY || meta.op_code == MRC_RDMA_WRITE_FIRST || meta.op_code == MRC_RDMA_WRITE_MIDDLE || meta.op_code == MRC_RDMA_WRITE_LAST)
			{
				txSetTimer_req.write(meta.qpn);
			}
#endif
		}
		break;
	case GET_MSN:
		if (!msnTable2txExh_rsp.empty())
		{
			msnTable2txExh_rsp.read(msnMeta);
			ge_state = PROCESS;
		}
		break;
	case PROCESS:
		{
			sendWord.last = 0;
			switch(meta.op_code)
			{
			case MRC_RDMA_WRITE_ONLY:
			case MRC_RDMA_WRITE_FIRST:
			{
				// [BTH][RETH][PayLd]
				rdmaHeader.setVirtualAddress(meta.addr);
				rdmaHeader.setLength(meta.length); //TODO Move up??
				rdmaHeader.setRemoteKey(msnMeta.r_key);
				ap_uint<8> remainingLength = rdmaHeader.consumeWord(sendWord.data);
				sendWord.keep = ~0;
				sendWord.last = (remainingLength == 0);
				std::cout << "[GENERATE EXH " << INSTID << "]: RDMA_WRITE_ONLY/FIRST" << std::endl;
#ifdef DBG_FULL
				print(std::cout, sendWord);
				std::cout << std::endl;
#endif
				output.write(sendWord);
				if (remainingLength == 0)
				{
					//TODO
				}
				if (!metaWritten) //TODO we are losing 1 cycle here
				{
					info.isAETH = false;
					info.hasHeader = true;
					info.hasPayload = (meta.length != 0); //TODO should be true
					packetInfoFifo.write(info);

					//BTH: 12, RETH: 16, PayLd: x, ICRC: 4
					ap_uint<32> payloadLen = meta.length;
					if (meta.op_code == MRC_RDMA_WRITE_FIRST)
					{
						payloadLen = PMTU;
					}
					udpLen = 12+16+payloadLen+4; //TODO dma_len can be much larger, for multiple packets we need to split this into multiple packets
					lengthFifo.write(udpLen);
					//Store meta for retransmit
					metaWritten = true;
				}
				break;
			}
			case MRC_RDMA_WRITE_MIDDLE:
			case MRC_RDMA_WRITE_LAST:
				// [BTH][PayLd]
				info.isAETH = false;
				info.hasHeader = false;
				info.hasPayload = (meta.length != 0); //TODO should be true
				packetInfoFifo.write(info);
				//BTH: 12, PayLd: x, ICRC: 4
				udpLen = 12+meta.length+4;
				lengthFifo.write(udpLen);
				//Store meta for retransmit
				ge_state = META;
				break;
			case MRC_ACK:
			{
				// [BTH][AETH]
				//Check if ACK or NAK
				if (!meta.isNak)
				{
					ackHeader.setSyndrome(0x1f);
				}
				else
				{
					//PSN SEQ error
					std::cout << "[GENERATE EXH " << INSTID << "]: psn seq error" << std::endl;
					ackHeader.setSyndrome(0x60);
				}
				std::cout << "[GENERATE EXH " << INSTID << "]: MRC_ACK with qpn " << meta.qpn << ", psn " << meta.psn << std::endl;
				ackHeader.setMsn(msnMeta.msn);
				ackHeader.consumeWord(sendWord.data); //TODO
				{
					info.isAETH = true;
					info.hasHeader = true;
					info.hasPayload = false;
					packetInfoFifo.write(info);
					sendWord.keep(AETH_SIZE/8-1, 0) = 0xFF;
					sendWord.keep(WIDTH/8-1, (AETH_SIZE/8)) = 0;
					sendWord.last = 1;
#ifdef DBG_FULL
					std::cout << "[GENERATE EXH " << INSTID << "]: MRC_ACK ";
					print(std::cout, sendWord);
					std::cout << std::endl;
#endif
					output.write(sendWord);
					//BTH: 12, AETH: 4, ICRC: 4
					lengthFifo.write(12+4+4);
				}
				break;
			}
			default:
				break;
			} //switch
		} //if empty
		if (sendWord.last)
		{
			ge_state = META;
		}
		break;
	}//switch
}
