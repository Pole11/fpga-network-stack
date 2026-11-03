#pragma once

#include "../mrc_headers.hpp"

template <int WIDTH, int INSTID = 0>
void rx_process_ibh(	
#ifdef DBG_IBV
	stream<psnPkg>& m_axis_dbg,
#endif
	stream<net_axis<WIDTH> >& input,
	stream<ibhMeta>& metaToIbhFsmOut,              	// data path only (requests)
	stream<mrcEthMeta>& metaToStripCtrlOut,        	// data path only, you only need to know if it has METH and TSETH
	stream<mrcCtrlBthMeta>& metaToProcCtrlOut,   	// control path only, you must have all the possible information of BTH needed for ctrl
	stream<bool>& isCtrlDataOut,                    // every packet, true = control
	stream<bool>& isCtrlUdpOut,                     // every packet, true = control
	stream<net_axis<WIDTH> >& forwardInputOut,
	ap_uint<32>& regIbvCountRx
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	static BaseTransportHeader<WIDTH> bth;
	static bool metaWritten = false;
	static ap_uint<32> validRx = 0;

	net_axis<WIDTH> currWord;

	if (!input.empty()) {
		input.read(currWord);
		bth.parseWord(currWord.data);

		if (bth.isReady()) {
			forwardInputOut.write(currWord);
			
			if (!metaWritten) {
				mrcOpCode opCode = bth.getOpCode();
				bool isCtrl = checkIfMrcCtrl(opCode);
				std::cout << "[RX PROCESS IBH " << INSTID << "]: input psn " << std::hex << bth.getPsn() << std::endl;

				isCtrlDataOut.write(isCtrl);
				isCtrlUdpOut.write(isCtrl);
				if (isCtrl)
				{
					// SACK/NACK/PROBE/EP: dqp and psn have an opcode-dependent meaning (Table 7-6),
					// these packets never go through the RC PSN checks of rx_ibh_fsm
					metaToProcCtrlOut.write(mrcCtrlBthMeta(opCode, bth.getPartitionKey(), bth.getDstQP(), bth.getPsn(),
					                                 bth.getAckReq(), bth.getRtx()));
				}
				else
				{
					ibhMeta m(opCode, bth.getPartitionKey(), bth.getDstQP(), bth.getPsn(), true, bth.getAckReq(), bth.getRtx(), bth.getTs());
					metaToIbhFsmOut.write(m);
					metaToStripCtrlOut.write(mrcEthMeta(opCode, bth.getTs()));
				}
				metaWritten = true;

                validRx++;
		        regIbvCountRx = validRx;

				std::cout << "[RX PROCESS IBH " << INSTID << "]: process IBH opcode: " << opCode << (isCtrl ? " (ctrl)" : "") << std::endl;
			}
		}
		
		if (currWord.last) {
			bth.clear();
			metaWritten = false;

#ifdef DBG_IBV
		m_axis_dbg.write(psnPkg(bth.getOpCode(), bth.getPsn(), bth.getDstQP(), currWord.last));
#endif
		}
	}
}
