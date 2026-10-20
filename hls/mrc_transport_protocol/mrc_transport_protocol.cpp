/************************************************
Copyright (c) 2019, Systems Group, ETH Zurich.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************/

// state machines and tables, and ipv4/, udp/ and crc/

#include "mrc_transport_protocol.hpp"
#include "mrc_headers.hpp"
#include "tables/conn_table.hpp"
#include "tables/state_table.hpp"
#include "tables/msn_table.hpp"
#include "tables/transport_timer.hpp"
#include "tables/qp_interface.hpp"
#include "tables/retransmitter.hpp"
#include "rx/rx_common.hpp"
#include "rx/rx_ctrl.hpp"
#include "rx/rx_ctrl_req.hpp"
#include "rx/rx_data_res.hpp"
#include "tx/tx_common.hpp"
#include "tx/tx_data_req.hpp"


template <int WIDTH, int INSTID = 0>
void mrc_transport_protocol(	
	// RX - net module
	stream<ipUdpMeta>& s_axis_rx_meta,
	stream<net_axis<WIDTH> >& s_axis_rx_data,

	// TX - net module
	stream<ipUdpMeta>& m_axis_tx_meta,
	stream<net_axis<WIDTH> >& m_axis_tx_data,

	// S(R)Q
	stream<txMeta>& s_axis_sq_meta,

	// ACKs
	stream<ackMeta>& m_axis_rx_ack_meta,

	// RDMA
	stream<memCmd>& m_axis_mem_write_cmd,
	stream<memCmd>& m_axis_mem_read_cmd,
	stream<net_axis<WIDTH> >& m_axis_mem_write_data,
	stream<net_axis<WIDTH> >& s_axis_mem_read_data,

	// QP
	stream<qpContext>& s_axis_qp_interface,
	stream<ifConnReq>& s_axis_qp_conn_interface,

	// Debug
#ifdef DBG_IBV
	stream<psnPkg>& m_axis_dbg_0,
    stream<psnPkg>& m_axis_dbg_1,
    stream<psnPkg>& m_axis_dbg_2,
#endif
	ap_uint<32>& regInvalidPsnDropCount,
    ap_uint<32>& regRetransCount,
	ap_uint<32>& regIbvCountRx,
    ap_uint<32>& regIbvCountTx
) {
#pragma HLS INLINE

	static stream<net_axis<WIDTH> > rx_ibh2shiftFifo("rx_ibh2shiftFifo");
	static stream<net_axis<WIDTH> > rx_shift2exhFifo("rx_shift2exhFifo");
	static stream<net_axis<WIDTH> > rx_exh2dropFifo("rx_exh2dropFifo");
	static stream<net_axis<WIDTH> >	rx_ibhDrop2exhFifo("rx_ibhDrop2exhFifo");
	static stream<ibhMeta> 			rx_ibh2fsm_MetaFifo("rx_ibh2fsm_MetaFifo");
	static stream<ibhMeta>			rx_fsm2exh_MetaFifo("rx_fsm2exh_MetaFifo");
	static stream<net_axis<WIDTH> >	rx_exh2rethShiftFifo("rx_exh2rethShiftFifo");
	static stream<net_axis<WIDTH> >	rx_exhNoShiftFifo("rx_exhNoShiftFifo");
	static stream<net_axis<WIDTH> >	rx_rethSift2mergerFifo("rx_rethSift2mergerFifo");
	static stream<pkgSplit>			rx_pkgSplitTypeFifo("rx_pkgSplitTypeFifo");
	static stream<pkgShift> 		rx_pkgShiftTypeFifo("rx_pkgShiftTypeFifo");
	#pragma HLS STREAM depth=2 variable=rx_ibh2shiftFifo
	#pragma HLS STREAM depth=2 variable=rx_shift2exhFifo
	#pragma HLS STREAM depth=32 variable=rx_exh2dropFifo
	#pragma HLS STREAM depth=32 variable=rx_ibhDrop2exhFifo
	#pragma HLS STREAM depth=2 variable=rx_ibh2fsm_MetaFifo
	#pragma HLS STREAM depth=2 variable=rx_fsm2exh_MetaFifo
	#pragma HLS STREAM depth=4 variable=rx_exh2rethShiftFifo
	#pragma HLS STREAM depth=4 variable=rx_exhNoShiftFifo
	#pragma HLS STREAM depth=4 variable=rx_rethSift2mergerFifo
	#pragma HLS STREAM depth=2 variable=rx_pkgSplitTypeFifo
	#pragma HLS STREAM depth=2 variable=rx_pkgShiftTypeFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rx_ibh2fsm_MetaFifo compact=bit
	#pragma HLS aggregate  variable=rx_fsm2exh_MetaFifo compact=bit
	#pragma HLS aggregate  variable=rx_pkgSplitTypeFifo compact=bit
	#pragma HLS aggregate  variable=rx_pkgShiftTypeFifo compact=bit
#else 
	#pragma HLS DATA_PACK variable=rx_ibh2fsm_MetaFifo
	#pragma HLS DATA_PACK variable=rx_fsm2exh_MetaFifo
	#pragma HLS DATA_PACK variable=rx_pkgSplitTypeFifo
	#pragma HLS DATA_PACK variable=rx_pkgShiftTypeFifo
#endif 
	static stream<ackEvent>  rx_ibhEventFifo("rx_ibhEventFifo"); //TODO rename
	static stream<ackEvent>  rx_exhEventMetaFifo("rx_exhEventMetaFifo");
	#pragma HLS STREAM depth=2 variable=rx_ibhEventFifo
	#pragma HLS STREAM depth=2 variable=rx_exhEventMetaFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rx_ibhEventFifo compact=bit
	#pragma HLS aggregate  variable=rx_exhEventMetaFifo compact=bit
#else
	#pragma HLS DATA_PACK variable=rx_ibhEventFifo
	#pragma HLS DATA_PACK variable=rx_exhEventMetaFifo
#endif

	static stream<ibhMeta>	tx_ibhMetaFifo("tx_ibhMetaFifo");
	static stream<event>	tx_appMetaFifo("tx_appMetaFifo");
	//static stream<event>	tx_localMetaFifo("tx_localMetaFifo");
	static stream<net_axis<WIDTH> >	tx_appDataFifo("tx_appDataFifo");
	#pragma HLS STREAM depth=8 variable=tx_ibhMetaFifo
	#pragma HLS STREAM depth=32 variable=tx_appMetaFifo
	//#pragma HLS STREAM depth=8 variable=tx_localMetaFifo
	#pragma HLS STREAM depth=8 variable=tx_appDataFifo

	static stream<event> tx_exhMetaFifo("tx_exhMetaFifo");
	static stream<net_axis<WIDTH> > tx_exh2shiftFifo("tx_exh2shiftFifo");
	static stream<net_axis<WIDTH> > tx_shift2ibhFifo("tx_shift2ibhFifo");
	static stream<net_axis<WIDTH> > tx_aethShift2payFifo("tx_aethShift2payFifo");
	static stream<net_axis<WIDTH> > tx_rethShift2payFifo("tx_rethShift2payFifo");
	static stream<net_axis<WIDTH> > tx_rawPayFifo("tx_rawPayFifo");
	static stream<net_axis<WIDTH> > tx_exh2payFifo("tx_exh2payFifo");
	static stream<BaseTransportHeader<WIDTH> > tx_ibhHeaderFifo("tx_ibhHeaderFifo");
	static stream<memCmdInternal>  tx_localMemCmdFifo("tx_localMemCmdFifo");
	#pragma HLS STREAM depth=4 variable=tx_exhMetaFifo
	#pragma HLS STREAM depth=2 variable=tx_exh2shiftFifo
	#pragma HLS STREAM depth=8 variable=tx_shift2ibhFifo
	#pragma HLS STREAM depth=2 variable=tx_aethShift2payFifo
	#pragma HLS STREAM depth=2 variable=tx_rethShift2payFifo
	#pragma HLS STREAM depth=4 variable=tx_rawPayFifo
	#pragma HLS STREAM depth=4 variable=tx_exh2payFifo
	#pragma HLS STREAM depth=2 variable=tx_ibhHeaderFifo
	#pragma HLS STREAM depth=2 variable=tx_localMemCmdFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=tx_exhMetaFifo compact=bit
	#pragma HLS aggregate  variable=tx_ibhHeaderFifo compact=bit
	#pragma HLS aggregate  variable=tx_localMemCmdFifo compact=bit
#else
	#pragma HLS DATA_PACK variable=tx_exhMetaFifo
	#pragma HLS DATA_PACK variable=tx_ibhHeaderFifo
	#pragma HLS DATA_PACK variable=tx_localMemCmdFifo
#endif

	static stream<txPacketInfo>	tx_packetInfoFifo("tx_packetInfoFifo");
	static stream<ap_uint<16> > tx_lengthFifo("tx_lengthFifo");
	#pragma HLS STREAM depth=2 variable=tx_packetInfoFifo
	#pragma HLS STREAM depth=4 variable=tx_lengthFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=tx_packetInfoFifo compact=bit
#else
	#pragma HLS DATA_PACK variable=tx_packetInfoFifo
#endif

	static stream<bool> rx_ibhDropFifo("rx_ibhDropFifo");
	static stream<fwdPolicy> rx_ibhDropMetaFifo("rx_ibhDropMetaFifo");
	#pragma HLS STREAM depth=2 variable=rx_ibhDropFifo
	#pragma HLS STREAM depth=2 variable=rx_ibhDropMetaFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rx_ibhDropMetaFifo compact=bit
#else
	#pragma HLS DATA_PACK variable=rx_ibhDropMetaFifo
#endif

	//Connection Table
	static stream<ap_uint<16> >	tx_ibhconnTable_req("tx_ibhconnTable_req");
	//static stream<ifConnReq>		qpi2connTable_req("qpi2connTable_req");
	static stream<connTableEntry>	tx_connTable2ibh_rsp("tx_connTable2ibh_rsp");
	//static stream<connTableEntry> connTable2qpi_rsp("connTable2qpi_rsp");
	#pragma HLS STREAM depth=2 variable=tx_ibhconnTable_req
	#pragma HLS STREAM depth=8 variable=tx_connTable2ibh_rsp
#if defined( __VITIS_HLS__)
	//#pragma HLS aggregate  variable=tx_connTable2qpi_rsp compact=bit
#else
	//#pragma HLS DATA_PACK variable=tx_connTable2qpi_rsp
#endif

	//State Table Fifos
	static stream<rxStateReq> rxIbh2stateTable_upd_req("rxIbh2stateTable_upd_req");
	static stream<txStateReq> txIbh2stateTable_upd_req("txIbh2stateTable_upd_req");
	static stream<ifStateReq> qpi2stateTable_upd_req("qpi2stateTable_upd_req");
	static stream<rxStateRsp> stateTable2rxIbh_rsp("stateTable2rxIbh_rsp");
	static stream<stateTableEntry> stateTable2txIbh_rsp("stateTable2txIbh_rsp");
	static stream<stateTableEntry> stateTable2qpi_rsp("stateTable2qpi_rsp");
	static stream<reqStateReq> rxCtrl2stateTable_upd_req("rxCtrl2stateTable_upd_req");
	static stream<reqStateRsp> stateTable2rxCtrl_rsp("stateTable2rxCtrl_rsp");
	static stream<ap_uint<16> > stateTable2rxIbh_inv("stateTable2rxIbh_inv");
	#pragma HLS STREAM depth=4 variable=stateTable2rxIbh_inv
	#pragma HLS STREAM depth=2 variable=rxCtrl2stateTable_upd_req
	#pragma HLS STREAM depth=2 variable=stateTable2rxCtrl_rsp
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rxCtrl2stateTable_upd_req compact=bit
	#pragma HLS aggregate  variable=stateTable2rxCtrl_rsp compact=bit
#else
	#pragma HLS DATA_PACK variable=rxCtrl2stateTable_upd_req
	#pragma HLS DATA_PACK variable=stateTable2rxCtrl_rsp
#endif
	#pragma HLS STREAM depth=2 variable=rxIbh2stateTable_upd_req
	#pragma HLS STREAM depth=2 variable=txIbh2stateTable_upd_req
	#pragma HLS STREAM depth=2 variable=qpi2stateTable_upd_req
	#pragma HLS STREAM depth=2 variable=stateTable2rxIbh_rsp
	#pragma HLS STREAM depth=2 variable=stateTable2txIbh_rsp
	#pragma HLS STREAM depth=2 variable=stateTable2qpi_rsp
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rxIbh2stateTable_upd_req compact=bit
	#pragma HLS aggregate  variable=txIbh2stateTable_upd_req compact=bit
	#pragma HLS aggregate  variable=qpi2stateTable_upd_req compact=bit
	#pragma HLS aggregate  variable=stateTable2rxIbh_rsp compact=bit
	#pragma HLS aggregate  variable=stateTable2txIbh_rsp compact=bit
	#pragma HLS aggregate  variable=stateTable2qpi_rsp compact=bit
#else
	#pragma HLS DATA_PACK variable=rxIbh2stateTable_upd_req
	#pragma HLS DATA_PACK variable=txIbh2stateTable_upd_req
	#pragma HLS DATA_PACK variable=qpi2stateTable_upd_req
	#pragma HLS DATA_PACK variable=stateTable2rxIbh_rsp
	#pragma HLS DATA_PACK variable=stateTable2txIbh_rsp
	#pragma HLS DATA_PACK variable=stateTable2qpi_rsp
#endif

	// MSN Table
	static stream<rxMsnReq>		rxExh2msnTable_upd_req("rxExh2msnTable_upd_req");
	static stream<ap_uint<16> >	txExh2msnTable_req("txExh2msnTable_req");
	static stream<ifMsnReq>	if2msnTable_init("if2msnTable_init");
	static stream<dmaState>	msnTable2rxExh_rsp("msnTable2rxExh_rsp");
	static stream<txMsnRsp>	msnTable2txExh_rsp("msnTable2txExh_rsp");
	#pragma HLS STREAM depth=2 variable=rxExh2msnTable_upd_req
	#pragma HLS STREAM depth=2 variable=txExh2msnTable_req
	#pragma HLS STREAM depth=2 variable=if2msnTable_init
	#pragma HLS STREAM depth=2 variable=msnTable2rxExh_rsp
	#pragma HLS STREAM depth=2 variable=msnTable2txExh_rsp
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rxExh2msnTable_upd_req compact=bit
	#pragma HLS aggregate  variable=if2msnTable_init compact=bit
	#pragma HLS aggregate  variable=msnTable2rxExh_rsp compact=bit
	#pragma HLS aggregate  variable=msnTable2txExh_rsp compact=bit
#else
	#pragma HLS DATA_PACK variable=rxExh2msnTable_upd_req
	#pragma HLS DATA_PACK variable=if2msnTable_init
	#pragma HLS DATA_PACK variable=msnTable2rxExh_rsp
	#pragma HLS DATA_PACK variable=msnTable2txExh_rsp
#endif

	static stream<ap_uint<16> > exh_lengthFifo("exh_lengthFifo");
	static stream<ackEvent>		rx_ackEventFifo("rx_ackEventFifo");
	#pragma HLS STREAM depth=4 variable=exh_lengthFifo
	#pragma HLS STREAM depth=32 variable=rx_ackEventFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rx_ackEventFifo compact=bit
	#pragma HLS bind_storage variable=rx_ackEventFifo type=FIFO impl=LUTRAM
#else
	#pragma HLS DATA_PACK variable=rx_ackEventFifo
#endif

	// Read Req Table
    /*
	static stream<txReadReqUpdate>	tx_readReqTable_upd("tx_readReqTable_upd");
	static stream<rxReadReqUpdate>	rx_readReqTable_upd_req("rx_readReqTable_upd_req");
	static stream<rxReadReqRsp>		rx_readReqTable_upd_rsp("rx_readReqTable_upd_rsp");
	#pragma HLS STREAM depth=2 variable=tx_readReqTable_upd
	#pragma HLS STREAM depth=2 variable=rx_readReqTable_upd_req
	#pragma HLS STREAM depth=2 variable=rx_readReqTable_upd_rsp
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=tx_readReqTable_upd compact=bit
	#pragma HLS aggregate  variable=rx_readReqTable_upd_req compact=bit
	#pragma HLS aggregate  variable=rx_readReqTable_upd_rsp compact=bit
#else
	#pragma HLS DATA_PACK variable=tx_readReqTable_upd
	#pragma HLS DATA_PACK variable=rx_readReqTable_upd_req
	#pragma HLS DATA_PACK variable=rx_readReqTable_upd_rsp
#endif
    */

	// Outstanding Read Req Table
	//TODO merge these two
    /*
	static stream<mqInsertReq<ap_uint<64> > >	tx_readReqAddr_push("tx_readReqAddr_push");
	static stream<mqPopReq> rx_readReqAddr_pop_req("rx_readReqAddr_pop_req");
	static stream<ap_uint<64> > rx_readReqAddr_pop_rsp("rx_readReqAddr_pop_rsp");
	#pragma HLS STREAM depth=2 variable=tx_readReqAddr_push
	#pragma HLS STREAM depth=2 variable=rx_readReqAddr_pop_req
	#pragma HLS STREAM depth=2 variable=rx_readReqAddr_pop_rsp
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rx_readReqAddr_pop_req compact=bit
	#pragma HLS aggregate  variable=rx_readReqAddr_pop_rsp compact=bit
#else
	#pragma HLS DATA_PACK variable=rx_readReqAddr_pop_req
	#pragma HLS DATA_PACK variable=rx_readReqAddr_pop_rsp
#endif
    */

	/*
	 * TIMER & RETRANSMITTER
	 */
#ifdef RETRANS_EN
	static stream<rxTimerUpdate> rxClearTimer_req("rxClearTimer_req");
	static stream<ap_uint<24> > txSetTimer_req("txSetTimer_req");
	static stream<retransUpdate> rx2retrans_upd("rx2retrans_upd");
    static stream<retransRdInit> retrans2rx_init("retrans2rx_init");
	static stream<retransmission> rx2retrans_req("rx2retrans_req");
	static stream<retransmission> timer2retrans_req("timer2retrans_req");
	static stream<retransMeta> tx2retrans_insertMeta("tx2retrans_insertMeta");
	static stream<retransAddrLen> tx2retrans_insertAddrLen("tx2retrans_insertAddrLen");
	static stream<retransEntry>	tx2retrans_insertRequest("tx2retrans_insertRequest");
	static stream<retransEvent> retransmitter2exh_eventFifo("retransmitter2exh_eventFifo");
	#pragma HLS STREAM depth=2 variable=rxClearTimer_req
	#pragma HLS STREAM depth=2 variable=txSetTimer_req
	#pragma HLS STREAM depth=2 variable=rx2retrans_upd
    #pragma HLS STREAM depth=16 variable=retrans2rx_init
	#pragma HLS STREAM depth=2 variable=rx2retrans_req
	#pragma HLS STREAM depth=2 variable=timer2retrans_req
	#pragma HLS STREAM depth=2 variable=tx2retrans_insertMeta
	#pragma HLS STREAM depth=8 variable=tx2retrans_insertAddrLen
	#pragma HLS STREAM depth=2 variable=tx2retrans_insertRequest
	#pragma HLS STREAM depth=8 variable=retransmitter2exh_eventFifo
#endif

	//TODO this is a hack
	static stream<ap_uint<24> > tx_dstQpFifo("tx_dstQpFifo");
	#pragma HLS STREAM depth=2 variable=tx_dstQpFifo

	// Interface
	qp_interface<INSTID>(s_axis_qp_interface, stateTable2qpi_rsp, qpi2stateTable_upd_req, if2msnTable_init);


	// ------------------------------------------------------------------------------------------------
	// RX path
	// ------------------------------------------------------------------------------------------------
	// MRC control path
	static stream<mrcCtrlBthMeta>      rx_ibh2ctrl_MetaFifo("rx_ibh2ctrl_MetaFifo");
	static stream<bool>                rx_ibh2split_Fifo("rx_ibh2split_Fifo");
	static stream<bool>                rx_ibh2udpSplit_Fifo("rx_ibh2udpSplit_Fifo");
	static stream<net_axis<WIDTH> >    rx_split2dataFifo("rx_split2dataFifo");
	static stream<net_axis<WIDTH> >    rx_split2ctrlFifo("rx_split2ctrlFifo");
	static stream<ipUdpMeta>           rx_udpSplit2data_MetaFifo("rx_udpSplit2data_MetaFifo");
	static stream<ipUdpMeta>           rx_udpSplit2ctrl_MetaFifo("rx_udpSplit2ctrl_MetaFifo");
	static stream<mrcCtrlMeta<WIDTH> > rx_ctrl2fsm_MetaFifo("rx_ctrl2fsm_MetaFifo");
	static stream<mrcCtrlMeta<WIDTH> > rx_ctrl2req_MetaFifo("rx_ctrl2req_MetaFifo");
	static stream<reqErrEvent> 			rx_req2err_Fifo("rx_req2err_Fifo");
	#pragma HLS STREAM depth=4 variable=rx_req2err_Fifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate variable=rx_req2err_Fifo compact=bit
#else
	#pragma HLS DATA_PACK variable=rx_req2err_Fifo
#endif
	#pragma HLS STREAM depth=4 variable=rx_ibh2ctrl_MetaFifo
	#pragma HLS STREAM depth=4 variable=rx_ibh2split_Fifo
	#pragma HLS STREAM depth=4 variable=rx_ibh2udpSplit_Fifo
	#pragma HLS STREAM depth=2 variable=rx_split2dataFifo
	#pragma HLS STREAM depth=8 variable=rx_split2ctrlFifo
	#pragma HLS STREAM depth=8 variable=rx_udpSplit2data_MetaFifo
	#pragma HLS STREAM depth=4 variable=rx_udpSplit2ctrl_MetaFifo
	#pragma HLS STREAM depth=2 variable=rx_ctrl2fsm_MetaFifo
	#pragma HLS STREAM depth=8  variable=rx_ctrl2req_MetaFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate variable=rx_ibh2ctrl_MetaFifo compact=bit
	#pragma HLS aggregate variable=rx_udpSplit2data_MetaFifo compact=bit
	#pragma HLS aggregate variable=rx_udpSplit2ctrl_MetaFifo compact=bit
	#pragma HLS aggregate variable=rx_ctrl2fsm_MetaFifo compact=bit
#else
	#pragma HLS DATA_PACK variable=rx_ibh2ctrl_MetaFifo
	#pragma HLS DATA_PACK variable=rx_udpSplit2data_MetaFifo
	#pragma HLS DATA_PACK variable=rx_udpSplit2ctrl_MetaFifo
	#pragma HLS DATA_PACK variable=rx_ctrl2fsm_MetaFifo
#endif

	// Write-with-Immediate payload alignment
	static stream<net_axis<WIDTH> > rx_exh2rethImmShiftFifo("rx_exh2rethImmShiftFifo");
	static stream<net_axis<WIDTH> > rx_rethImmShift2mergerFifo("rx_rethImmShift2mergerFifo");
	#pragma HLS STREAM depth=4 variable=rx_exh2rethImmShiftFifo
	#pragma HLS STREAM depth=4 variable=rx_rethImmShift2mergerFifo

	static stream<mrcEthMeta> rx_ibh2exh_MetaFifo("rx_ibh2exh_MetaFifo");
	static stream<exhOpMeta>        rx_mrc2exh_MetaFifo("rx_mrc2exh_MetaFifo");
	static stream<net_axis<WIDTH> > rx_mrc2exhFifo("rx_mrc2exhFifo");
	static stream<ExHeader<WIDTH> > rx_exh2drop_MetaFifo("rx_exh2drop_MetaFifo");
	static stream<ExHeader<WIDTH> > rx_drop2exhFsm_MetaFifo("rx_drop2exhFsm_MetaFifo");
	static stream<exhMeta>	rx_exhMetaFifo("rx_exhMetaFifo");
	#pragma HLS STREAM depth=2 variable=rx_ibh2exh_MetaFifo
	#pragma HLS STREAM depth=4 variable=rx_mrc2exh_MetaFifo
	#pragma HLS STREAM depth=4 variable=rx_mrc2exhFifo
	#pragma HLS STREAM depth=8 variable=rx_exh2drop_MetaFifo
	#pragma HLS STREAM depth=2 variable=rx_drop2exhFsm_MetaFifo
	#pragma HLS STREAM depth=2 variable=rx_exhMetaFifo
#if defined( __VITIS_HLS__)
	#pragma HLS aggregate  variable=rx_ibh2exh_MetaFifo compact=bit
	#pragma HLS aggregate  variable=rx_exh2drop_MetaFifo compact=bit
	#pragma HLS aggregate  variable=rx_drop2exhFsm_MetaFifo compact=bit
	#pragma HLS aggregate  variable=rx_exhMetaFifo compact=bit
#else
	#pragma HLS DATA_PACK variable=rx_ibh2exh_MetaFifo
	#pragma HLS DATA_PACK variable=rx_exh2drop_MetaFifo
	#pragma HLS DATA_PACK variable=rx_drop2exhFsm_MetaFifo
	#pragma HLS DATA_PACK variable=rx_exhMetaFifo
#endif

	rx_process_ibh<WIDTH, INSTID>(
#ifdef DBG_IBV
		m_axis_dbg_0,
#endif 
		s_axis_rx_data, 
		rx_ibh2fsm_MetaFifo,
		rx_ibh2exh_MetaFifo,
		rx_ibh2ctrl_MetaFifo,  // NEW
		rx_ibh2split_Fifo,     // NEW
		rx_ibh2udpSplit_Fifo,  // NEW
		rx_ibh2shiftFifo,
		regIbvCountRx
	);

	rshiftWordByOctet<net_axis<WIDTH>, WIDTH, 11, ((BTH_SIZE%WIDTH)/8), INSTID>(rx_ibh2shiftFifo, rx_shift2exhFifo);

	// data / control split
	rx_split_ctrl<WIDTH, INSTID>(rx_shift2exhFifo, rx_ibh2split_Fifo, rx_split2dataFifo, rx_split2ctrlFifo);
	rx_split_udp_meta<INSTID>(s_axis_rx_meta, rx_ibh2udpSplit_Fifo, rx_udpSplit2data_MetaFifo, rx_udpSplit2ctrl_MetaFifo);

	// control path
	rx_process_ctrl<WIDTH, INSTID>(rx_split2ctrlFifo, rx_ibh2ctrl_MetaFifo, rx_udpSplit2ctrl_MetaFifo, rx_ctrl2fsm_MetaFifo);
	rx_ctrl_fsm<WIDTH, INSTID>(rx_ctrl2fsm_MetaFifo, rx_ctrl2req_MetaFifo);

	// Requestor side of the control path (mrc_rx_req.hpp)
	rx_req_fsm<WIDTH, INSTID>(
		rx_ctrl2req_MetaFifo,
		stateTable2rxCtrl_rsp,
		rxCtrl2stateTable_upd_req,
#ifdef RETRANS_EN
		rxClearTimer_req,
		rx2retrans_upd,
		rx2retrans_req,
#endif
		rx_req2err_Fifo
	);

	rx_ack_cpl<INSTID>(
#ifdef RETRANS_EN
		retrans2rx_init,
#endif
		rx_req2err_Fifo,
		m_axis_rx_ack_meta
	);

	// data path 

	rx_strip_mrc_eth<WIDTH, INSTID>(
		rx_split2dataFifo,     
		rx_ibh2exh_MetaFifo,
		rx_mrc2exh_MetaFifo,
		rx_mrc2exhFifo
	);

	rx_process_exh<WIDTH, INSTID>(
		rx_mrc2exhFifo,         
		rx_mrc2exh_MetaFifo,     
		rx_exhMetaFifo,
		rx_exh2drop_MetaFifo,
		rx_exh2dropFifo
	);

	rx_ibh_fsm<INSTID>(	
    #ifdef DBG_IBV
		m_axis_dbg_1,
#endif 
		rx_ibh2fsm_MetaFifo,
		rx_exhMetaFifo,
		stateTable2rxIbh_rsp,
		rxIbh2stateTable_upd_req,
		rx_fsm2exh_MetaFifo,
		rx_ibhEventFifo,
		rx_ibhDropFifo,
		rx_ibhDropMetaFifo,
		stateTable2rxIbh_inv,
		//m_axis_rx_ack_meta,
		regInvalidPsnDropCount
	);

	drop_ooo_ibh<WIDTH, INSTID>(
        rx_exh2dropFifo, rx_ibhDropFifo, rx_ibhDrop2exhFifo);

	//some hack TODO, make this nicer.. not sure what this is still for
	ipUdpMetaHandler<WIDTH, INSTID>(rx_udpSplit2data_MetaFifo, rx_exh2drop_MetaFifo, rx_ibhDropMetaFifo, exh_lengthFifo, rx_drop2exhFsm_MetaFifo);

	rx_exh_fsm<WIDTH, INSTID>(	
    #ifdef DBG_IBV
		m_axis_dbg_2,
#endif 
		rx_fsm2exh_MetaFifo,
		exh_lengthFifo,
		msnTable2rxExh_rsp,
		//rx_readReqAddr_pop_rsp,
		rx_drop2exhFsm_MetaFifo,
		//rx_ibhDrop2exhFifo,
		m_axis_mem_write_cmd,
		rxExh2msnTable_upd_req,
		//rx_readReqAddr_pop_req,
		rx_exhEventMetaFifo,
		//rx_rethSift2mergerFifo,
		//rx_exhNoShiftFifo,
		rx_pkgSplitTypeFifo,
		rx_pkgShiftTypeFifo
	);

	rx_exh_payload<WIDTH, INSTID>(	
		rx_pkgSplitTypeFifo,
		rx_ibhDrop2exhFifo,
		rx_exh2rethShiftFifo,
		rx_exh2rethImmShiftFifo,
		rx_exhNoShiftFifo
	);

	//TODO is order important??
	stream_merger<ackEvent>(rx_exhEventMetaFifo, rx_ibhEventFifo, rx_ackEventFifo);

	// RETH: 16 bytes
	//TODO not required for AXI_WIDTH == 64, also this seems to have a bug, this goes together with the hack in process_exh where we don't write the first word out
//#if AXI_WIDTH != 64
	rshiftWordByOctet<net_axis<WIDTH>, WIDTH, 12, ((RETH_SIZE%WIDTH)/8), INSTID>(rx_exh2rethShiftFifo, rx_rethSift2mergerFifo);
//#endif
	// RETH + ImmDt: 20 bytes (4 on 64/128-bit buses, where rx_process_exh already drops the full RETH words)
	rshiftWordByOctet<net_axis<WIDTH>, WIDTH, 14, (((RETH_SIZE + IMMDT_SIZE)%WIDTH)/8), INSTID>(rx_exh2rethImmShiftFifo, rx_rethImmShift2mergerFifo);

	merge_rx_pkgs<WIDTH, INSTID>(rx_pkgShiftTypeFifo, rx_rethSift2mergerFifo,
	                             rx_rethImmShift2mergerFifo, rx_exhNoShiftFifo, m_axis_mem_write_data);

	// ------------------------------------------------------------------------------------------------
	// TX path
	// ------------------------------------------------------------------------------------------------

	//application request handler
	static stream<pkgInfo> tx_pkgInfoFifo("tx_pkgInfoFifo");
	//static stream<net_axis<WIDTH> > tx_readDataFifo("tx_readDataFifo");
	static stream<net_axis<WIDTH> > tx_split2aethShift("tx_split2aethShift");
	static stream<net_axis<WIDTH> > tx_split2rethMerge("tx_split2rethMerge");
	static stream<net_axis<WIDTH> > tx_rethMerge2rethShift("tx_rethMerge2rethShift");
	#pragma HLS STREAM depth=128 variable=tx_pkgInfoFifo
	//#pragma HLS STREAM depth=4 variable=tx_readDataFifo
	#pragma HLS STREAM depth=4 variable=tx_split2aethShift
	#pragma HLS STREAM depth=4 variable=tx_split2rethMerge
	#pragma HLS STREAM depth=4 variable=tx_rethMerge2rethShift

	local_req_handler<INSTID>(
		s_axis_sq_meta,
#ifdef RETRANS_EN
		retransmitter2exh_eventFifo,
		tx2retrans_insertAddrLen,
#endif
		tx_localMemCmdFifo,
		//tx_readReqAddr_push,
		tx_appMetaFifo,
        regRetransCount
	);

	tx_pkg_arbiter<WIDTH, INSTID>(	
		tx_pkgInfoFifo,
		s_axis_mem_read_data,
		tx_split2aethShift,
		tx_rethMerge2rethShift,
		tx_rawPayFifo
	);

#ifdef FPGA_STANDALONE
	stream_merger(tx_split2rethMerge, tx_appDataFifo, tx_rethMerge2rethShift);
#endif
	//merges and orders event going to TX path
   	meta_merger<INSTID>(rx_ackEventFifo, tx_appMetaFifo, tx_ibhconnTable_req, tx_ibhMetaFifo, tx_exhMetaFifo);
	
	//Shift playload by 4 bytes for AETH (data from memory)
	lshiftWordByOctet<WIDTH, 12, ((AETH_SIZE%WIDTH)/8), INSTID>(tx_split2aethShift, tx_aethShift2payFifo);
	//Shift payload another 12 bytes for RETH (data from application)
	lshiftWordByOctet<WIDTH, 13, ((RETH_SIZE%WIDTH)/8), INSTID>(tx_rethMerge2rethShift, tx_rethShift2payFifo);

	//Generate EXH
	generate_exh<WIDTH, INSTID>(	
		tx_exhMetaFifo,
		msnTable2txExh_rsp,
		txExh2msnTable_req,
		//tx_readReqTable_upd,
		tx_lengthFifo,
		tx_packetInfoFifo,
#ifdef RETRANS_EN
		txSetTimer_req,
#endif
		tx_exh2payFifo
	);

	// Append payload to AETH or RETH
	append_payload<WIDTH, INSTID>(tx_packetInfoFifo, tx_exh2payFifo, tx_aethShift2payFifo, tx_rethShift2payFifo, tx_rawPayFifo, tx_exh2shiftFifo);

	// BTH: 12 bytes
	lshiftWordByOctet<WIDTH, 11, ((BTH_SIZE%WIDTH)/8), INSTID>(tx_exh2shiftFifo, tx_shift2ibhFifo);
	generate_ibh<WIDTH, INSTID>(	
		tx_ibhMetaFifo,
		tx_dstQpFifo,
		stateTable2txIbh_rsp,
		txIbh2stateTable_upd_req,
#ifdef RETRANS_EN
		tx2retrans_insertMeta,
#endif
		tx_ibhHeaderFifo
	);

	//prependt ib header
	prepend_ibh_header<WIDTH, INSTID>(tx_ibhHeaderFifo, tx_shift2ibhFifo, m_axis_tx_data, regIbvCountTx);

	//Get Meta data for UDP & IP layer
	tx_ipUdpMetaMerger(tx_connTable2ibh_rsp, tx_lengthFifo, m_axis_tx_meta, tx_dstQpFifo);

	//merge read requests
   	mem_cmd_merger<WIDTH>(tx_localMemCmdFifo, m_axis_mem_read_cmd, tx_pkgInfoFifo);

	// ------------------------------------------------------------------------------------------------
	// Data structures
	// ------------------------------------------------------------------------------------------------

	conn_table<INSTID>(	
		tx_ibhconnTable_req,
		s_axis_qp_conn_interface,
		tx_connTable2ibh_rsp
	);

 	state_table<INSTID>(rxIbh2stateTable_upd_req,
	                    rxCtrl2stateTable_upd_req,
 	                    txIbh2stateTable_upd_req,
 	                    qpi2stateTable_upd_req,
 	                    stateTable2rxIbh_rsp,
	                    stateTable2rxCtrl_rsp,
 	                    stateTable2txIbh_rsp,
 	                    stateTable2qpi_rsp,
	                    stateTable2rxIbh_inv);

	msn_table<INSTID>(
		rxExh2msnTable_upd_req,
		txExh2msnTable_req,
		if2msnTable_init,
		msnTable2rxExh_rsp,
		msnTable2txExh_rsp
	);
    /*
	read_req_table<INSTID>(	
		tx_readReqTable_upd,
#ifndef RETRANS_EN
		rx_readReqTable_upd_req
	);
#else
		rx_readReqTable_upd_req,
		rx_readReqTable_upd_rsp
	);
#endif
    */

    /*
	multi_queue<ap_uint<64>,MAX_QPS, 2048, INSTID>(	
		tx_readReqAddr_push,
		rx_readReqAddr_pop_req,
		rx_readReqAddr_pop_rsp
	);
    */

#ifdef RETRANS_EN
	merge_retrans_request(tx2retrans_insertMeta, tx2retrans_insertAddrLen, tx2retrans_insertRequest);

	transport_timer<INSTID>(
		rxClearTimer_req,
		txSetTimer_req,
		timer2retrans_req
	);

	retransmitter<INSTID>(	
		rx2retrans_upd,
        retrans2rx_init,
		rx2retrans_req,
		timer2retrans_req,
		tx2retrans_insertRequest,
		retransmitter2exh_eventFifo
	);
#endif

}

#ifdef DBG_IBV
#define mrc_transport_protocol_spec_decla(ninst)                 \
template void mrc_transport_protocol<DATA_WIDTH, ninst>(		   	\
	stream<ipUdpMeta>& s_axis_rx_meta,		                    \
	stream<net_axis<DATA_WIDTH> >& s_axis_rx_data,		        \
	stream<ipUdpMeta>& m_axis_tx_meta,		                    \
	stream<net_axis<DATA_WIDTH> >& m_axis_tx_data,		        \
	stream<txMeta>& s_axis_sq_meta,		                       	\
	stream<ackMeta>& m_axis_rx_ack_meta,		                \
	stream<memCmd>& m_axis_mem_write_cmd,		                \
	stream<memCmd>& m_axis_mem_read_cmd,		                \
	stream<net_axis<DATA_WIDTH> >& m_axis_mem_write_data,		\
	stream<net_axis<DATA_WIDTH> >& s_axis_mem_read_data,		\
	stream<qpContext>& s_axis_qp_interface,		               	\
	stream<ifConnReq>& s_axis_qp_conn_interface,		        \
	stream<psnPkg>& m_axis_dbg_0,		                        \
    stream<psnPkg>& m_axis_dbg_1,		                        \
    stream<psnPkg>& m_axis_dbg_2,		                        \
	ap_uint<32>& regInvalidPsnDropCount,		                \
    ap_uint<32>& regRetransCount,		                        \
	ap_uint<32>& regIbvCountRx,		                       	    \
    ap_uint<32>& regIbvCountTx		                       	    \
);
#else
#define mrc_transport_protocol_spec_decla(ninst)                 \
template void mrc_transport_protocol<DATA_WIDTH, ninst>(		   	\
	stream<ipUdpMeta>& s_axis_rx_meta,		                    \
	stream<net_axis<DATA_WIDTH> >& s_axis_rx_data,		        \
	stream<ipUdpMeta>& m_axis_tx_meta,		                    \
	stream<net_axis<DATA_WIDTH> >& m_axis_tx_data,		        \
	stream<txMeta>& s_axis_sq_meta,		                       	\
	stream<ackMeta>& m_axis_rx_ack_meta,		                \
	stream<memCmd>& m_axis_mem_write_cmd,		                \
	stream<memCmd>& m_axis_mem_read_cmd,		                \
	stream<net_axis<DATA_WIDTH> >& m_axis_mem_write_data,		\
	stream<net_axis<DATA_WIDTH> >& s_axis_mem_read_data,		\
	stream<qpContext>& s_axis_qp_interface,		               	\
	stream<ifConnReq>& s_axis_qp_conn_interface,		        \
	ap_uint<32>& regInvalidPsnDropCount,		                \
    ap_uint<32>& regRetransCount,		                        \
	ap_uint<32>& regIbvCountRx,		                       	    \
    ap_uint<32>& regIbvCountTx		                       	    \
);
#endif

mrc_transport_protocol_spec_decla(0);
mrc_transport_protocol_spec_decla(1);
