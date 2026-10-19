#pragma once

#include "../mrc_headers.hpp"

template <int INSTID = 0>
void qp_interface(	
	stream<qpContext>& 			contextIn,
	stream<stateTableEntry>&	stateTable2qpi_rsp,
	stream<ifStateReq>&			qpi2stateTable_upd_req,
	stream<ifMsnReq>&			if2msnTable_init
) {
#pragma HLS inline off
#pragma HLS pipeline II=1

	enum fstStateType{GET_STATE, UPD_STATE};
	static fstStateType qp_fsmState = GET_STATE;
	static qpContext context;
	stateTableEntry state;

	switch(qp_fsmState)
	{
	case GET_STATE:
		if (!contextIn.empty())
		{
			contextIn.read(context);
			qpi2stateTable_upd_req.write(context.qp_num);
			qp_fsmState = UPD_STATE;
		}
		break;
	case UPD_STATE:
		if (!stateTable2qpi_rsp.empty())
		{
			stateTable2qpi_rsp.read(state);
			//TODO check if valid transition

			// Diversify this interface: Based on the vaddr, call either one of the following: 
			if(context.virtual_address == 0xDEADBEEF) {
				// Update of the rkey: Just call the if2msnTable_init
				if2msnTable_init.write(ifMsnReq(context.qp_num, context.r_key));
			} else if (context.virtual_address == 0xFEEDBEEF) {
				// Update of the remote PSN only: Special call to the qpi2stateTable_upd_req
				qpi2stateTable_upd_req.write(ifStateReq(context.qp_num, context.remote_psn, true));
			} else {
				// Default case: Call both, conventional call 
				qpi2stateTable_upd_req.write(ifStateReq(context.qp_num, context.newState, context.remote_psn, context.local_psn));
				if2msnTable_init.write(ifMsnReq(context.qp_num, context.r_key)); //TODO store virtual address somewhere??
			}

			// qpi2stateTable_upd_req.write(ifStateReq(context.qp_num, context.newState, context.remote_psn, context.local_psn));
			// if2msnTable_init.write(ifMsnReq(context.qp_num, context.r_key)); //TODO store virtual address somewhere??
			qp_fsmState = GET_STATE;
		}
		break;
	}
}