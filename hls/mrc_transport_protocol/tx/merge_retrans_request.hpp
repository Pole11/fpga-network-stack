#pragma once

#include "../mrc_headers.hpp"
#include "../tables/retransmitter/retransmitter_types.hpp"

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
