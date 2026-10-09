/************************************************
 * MRC control-plane extension headers (OCP MRC 1.0, chapter 7)
 *
 *   0xDC  Reliability SACK           SETH + CC_STATE   (7.5.5.4, 7.5.5.5)
 *   0xDD  Reliability NACK           NETH              (7.5.5.6)
 *   0xDE  Reliability PROBE Request  PETH              (7.5.5.7)
 *   0xD8  Endpoint Request           ERTH              (7.5.6.1)
 *   0xD9  Endpoint Response          EETH              (7.5.6.2)
 *
 * All of them are header-only packets: [BTH][xETH] and nothing else (no payload).
 *
 * Wire convention (same as the rest of the stack):
 *   byte b of a header lives in header(8b+7, 8b), bit 7 of a byte is the first bit on the wire,
 *   multi-byte fields are big-endian and are read with reverse().
 *   Sub-byte fields that straddle a byte boundary are extracted from the big-endian 16-bit value
 *   (e.g. the first two bytes: type[15:12] nh[11:7] ...), which matches the spec drawings.
 ************************************************/
#pragma once

#include "ib_transport_protocol.hpp"

// ------------------------------------------------------------------------------------------------
// Sizes (bits)
// ------------------------------------------------------------------------------------------------
const uint32_t SETH_SIZE         = 224;                       // 28 B, Table 7-14 without CC_STATE
const uint32_t CC_STATE_SIZE     = 64;                        //  8 B, Table 7-12
const uint32_t SACK_SIZE         = SETH_SIZE + CC_STATE_SIZE; // 36 B, SETH is always followed by CC_STATE
const uint32_t NETH_SIZE         = 160;                       // 20 B, Table 7-16
const uint32_t PETH_SIZE         = 128;                       // 16 B, Table 7-18
const uint32_t ERTH_SIZE         = 128;                       // 16 B, Table 7-20
const uint32_t EETH_SIZE         = 288;                       // 36 B, Table 7-22
const uint32_t MRC_CTRL_HDR_SIZE = 288;                       // largest of the above

// thanks claude for this check
static_assert(MRC_CTRL_HDR_SIZE >= SACK_SIZE, "MRC_CTRL_HDR_SIZE too small for SETH+CC_STATE");
static_assert(MRC_CTRL_HDR_SIZE >= NETH_SIZE, "MRC_CTRL_HDR_SIZE too small for NETH");
static_assert(MRC_CTRL_HDR_SIZE >= PETH_SIZE, "MRC_CTRL_HDR_SIZE too small for PETH");
static_assert(MRC_CTRL_HDR_SIZE >= ERTH_SIZE, "MRC_CTRL_HDR_SIZE too small for ERTH");
static_assert(MRC_CTRL_HDR_SIZE >= EETH_SIZE, "MRC_CTRL_HDR_SIZE too small for EETH");

// ------------------------------------------------------------------------------------------------
// Field encodings
// ------------------------------------------------------------------------------------------------

// SETH.m (Table 7-15)
typedef enum {
	SACK_M_NONE        = 0x0,
	SACK_M_SKIP        = 0x1, // ECN marked, skip this path once
	SACK_M_ALWAYS_SKIP = 0x2  // bad path
} mrcSackMark;

// NETH.nack_reason (Table 7-17)
typedef enum {
	NACK_TRIMMED         = 0x01,
	NACK_TRIMMED_LASTHOP = 0x02,
	NACK_NO_BITMAP       = 0x06,
	NACK_NO_PKT_BUFFER   = 0x07,
	NACK_NO_RESOURCE     = 0x0A,
	NACK_PSN_OOR_WINDOW  = 0x0B,
	NACK_UNEXP_EVENT     = 0x19
} mrcNackReason;

// ERTH.op / EETH.op (Tables 7-21, 7-23)
typedef enum {
	EP_OP_PORT_STATUS = 0x0,
	EP_OP_EV_PROBE    = 0x1
} mrcEpOp;

const ap_uint<4>  MRC_CC_TYPE_DEFAULT = 0x0; // CC_STATE layout of Table 7-12
const ap_uint<4>  MRC_FTYPE_TS        = 0x1; // ftype when tx_timestamp is carried
const ap_uint<24> MRC_EV_PROBE_DQP    = 0x2; // BTH.dqp of EV probes (Table 7-6)

// Bytes each control header occupies on the wire (0 = not a control opcode)
inline ap_uint<6> mrcCtrlHeaderBytes(mrcOpCode op)
{
#pragma HLS inline
	switch (op)
	{
	case MRC_SACK:      return SACK_SIZE / 8;
	case MRC_NACK:      return NETH_SIZE / 8;
	case MRC_PROBE_REQ: return PETH_SIZE / 8;
	case MRC_EP_REQ:    return ERTH_SIZE / 8;
	case MRC_EP_RSP:    return EETH_SIZE / 8;
	default:            return 0;
	}
}

// ------------------------------------------------------------------------------------------------
// Byte accessors (b = byte offset inside the header, i = bit inside the byte, 7 = MSB)
// ------------------------------------------------------------------------------------------------
template <int N> inline ap_uint<8>  mrcGet8 (ap_uint<N>& h, int b) { return h(8*b+7, 8*b); }
template <int N> inline ap_uint<16> mrcGet16(ap_uint<N>& h, int b) { return reverse((ap_uint<16>) h(8*b+15, 8*b)); }
template <int N> inline ap_uint<24> mrcGet24(ap_uint<N>& h, int b) { return reverse((ap_uint<24>) h(8*b+23, 8*b)); }
template <int N> inline ap_uint<32> mrcGet32(ap_uint<N>& h, int b) { return reverse((ap_uint<32>) h(8*b+31, 8*b)); }
template <int N> inline ap_uint<64> mrcGet64(ap_uint<N>& h, int b) { return reverse((ap_uint<64>) h(8*b+63, 8*b)); }
template <int N> inline bool        mrcGetBit(ap_uint<N>& h, int b, int i) { return h[8*b+i]; }

// ------------------------------------------------------------------------------------------------
// SETH + CC_STATE (opcode 0xDC), 36 bytes
//
//  0-1  type[15:12] nh[11:7] m[6:5] r r r pr[1] r
//  2-3  ack_psn_offset (signed) / probe_id when pr=1
//  4-7  entropy
//  8-9  spdcid          10-11 dpdcid
//  12   res             13-15 cack_psn
//  16   cc_type[7:4] cc_fl[3:0]
//  17   mpr             18-19 sack_offset (signed)
//  20-27 sack_bitmap[63:0]
//  28-35 CC_STATE: 28-29 tx_timestamp, 30-31 r|ooo_count[14:0], 32 r_c|rcv_cwnd_pen[6:0], 33-35 rcvd_bytes
//
// NOTE: Table 7-15 lists an extra "res 6" before cc_type that does not fit the 32-bit row of the
// figure (Table 7-14). This layout follows the figure.
// ------------------------------------------------------------------------------------------------
template <int W>
class SackExHeader : public packetHeader<W, SACK_SIZE>
{
	using packetHeader<W, SACK_SIZE>::header;

public:
	SackExHeader() {}

	ap_uint<2>  getMark()            { return mrcGet16(header, 0)(6, 5); }
	bool        isProbeResponse()    { return mrcGet16(header, 0)[1]; }
	ap_int<16>  getAckPsnOffset()    { return (ap_int<16>) mrcGet16(header, 2); }
	ap_uint<16> getProbeId()         { return mrcGet16(header, 2); } // same field, valid when pr=1
	ap_uint<32> getEntropy()         { return mrcGet32(header, 4); }
	ap_uint<16> getSpdcid()          { return mrcGet16(header, 8); }
	ap_uint<16> getDpdcid()          { return mrcGet16(header, 10); }
	ap_uint<24> getCackPsn()         { return mrcGet24(header, 13); }
	ap_uint<4>  getCcType()          { return mrcGet8(header, 16)(7, 4); }
	ap_uint<4>  getCcFlags()         { return mrcGet8(header, 16)(3, 0); }
	ap_uint<8>  getMpr()             { return mrcGet8(header, 17); }
	ap_int<16>  getSackOffset()      { return (ap_int<16>) mrcGet16(header, 18); }
	ap_uint<64> getSackBitmap()      { return mrcGet64(header, 20); }

	// ack_psn = cack_psn + ack_psn_offset, sack base = cack_psn + sack_offset (both mod 2^24)
	ap_uint<24> getAckPsn()          { ap_int<24> o = getAckPsnOffset(); return getCackPsn() + (ap_uint<24>) o; }
	ap_uint<24> getSackBasePsn()     { ap_int<24> o = getSackOffset();   return getCackPsn() + (ap_uint<24>) o; }

	// CC_STATE, layout valid for cc_type == MRC_CC_TYPE_DEFAULT
	ap_uint<64> getRawCcState()      { return header(SACK_SIZE-1, SETH_SIZE); }
	ap_uint<16> getCcTxTimestamp()   { return mrcGet16(header, 28); }
	ap_uint<15> getOooCount()        { return mrcGet16(header, 30)(14, 0); }
	bool        getRestoreCwnd()     { return mrcGetBit(header, 32, 7); }
	ap_uint<7>  getRcvCwndPenalty()  { return mrcGet8(header, 32)(6, 0); }
	ap_uint<24> getRcvdBytes()       { return mrcGet24(header, 33); } // 256 B units
};

// ------------------------------------------------------------------------------------------------
// NETH (opcode 0xDD), 20 bytes
//
//  0-1  type nh reserved     2 nack_reason     3 vendor_info
//  4-7  entropy
//  8-9  spdcid               10-11 dpdcid
//  12   res                  13-15 nack_psn
//  16   cc_type[7:4] cc_fl[3:0]   17 reserved  18-19 tx_timestamp
//
// NOTE: the UDP length of a NACK carries the length of the trimmed packet (7.5.5.6),
// not the length of the NACK itself.
// ------------------------------------------------------------------------------------------------
template <int W>
class NackExHeader : public packetHeader<W, NETH_SIZE>
{
	using packetHeader<W, NETH_SIZE>::header;

public:
	NackExHeader() {}

	ap_uint<8>  getNackReason()  { return mrcGet8(header, 2); }
	ap_uint<8>  getVendorInfo()  { return mrcGet8(header, 3); }
	ap_uint<32> getEntropy()     { return mrcGet32(header, 4); }
	ap_uint<16> getSpdcid()      { return mrcGet16(header, 8); }
	ap_uint<16> getDpdcid()      { return mrcGet16(header, 10); }
	ap_uint<24> getNackPsn()     { return mrcGet24(header, 13); }
	ap_uint<4>  getCcType()      { return mrcGet8(header, 16)(7, 4); }
	ap_uint<4>  getCcFlags()     { return mrcGet8(header, 16)(3, 0); }
	ap_uint<16> getTxTimestamp() { return mrcGet16(header, 18); }
};

// ------------------------------------------------------------------------------------------------
// PETH (opcode 0xDE), 16 bytes
//
//  0-1  type nh reserved     2 reserved        3 vendor_info
//  4-5  probe_id             6-7 reserved
//  8-9  spdcid               10-11 dpdcid
//  12-13 tx_timestamp        14 tsr[7] ...     15 ...ftype[3:0]
// ------------------------------------------------------------------------------------------------
template <int W>
class ProbeExHeader : public packetHeader<W, PETH_SIZE>
{
	using packetHeader<W, PETH_SIZE>::header;

public:
	ProbeExHeader() {}

	ap_uint<8>  getVendorInfo()  { return mrcGet8(header, 3); }
	ap_uint<16> getProbeId()     { return mrcGet16(header, 4); }
	ap_uint<16> getSpdcid()      { return mrcGet16(header, 8); }
	ap_uint<16> getDpdcid()      { return mrcGet16(header, 10); }
	ap_uint<16> getTxTimestamp() { return mrcGet16(header, 12); }
	bool        getTsr()         { return mrcGetBit(header, 14, 7); }
	ap_uint<4>  getFtype()       { return mrcGet8(header, 15)(3, 0); }
	bool        hasTimestamp()   { return getFtype() == MRC_FTYPE_TS; }
};

// ------------------------------------------------------------------------------------------------
// ERTH (opcode 0xD8), 16 bytes
//
//  0-1  type nh reserved op[1:0]   2 reserved   3 vendor_info
//  4-7  port_status_mask
//  8-11 reserved
//  12-13 tx_timestamp        14 tsr[7] ...     15 ...ftype[3:0]
// ------------------------------------------------------------------------------------------------
template <int W>
class EpReqExHeader : public packetHeader<W, ERTH_SIZE>
{
	using packetHeader<W, ERTH_SIZE>::header;

public:
	EpReqExHeader() {}

	ap_uint<2>  getOp()             { return mrcGet16(header, 0)(1, 0); }
	ap_uint<8>  getVendorInfo()     { return mrcGet8(header, 3); }
	ap_uint<32> getPortStatusMask() { return mrcGet32(header, 4); } // only for EP_OP_PORT_STATUS
	ap_uint<16> getTxTimestamp()    { return mrcGet16(header, 12); }
	bool        getTsr()            { return mrcGetBit(header, 14, 7); }
	ap_uint<4>  getFtype()          { return mrcGet8(header, 15)(3, 0); }
	bool        hasTimestamp()      { return getFtype() == MRC_FTYPE_TS; }
};

// ------------------------------------------------------------------------------------------------
// EETH (opcode 0xD9), 36 bytes
//
//  0-1  type[15:12] nh[11:7] r[6:4] op[3:2] r[1:0]    2-3 reserved
//  4-27 reserved
//  28-29 tx_timestamp        30-31 reserved
//  32-35 reserved
//
// NOTE: Table 7-23 lists "reserved 3, op 2" and stops there; the figure (Table 7-22) shows an
// extra 2-bit "r" after op, which places op at [3:2]. This layout follows the figure.
// ------------------------------------------------------------------------------------------------
template <int W>
class EpRspExHeader : public packetHeader<W, EETH_SIZE>
{
	using packetHeader<W, EETH_SIZE>::header;

public:
	EpRspExHeader() {}

	ap_uint<2>  getOp()          { return mrcGet16(header, 0)(3, 2); }
	ap_uint<16> getTxTimestamp() { return mrcGet16(header, 28); }
};

// ------------------------------------------------------------------------------------------------
// rx_process_ibh -> rx_process_ctrl: BTH fields of a control packet
//
// Meaning of dest_qp / psn depends on the opcode (Table 7-6):
//   SACK      dqp = requestor QP, psn = cack_psn
//   NACK      dqp = requestor QP, psn = psn of the packet that triggered the NACK
//   PROBE     dqp = peer QP,      psn not consumed
//   EP REQ/RSP dqp = 0x2,         psn[15:0] = requestor-private probe id (reflected in the response)
// ------------------------------------------------------------------------------------------------
struct mrcCtrlBthMeta
{
	mrcOpCode   op_code;
	ap_uint<16> partition_key;
	ap_uint<24> dest_qp;
	ap_uint<24> psn;
	bool        ackReq;
	bool        rtx;
	mrcCtrlBthMeta() : op_code(MRC_SACK), ackReq(false), rtx(false) {}
	mrcCtrlBthMeta(mrcOpCode op, ap_uint<16> key, ap_uint<24> qp, ap_uint<24> psn, bool a, bool rtx)
		: op_code(op), partition_key(key), dest_qp(qp), psn(psn), ackReq(a), rtx(rtx) {}
};

// ------------------------------------------------------------------------------------------------
// rx_process_ctrl -> rx_ctrl_fsm: one fully parsed control packet
//
// The header is kept raw (like ExHeader does for RETH/AETH) and typed views are built on demand,
// so a single FIFO carries every control header type.
// ------------------------------------------------------------------------------------------------
template <int W>
struct mrcCtrlMeta
{
	// BTH
	mrcOpCode    op_code;
	ap_uint<16>  partition_key;
	ap_uint<24>  dest_qp;
	ap_uint<24>  psn;
	bool         ackReq;
	bool         rtx;
	// UDP / IP
	ap_uint<16>  udpLength;
	ap_uint<128> srcAddress; // endpoint operations are not bound to a QP, the reply goes back here
	ap_uint<16>  srcPort;
	// parsing result
	bool         lengthOk;   // UDP length is large enough to hold the header of this opcode
	ap_uint<MRC_CTRL_HDR_SIZE> raw;

	mrcCtrlMeta() : op_code(MRC_SACK), ackReq(false), rtx(false), lengthOk(false) {}

	SackExHeader<W> getSackHeader()
	{
		SackExHeader<W> h;
		h.setRawHeader((ap_uint<SACK_SIZE>) raw(SACK_SIZE-1, 0));
		return h;
	}
	NackExHeader<W> getNackHeader()
	{
		NackExHeader<W> h;
		h.setRawHeader((ap_uint<NETH_SIZE>) raw(NETH_SIZE-1, 0));
		return h;
	}
	ProbeExHeader<W> getProbeHeader()
	{
		ProbeExHeader<W> h;
		h.setRawHeader((ap_uint<PETH_SIZE>) raw(PETH_SIZE-1, 0));
		return h;
	}
	EpReqExHeader<W> getEpReqHeader()
	{
		EpReqExHeader<W> h;
		h.setRawHeader((ap_uint<ERTH_SIZE>) raw(ERTH_SIZE-1, 0));
		return h;
	}
	EpRspExHeader<W> getEpRspHeader()
	{
		EpRspExHeader<W> h;
		h.setRawHeader((ap_uint<EETH_SIZE>) raw(EETH_SIZE-1, 0));
		return h;
	}
};