#ifndef ULE_H

#define ULE_H

#include <stdbool.h>
#include <linux/types.h>
#include <stddef.h>

#include "ts.h"

/** protocol data unit */
typedef struct {
    unsigned char* data;
    uint16_t length;
} PDU;

/** protocol type */
typedef enum {
    IPv4,
    IPv6
} SNDUType;

/** subnetwork data unit information */
typedef struct {
    /** from the byte following the Type field of the
     *  SNDU base header up to and including the CRC
     */
    uint16_t length;
    SNDUType type; // IPv4, IPv6
    PDU pdu;
} SNDUInfo;

/** ule encapsulation context */
typedef struct {
    // TS
    uint16_t pid; // Packet ID
    uint8_t tscc; // TS continuity counter
    unsigned char tsPkt[TS_SZ]; // TS packet max size 188

    // SNDU
    uint32_t snduLen;
    unsigned char* snduPkt;
    uint32_t snduIndex;
} ULEEncapCtx;

/** ule demultiplexing context */
typedef struct {
    uint16_t pid;
    int need_pusi;				/* Set to 1, if synchronization on PUSI required. */
    unsigned char tscc;			/* TS continuity counter after sync on PUSI. */

    unsigned char *ule_skb;		/* ULE SNDU decodes into this buffer. */
    //unsigned char *ule_next_hdr;		/* Pointer into skb to next ULE extension header. */
    unsigned short ule_sndu_len;		/* ULE SNDU length in bytes, w/o D-Bit. */
    unsigned short ule_sndu_type;		/* ULE SNDU type field, complete. */
    unsigned char ule_sndu_type_1;		/* ULE SNDU type field, if split across 2 TS cells. */
    unsigned char ule_dbit;			/* Whether the DestMAC address present or not (bit is set). */
    int ule_sndu_remain;			/* Nr. of bytes still required for current ULE SNDU. */

    unsigned long ts_count;			/* Current ts cell counter. */

    unsigned char *ule_sndu_outbuf;
    unsigned short ule_sndu_outbuf_len;

    int rx_errors;
    int rx_frame_errors;
    int rx_crc_errors;
    int rx_dropped;
} ULEDemuxCtx;

static inline void ule_initEncapCtx(ULEEncapCtx* ctx)
{
    ctx->pid = 0x1FFF;
    ctx->tscc = 0;
    memset(ctx->tsPkt, 0, TS_SZ);

    ctx->snduLen = 0;
    ctx->snduPkt = NULL;
    ctx->snduIndex = 0;
}

static inline void ule_resetStats(ULEDemuxCtx* p)
{
    p->rx_errors = 0;
    p->rx_frame_errors = 0;
    p->rx_crc_errors = 0;
    p->rx_dropped = 0;
}

static inline void ule_initDemuxCtx(ULEDemuxCtx* p)
{
    p->pid = 0x1FFF;
    p->need_pusi = 1;
    p->tscc = 0;

    p->ule_skb = NULL;
    //p->ule_next_hdr = NULL;
    p->ule_sndu_len = 0;
    p->ule_sndu_type = 0;
    p->ule_sndu_type_1 = 0;
    p->ule_sndu_remain = 0;
    p->ule_dbit = 0xFF;

    p->ts_count = 0;

    p->ule_sndu_outbuf = NULL;
    p->ule_sndu_outbuf_len = 0;

    ule_resetStats(p);
}

/**
 * Destination Address Absent
 */
static inline bool ule_isDstPresent(unsigned char* header)
{
    return ((*header) & 0x80);
}
/**
 * subnetwork data unit length
 * the SNDU counted from the byte following the Type field of the SNDU base
 * header up to and including the CRC.
 */
static inline uint8_t ule_getLen(unsigned char* header)
{
    return header[0] & 0x7f;
}
/**
 * SNDU type
 */
//static inline uint16_t ule_getType(unsigned char* header)
//{
    //return (uint16_t)*(++header);
//}

/**
 * SNDU total length: header + data + crc32
 */
static inline uint32_t ule_getTotalLength(SNDUInfo* info)
{
    return 4 + info->length;
}

/**
 * @param info 
 * @param type protocol type
 * @param data protocol data unit
 * @param length protocol data unit length
 * @return error code
 */
int ule_init(SNDUInfo* info, SNDUType type, unsigned char* data, uint16_t length);

/**
 * @param info
 * @param pkt SNDU packet
 * @return error code
 */
int ule_encode(SNDUInfo* info, unsigned char* pkt, size_t pktLength);

int ule_padding(ULEEncapCtx* ctx);

void ule_demux(ULEDemuxCtx* priv , const unsigned char *buf, size_t buf_len);

void ule_resetDemuxCtx(ULEDemuxCtx* p);

#endif /* end of include guard: ULE_H */
