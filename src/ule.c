#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/slab.h>


#include "ts.h"
#include "ule.h"
#include "crc.h"
#include "debug.h"

typedef unsigned char byte;

static const uint16_t SNDUTypeValue[] = {
    0x0800, /* IPv4 */
    0x86dd, /* IPv6 */
};

#define TS_SZ	188
#define TS_SYNC	0x47
#define TS_TEI	0x80
#define TS_SC	0xC0
#define TS_PUSI	0x40
#define TS_AF_A	0x20
#define TS_AF_D	0x10

#define LOG_WARN    "[WARN ] "
#define LOG_ERR     "[ERROR] "
#define LOG_INFO    "[INFO ] "

static void hexdump( const unsigned char *buf, unsigned short len );

static inline void* xmalloc(size_t size)
{
    return kmalloc(size, GFP_KERNEL);
}
static inline void xfree(void* p)
{
    kfree(p);
}

int ule_init(SNDUInfo* info, SNDUType type, unsigned char* data, uint16_t length)
{
    // TODO: check length max size
    info->length = length + 4; // 4:crc32
    info->type = type;
    info->pdu.data = data;
    info->pdu.length = length;
    return 0;
}

int ule_encode(SNDUInfo* info, unsigned char* pkt, size_t pktLength)
{
    uint16_t* header;
    uint32_t crc;

    // TODO: d bit
    memset(pkt, 0, pktLength);
    header = (uint16_t*) pkt;
    // set length
    header[0] = htons(info->length);
    // set protocol type
    header[1] = htons(SNDUTypeValue[info->type]);
    // set protocol data unit
    memcpy((void*) &header[2], info->pdu.data, info->pdu.length);
    // set crc32
    crc = crc32(pkt, pktLength - 4);
    *((uint32_t*) (pkt + pktLength - 4)) = htonl(crc);
    
    return 0;
}

int ule_padding(ULEEncapCtx* ctx)
{
    byte* pp;
    uint8_t remaining;
    uint8_t maxLen;

    ts_reset(ctx->tsPkt);
    ts_setPID(ctx->tsPkt, ctx->pid);
    ts_setContinuityCounter(ctx->tsPkt, ctx->tscc);
    ctx->tscc = (ctx->tscc + 1) & 0x0F; // inc count

    pp = ctx->tsPkt + 4; // payload pointer
    remaining = TS_SZ - 4; // ts pkt remaining bytes

    // contain the start of an SNDU
    if (ctx->snduIndex == 0) {
        ts_setPUSI(ctx->tsPkt, true);
        ts_setPayloadPointer(ctx->tsPkt, 0);
        remaining--;
        pp++;
    }
    maxLen = min(ctx->snduLen - ctx->snduIndex, (uint32_t) remaining);
    memcpy((void*) pp, (void*) (ctx->snduPkt + ctx->snduIndex), maxLen);
    ctx->snduIndex += maxLen; // inc sndu index

//printf("%d, %d\n", remaining, maxLen);
    // All unused bytes MUST be set to the value of 0xFF
    if (maxLen < remaining) {
        memset((void*) (pp + maxLen), 0xFF, remaining - maxLen);
    }
    return 0;
}

/** Prepare for a new ULE SNDU: reset the decoder state. */
static inline void reset_ule(ULEDemuxCtx *p)
{
	p->ule_skb = NULL;
	//p->ule_next_hdr = NULL;
	p->ule_sndu_len = 0;
	p->ule_sndu_type = 0;
	p->ule_sndu_type_1 = 0;
	p->ule_sndu_remain = 0;
	p->ule_dbit = 0xFF;
}

/**
 * Decode ULE SNDUs according to draft-ietf-ipdvb-ule-03.txt from a sequence of
 * TS cells of a single PID.
 */
void ule_demux(ULEDemuxCtx* priv , const unsigned char *buf, size_t buf_len)
{
    //struct dvb_net_priv *priv = netdev_priv(dev);
    unsigned long skipped = 0L;
    const unsigned char *ts, *ts_end, *from_where = NULL;
    uint8_t ts_remain = 0, how_much = 0, new_ts = 1;
    //struct ethhdr *ethh = NULL;
    bool error = false;

    ule_resetStats(priv);

    /* For all TS cells in current buffer.
     * Appearently, we are called for every single TS cell.
     */
    for (ts = buf, ts_end = buf + buf_len; ts < ts_end; /* no default incr. */ ) {

        if (new_ts) {
            /* We are about to process a new TS cell. */

            /* Check TS error conditions: sync_byte, transport_error_indicator, scrambling_control . */
            if ((ts[0] != TS_SYNC) || (ts[1] & TS_TEI) || ((ts[3] & TS_SC) != 0)) {
                printk(KERN_WARNING "%lu: Invalid TS cell: SYNC %#x, TEI %u, SC %#x.\n",
                        priv->ts_count, ts[0], ts[1] & TS_TEI >> 7, ts[3] & 0xC0 >> 6);

                /* Drop partly decoded SNDU, reset state, resync on PUSI. */
                if (priv->ule_skb) {
                    xfree( priv->ule_skb );
                    /* Prepare for next SNDU. */

                    priv->rx_errors++;
                }
                reset_ule(priv);
                priv->need_pusi = 1;

                /* Continue with next TS cell. */
                ts += TS_SZ;
                priv->ts_count++;
                continue;
            }

            ts_remain = 184;
            from_where = ts + 4;
        }
        /* Synchronize on PUSI, if required. */
        if (priv->need_pusi) {
            if (ts[1] & TS_PUSI) {
                /* Find beginning of first ULE SNDU in current TS cell. */
                /* Synchronize continuity counter. */
                priv->tscc = ts[3] & 0x0F;
                /* There is a pointer field here. */
                if (ts[4] > ts_remain) {
                    printk(KERN_ERR "%lu: Invalid ULE packet "
                            "(pointer field %d)\n", priv->ts_count, ts[4]);
                    ts += TS_SZ;
                    priv->ts_count++;
                    continue;
                }
                /* Skip to destination of pointer field. */
                from_where = &ts[5] + ts[4];
                ts_remain -= 1 + ts[4];
                skipped = 0;
            } else {
                skipped++;
                ts += TS_SZ;
                priv->ts_count++;
                continue;
            }
        }

        if (new_ts) {
            /* Check continuity counter. */
            if ((ts[3] & 0x0F) == priv->tscc)
                priv->tscc = (priv->tscc + 1) & 0x0F;
            else {
                /* TS discontinuity handling: */
                printk(KERN_WARNING "%lu: TS discontinuity: got %#x, "
                        "expected %#x.\n", priv->ts_count, ts[3] & 0x0F, priv->tscc);
                /* Drop partly decoded SNDU, reset state, resync on PUSI. */
                if (priv->ule_skb) {
                    xfree( priv->ule_skb );
                    /* Prepare for next SNDU. */
                    // reset_ule(priv);  moved to below.

                    priv->rx_errors++;
                }
                reset_ule(priv);
                /* skip to next PUSI. */
                priv->need_pusi = 1;
                continue;
            }
            /* If we still have an incomplete payload, but PUSI is
             * set; some TS cells are missing.
             * This is only possible here, if we missed exactly 16 TS
             * cells (continuity counter wrap). */
            if (ts[1] & TS_PUSI) {
                if (! priv->need_pusi) {
                    if (!(*from_where < (ts_remain-1)) || *from_where != priv->ule_sndu_remain) {
                        /* Pointer field is invalid.  Drop this TS cell and any started ULE SNDU. */
                        printk(KERN_WARNING "%lu: Invalid pointer "
                                "field: %u.\n", priv->ts_count, *from_where);

                        /* Drop partly decoded SNDU, reset state, resync on PUSI. */
                        if (priv->ule_skb) {
                            error = true;
                            xfree(priv->ule_skb);
                        }

                        if (error || priv->ule_sndu_remain) {
                            priv->rx_errors++;
                            error = false;
                        }

                        reset_ule(priv);
                        priv->need_pusi = 1;
                        continue;
                    }
                    /* Skip pointer field (we're processing a
                     * packed payload). */
                    from_where += 1;
                    ts_remain -= 1;
                } else
                    priv->need_pusi = 0;

                if (priv->ule_sndu_remain > 183) {
                    /* Current SNDU lacks more data than there could be available in the
                     * current TS cell. */
                    //TODO: error rate
                    //dev->stats.rx_errors++;
                    //dev->stats.rx_length_errors++;
                    printk(KERN_WARNING "%lu: Expected %d more SNDU bytes, but "
                            "got PUSI (pf %d, ts_remain %d).  Flushing incomplete payload.\n",
                            priv->ts_count, priv->ule_sndu_remain, ts[4], ts_remain);
                    xfree(priv->ule_skb);
                    /* Prepare for next SNDU. */
                    reset_ule(priv);
                    /* Resync: go to where pointer field points to: start of next ULE SNDU. */
                    from_where += ts[4];
                    ts_remain -= ts[4];
                }
            }
        }

        /* Check if new payload needs to be started. */
        if (priv->ule_skb == NULL) {
            /* Start a new payload with skb.
             * Find ULE header.  It is only guaranteed that the
             * length field (2 bytes) is contained in the current
             * TS.
             * Check ts_remain has to be >= 2 here. */
            if (ts_remain < 2) {
                printk(KERN_WARNING "Invalid payload packing: only %d "
                        "bytes left in TS.  Resyncing.\n", ts_remain);
                priv->ule_sndu_len = 0;
                priv->need_pusi = 1;
                ts += TS_SZ;
                continue;
            }

            if (! priv->ule_sndu_len) {
                /* Got at least two bytes, thus extrace the SNDU length. */
                priv->ule_sndu_len = from_where[0] << 8 | from_where[1];
                if (priv->ule_sndu_len & 0x8000) {
                    /* D-Bit is set: no dest mac present. */
                    priv->ule_sndu_len &= 0x7FFF;
                    priv->ule_dbit = 1;
                } else
                    priv->ule_dbit = 0;

                if (priv->ule_sndu_len < 5) {
                    printk(KERN_WARNING "%lu: Invalid ULE SNDU length %u. "
                            "Resyncing.\n", priv->ts_count, priv->ule_sndu_len);
                    priv->rx_errors++;
                    priv->ule_sndu_len = 0;
                    priv->need_pusi = 1;
                    new_ts = 1;
                    ts += TS_SZ;
                    priv->ts_count++;
                    continue;
                }
                ts_remain -= 2;	/* consume the 2 bytes SNDU length. */
                from_where += 2;
            }

            priv->ule_sndu_remain = priv->ule_sndu_len + 2;
            /*
             * State of current TS:
             *   ts_remain (remaining bytes in the current TS cell)
             *   0	ule_type is not available now, we need the next TS cell
             *   1	the first byte of the ule_type is present
             * >=2	full ULE header present, maybe some payload data as well.
             */
            switch (ts_remain) {
                case 1:
                    priv->ule_sndu_remain--;
                    priv->ule_sndu_type = from_where[0] << 8;
                    priv->ule_sndu_type_1 = 1; /* first byte of ule_type is set. */
                    ts_remain -= 1; from_where += 1;
                    /* Continue w/ next TS. */
                case 0:
                    new_ts = 1;
                    ts += TS_SZ;
                    priv->ts_count++;
                    continue;

                default: /* complete ULE header is present in current TS. */
                    /* Extract ULE type field. */
                    if (priv->ule_sndu_type_1) {
                        priv->ule_sndu_type_1 = 0;
                        priv->ule_sndu_type |= from_where[0];
                        from_where += 1; /* points to payload start. */
                        ts_remain -= 1;
                    } else {
                        /* Complete type is present in new TS. */
                        priv->ule_sndu_type = from_where[0] << 8 | from_where[1];
                        from_where += 2; /* points to payload start. */
                        ts_remain -= 2;
                    }
                    break;
            }

            /* Allocate the skb (decoder target buffer) with the correct size, as follows:
             * prepare for the largest case: bridged SNDU with MAC address (dbit = 0). */
            //priv->ule_skb = xmalloc( priv->ule_sndu_len + ETH_HLEN + ETH_ALEN );
            //XXX: without MAC address
            priv->ule_skb = xmalloc(priv->ule_sndu_len);
            if (priv->ule_skb == NULL) {
                printk(KERN_INFO "Memory squeeze, dropping packet.\n");
                priv->rx_dropped++;
                return;
            }

            /* This includes the CRC32 _and_ dest mac, if !dbit. */
            priv->ule_sndu_remain = priv->ule_sndu_len;

            //priv->ule_skb->dev = dev;
            ///* Leave space for Ethernet or bridged SNDU header (eth hdr plus one MAC addr). */
            //skb_reserve( priv->ule_skb, ETH_HLEN + ETH_ALEN );
        }

        /* Copy data into our current skb. */
        how_much = min(priv->ule_sndu_remain, (int) ts_remain);
        memcpy(priv->ule_skb + priv->ule_sndu_len - priv->ule_sndu_remain, from_where, how_much);
        priv->ule_sndu_remain -= how_much;
        ts_remain -= how_much;
        from_where += how_much;

        /* Check for complete payload. */
        if (priv->ule_sndu_remain <= 0) {
            /* TODO: Check CRC32, we've got it in our skb already. */
            uint16_t ulen = htons(priv->ule_sndu_len);
            uint16_t utype = htons(priv->ule_sndu_type);
            //const uint8_t *tail;

            uint32_t ule_crc = ~0L, expected_crc = ~0L;
            unsigned char uleHeader[4];

            if (priv->ule_dbit) {
                /* Set D-bit for CRC32 verification,
                 * if it was set originally. */
                ulen |= htons(0x8000);
            }

            *((uint16_t*) uleHeader) = ulen;
            *((uint16_t*) (uleHeader + 2)) = utype;

            // calculate crc
            ule_crc = crc32ForUle(uleHeader, priv->ule_skb, priv->ule_sndu_len - 4);

            // get crc from payload
            expected_crc = *(uint32_t*)(priv->ule_skb + priv->ule_sndu_len - 4);
            expected_crc = ntohl(expected_crc);

            if (ule_crc != expected_crc) {
                printk(KERN_WARNING "%lu: CRC32 check FAILED: %08x / %08x, SNDU len %d type %#x, ts_remain %d, next 2: %x.\n",
                        priv->ts_count, ule_crc, expected_crc, priv->ule_sndu_len, priv->ule_sndu_type, ts_remain, ts_remain > 2 ? *(unsigned short *)from_where : 0);

                priv->rx_errors++;
                priv->rx_crc_errors++;
                hexdump(buf, buf_len);
                xfree(priv->ule_skb);
            } else {
                /* CRC32 verified OK. */

                /* Stuff into kernel's protocol stack. */
                //priv->ule_skb->protocol = dvb_net_eth_type_trans(priv->ule_skb, dev);
                /* If D-bit is set (i.e. destination MAC address not present),
                 * receive the packet anyhow. */
                /* if (priv->ule_dbit && skb->pkt_type == PACKET_OTHERHOST)
                   priv->ule_skb->pkt_type = PACKET_HOST; */
                //dev->stats.rx_packets++;
                //dev->stats.rx_bytes += priv->ule_skb->len;
                /* CRC32 was OK. Remove it from skb. */
                priv->ule_sndu_outbuf = priv->ule_skb;
                priv->ule_sndu_outbuf_len = priv->ule_sndu_len - 4;// 4:CRC
            }
            //sndu_done:
            /* Prepare for next SNDU. */
            reset_ule(priv);
        }

        /* More data in current TS (look at the bytes following the CRC32)? */
        if (ts_remain >= 2 && *((unsigned short *)from_where) != 0xFFFF) {
            /* Next ULE SNDU starts right there. */
            new_ts = 0;
            priv->ule_skb = NULL;
            priv->ule_sndu_type_1 = 0;
            priv->ule_sndu_len = 0;
            // printf(LOG_WARN "More data in current TS: [%#x %#x %#x %#x]\n",
            //	*(from_where + 0), *(from_where + 1),
            //	*(from_where + 2), *(from_where + 3));
            // printf(LOG_WARN "ts @ %p, stopped @ %p:\n", ts, from_where + 0);
            // hexdump(ts, 188);
        } else {
            new_ts = 1;
            ts += TS_SZ;
            priv->ts_count++;
            if (priv->ule_skb == NULL) {
                priv->need_pusi = 1;
                priv->ule_sndu_type_1 = 0;
                priv->ule_sndu_len = 0;
            }
        }
    }	/* for all available TS cells */
}

void ule_resetDemuxCtx(ULEDemuxCtx* p)
{    
    //p->pid = 0x1FFF;
    p->need_pusi = 1;
    p->tscc = 0;

    if (p->ule_skb)
        xfree(p->ule_skb);

    p->ule_skb = NULL;
    //p->ule_next_hdr = NULL;
    p->ule_sndu_len = 0;
    p->ule_sndu_type = 0;
    p->ule_sndu_type_1 = 0;
    p->ule_sndu_remain = 0;
    p->ule_dbit = 0xFF;

    //p->ts_count = 0;

    p->ule_sndu_outbuf = NULL;
    p->ule_sndu_outbuf_len = 0;
}

static void hexdump( const unsigned char *buf, unsigned short len ){
    char str[80], octet[10];
    int ofs, i, l;
    for (ofs = 0; ofs < len; ofs += 16) {
        sprintf( str, "%03d: ", ofs );
        for (i = 0; i < 16; i++) {
            if ((i + ofs) < len)
                sprintf( octet, "%02x ", buf[ofs + i] );
            else
                strcpy( octet, "   " );
            strcat( str, octet );
        }
        strcat( str, "  " );
        l = strlen( str );
        for (i = 0; (i < 16) && ((i + ofs) < len); i++)
            str[l++] = isprint( buf[ofs + i] ) ? buf[ofs + i] : '.';
        str[l] = '\0';
        printk( KERN_WARNING "%s\n", str );
    }
}

