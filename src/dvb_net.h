#ifndef _DVB_NET_H_
#define _DVB_NET_H_

#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>
#include "ule.h"

struct it950x_dev;

typedef struct dvb_netdev {
    struct it950x_dev* itdev;
    struct net_device* netdev;
    ULEDemuxCtx demux;
    atomic_t tx_count;
    spinlock_t tx_lock;
} dvb_netdev;

extern dvb_netdev* dvb_alloc_netdev(struct it950x_dev*);
extern void dvb_free_netdev(dvb_netdev*);

#endif
