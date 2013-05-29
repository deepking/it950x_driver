#ifndef _DVB_NET_H_
#define _DVB_NET_H_

#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "ule.h"

/* define in src/it950x_core.c */
struct it950x_dev;

typedef struct dvb_netdev {
    struct it950x_dev* itdev;
    struct net_device* netdev;
    ULEDemuxCtx demux;

    volatile bool tx_run;
    spinlock_t tx_lock;
    struct workqueue_struct* tx_queue;
    struct delayed_work tx_send_task;

    volatile bool rx_run;
    spinlock_t rx_lock;
    struct workqueue_struct* rx_queue;
    struct delayed_work rx_read_task;
} dvb_netdev;

extern dvb_netdev* dvb_alloc_netdev(struct it950x_dev*);
extern void dvb_free_netdev(dvb_netdev*);

#endif
