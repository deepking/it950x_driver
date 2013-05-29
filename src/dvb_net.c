/*  
 *  dvb net_device
 */
//#include <linux/module.h>       /* Needed by all modules */
#include <linux/kernel.h>       /* Needed for KERN_INFO */
#include <linux/mm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>          /* struct iphdr */
#include <linux/ctype.h>

#include <linux/workqueue.h>    /* We scheduale tasks here */
#include <linux/sched.h>        /* We need to put ourselves to sleep 
                                   and wake up later */
#include <linux/slab.h>
#include <asm/atomic.h>

#include "dvb_net.h"
#include "dvb_api.h"
#include "error.h"
#include "ule.h"
#define MIN(X,Y) (((X) < (Y)) ? : (X) : (Y))
#define RX_RING_BUF_COUNT 348
#define SEND_FREQ 666000
#define RECV_FREQ 666000

//#define PDEBUG(fmt, args...) printk(KERN_DEBUG "dvbnet: " fmt, ## args)
#define PDEBUG(fmt, args...)

static void intrpt_readTask(struct work_struct*);
static void intrpt_sendTask(struct work_struct*);

static int g_die = 0;     /* set this to 1 for shutdown */
static atomic_t g_tx_count;

static void hexdump( const unsigned char *buf, unsigned short len )
{
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

static void intrpt_sendTask(struct work_struct* work)
{
    Dword dwError = 0;
    Byte garbage[188] = {0x47,0x1f,0xff,0x1c, 0x00, 0x00};
    int count;
    int i;
    unsigned long cpu_flag;
    struct delayed_work* dwork;
    dvb_netdev* dvb;

    // find dvb_netdev
    dwork = to_delayed_work(work);
    dvb = container_of(dwork, dvb_netdev, tx_send_task);

    count = atomic_read(&g_tx_count);
    if (count == 0) {
        // idle
        spin_lock_irqsave(&dvb->tx_lock, cpu_flag);
        dwError = g_ITEAPI_TxSendTSData(dvb->itdev, garbage, 188);
        spin_unlock_irqrestore(&dvb->tx_lock, cpu_flag);

        dvb->netdev->stats.tx_carrier_errors++;

        if (dwError != 188) {
            printk(KERN_ERR "sendTask g_ITEAPI_TxSendTSData %ld\n", dwError);
        }

        goto next_send;
    }
    else if (count >= RX_RING_BUF_COUNT) {
        int remaining = count % RX_RING_BUF_COUNT;
        for (i = RX_RING_BUF_COUNT - remaining; i > 0; i--) {

            spin_lock_irqsave(&dvb->tx_lock, cpu_flag);
            dwError = g_ITEAPI_TxSendTSData(dvb->itdev, garbage, 188);
            spin_unlock_irqrestore(&dvb->tx_lock, cpu_flag);

            dvb->netdev->stats.tx_carrier_errors++;

            if (dwError != 188) {
                printk(KERN_ERR "sendTask g_ITEAPI_TxSendTSData %ld\n", dwError);
                goto reset_counter;
            }
        }
    }
    else {
        for (i = RX_RING_BUF_COUNT - count; i > 0; i--) {

            spin_lock_irqsave(&dvb->tx_lock, cpu_flag);
            dwError = g_ITEAPI_TxSendTSData(dvb->itdev, garbage, 188);
            spin_unlock_irqrestore(&dvb->tx_lock, cpu_flag);

            dvb->netdev->stats.tx_carrier_errors++;

            if (dwError != 188) {
                printk(KERN_ERR "sendTask g_ITEAPI_TxSendTSData %ld\n", dwError);
                goto reset_counter;
            }
        }
    }

reset_counter:
    atomic_sub(count, &g_tx_count);

next_send:
    if (!g_die && dvb)
        queue_delayed_work(dvb->tx_queue, &dvb->tx_send_task, 50);
}

static void intrpt_readTask(struct work_struct* work)
{
    Dword len = 188;
    Dword err = 0;
    Byte buf[len];
    Dword r;
    unsigned short pidFromBuf;
    int nFailCount = 0;
    dvb_netdev* netdev;
    struct sk_buff* skb;
    int errRX = NET_RX_SUCCESS;
    struct delayed_work* dwork;
    dvb_netdev* dvb;

    // find dvb_netdev
    dwork = to_delayed_work(work);
    dvb = container_of(dwork, dvb_netdev, rx_read_task);

    while (true) {
        memset(buf, 0, len);
        r = len;
        err = DTV_GetData(dvb->itdev, buf, &r);

        if (r<=0 || r != 188) {
            //printk(KERN_ERR "ERROR: only read %lu\n", r);
            goto next;
        }

        /*
        if (err) {
            printk(KERN_ERR "ERROR: DTV_GetData() %lu\n", err);
            goto next;
        }
        */

        if (buf[0] != 0x47) {
            //printk(KERN_INFO "desync (%x, %x, %x, %x) - %lu\n", buf[0], buf[1], buf[2], buf[3], r);
            while (buf[0] != 0x47) {
                r = 1;
                err = DTV_GetData(dvb->itdev, buf, &r);
                if (r <= 0 || r != 1) {
                    printk(KERN_ERR "ERROR read %lu", r);
                    goto next;
                }
            }

            // remaining
            r = 187;
            DTV_GetData(dvb->itdev, buf+1, &r);
            if (r != 187) {
                printk(KERN_ERR "sync error read %lu", r);
                continue;
            }
            continue;
        }

        pidFromBuf = ts_getPID(buf);
        if (pidFromBuf == 0x1FFF) {
            continue;
        }

        if (pidFromBuf != 0x1FAF) {
            continue;
        }

        ule_demux(&dvb->demux, buf, len);
        if (dvb->demux.ule_sndu_outbuf) {
            PDEBUG("outbuf: len=%d\n", dvb->demux.ule_sndu_outbuf_len);
            //hexdump(dvb->demux.ule_sndu_outbuf, dvb->demux.ule_sndu_outbuf_len);

            skb = dev_alloc_skb(dvb->demux.ule_sndu_outbuf_len);
            memcpy(skb_put(skb, dvb->demux.ule_sndu_outbuf_len), dvb->demux.ule_sndu_outbuf, dvb->demux.ule_sndu_outbuf_len);
            skb->dev = dvb->netdev;
            skb->protocol = eth_type_trans(skb, dvb->netdev);
            skb->pkt_type = PACKET_HOST;
            //skb->ip_summed = CHECKSUM_UNNECESSARY;

            errRX = netif_rx(skb);
            if (errRX != NET_RX_SUCCESS) {
                dvb->netdev->stats.rx_errors++;
                printk(KERN_WARNING "RxQueue drop packet.\n");
            }
            else {
                dvb->netdev->stats.rx_packets++;
            }

            // netdevice stats
            dvb->netdev->stats.rx_packets++;
            dvb->netdev->stats.rx_bytes += dvb->demux.ule_sndu_outbuf_len;
                        
            // clean & reset outbuf
            kfree(dvb->demux.ule_sndu_outbuf);
            dvb->demux.ule_sndu_outbuf = NULL;
            dvb->demux.ule_sndu_outbuf_len = 0;
        }

    }// end loop

next:
    if (!g_die && dvb) {
        queue_delayed_work(dvb->rx_queue, &dvb->rx_read_task, 50);
    }
}

void startCapture(dvb_netdev* dvb)
{
    Dword err = 0;
    Bool bLocked = false;
    struct it950x_dev* dev;

    dev = dvb->itdev;

    printk(KERN_INFO "%s()\n", __FUNCTION__);

    err = DTV_AcquireChannel(dev, RECV_FREQ, 8000);
    if (err) {
        printk(KERN_ERR "DTV_AcquireChannel error %ld\n", err);
        return;
    }

    printk(KERN_INFO "%s() end DTV_AcquireChannel\n", __FUNCTION__);

    err = DTV_DisablePIDTbl(dev);
    if (err) {
        printk(KERN_ERR "DTV_DisablePIDTbl error %lu\n", err);
        return;
    }

    printk(KERN_INFO "%s() end DTV_DisablePIDTbl\n", __FUNCTION__);

    err = DTV_IsLocked(dev, &bLocked);
    if (err) {
        printk(KERN_ERR "DTV_IsLocked error %lu\n", err);
        return;
    }

    printk(KERN_INFO "%s() end DTV_IsLocked\n", __FUNCTION__);

    if (bLocked) {
        printk(KERN_INFO "[dvbnet] Channel locked!!\n");
    }
    else {
        printk(KERN_ERR "[dvbnet] Channel unlocked!!\n");
    }

    err = DTV_StartCapture(dev);
    if (err) {
        printk(KERN_ERR "DTV_StartCapture error %lu\n", err);
        return;
    }

    queue_delayed_work(dvb->rx_queue, &dvb->rx_read_task, 100);

    printk(KERN_INFO "dvbnet startCapture ok!\n");
}

void stopCapture(dvb_netdev* dvb)
{
    if (dvb == NULL)
        return;

    g_die = 1;// XXX

    if (dvb->itdev) {
        DTV_StopCapture(dvb->itdev);
    }
}

static int dvb_net_tx(struct sk_buff *skb, struct net_device *dev)
{
    dvb_netdev* netdev;
    SNDUInfo snduinfo;
    uint32_t totalLength;
    ULEEncapCtx encapCtx;
    Dword len = 0;
    int count = 0;
    unsigned long cpu_flag;
    
    netdev = netdev_priv(dev);

    ule_init(&snduinfo, IPv4, skb->data, skb->len);
    totalLength = ule_getTotalLength(&snduinfo);
    unsigned char pkt[totalLength];
    ule_encode(&snduinfo, pkt, totalLength);
    
    //printk(KERN_INFO "sndu: totalLength=%d, Len=%d, type=%xd, dataLen=%d\n", 
    //    totalLength, snduinfo.length, snduinfo.type, snduinfo.pdu.length);

    ule_initEncapCtx(&encapCtx);
    encapCtx.pid = 0x1FAF;// TODO: param
    encapCtx.snduPkt = pkt;
    encapCtx.snduLen = totalLength;

    while (encapCtx.snduIndex < encapCtx.snduLen) {
        ule_padding(&encapCtx);
        PDEBUG("tx %d snduIndex:%d snduLen:%d\n", count++, encapCtx.snduIndex, encapCtx.snduLen);
        //hexdump(encapCtx.tsPkt, 188);

        spin_lock_irqsave(&netdev->tx_lock, cpu_flag);
        len = g_ITEAPI_TxSendTSData(netdev->itdev, encapCtx.tsPkt, 188);
        spin_unlock_irqrestore(&netdev->tx_lock, cpu_flag);

        if (len != 188) {
            printk(KERN_ERR "sendTsData error %ld\n", len);
            dev->stats.tx_errors++;
            return len; //XXX
        }
        else {
            atomic_inc(&g_tx_count);
            dev->stats.tx_packets++;
            dev->stats.tx_bytes += 188;
        }
    }

    return NETDEV_TX_OK;
}

static void dvb_net_set_multicast_list (struct net_device *dev)
{
    /* struct dvb_net_priv *priv = netdev_priv(dev); */
    /* schedule_work(&priv->set_multicast_list_wq); */
}

static int dvb_net_set_mac (struct net_device *dev, void *p)
{
    /* struct dvb_net_priv *priv = netdev_priv(dev); */
    /* struct sockaddr *addr=p; */

    /* memcpy(dev->dev_addr, addr->sa_data, dev->addr_len); */

    /* if (netif_running(dev)) */
    /*     schedule_work(&priv->restart_net_feed_wq); */

    return 0;
}

static Dword startTransfer(dvb_netdev* dvb)
{
    Dword dwError = 0;
    struct it950x_dev* dev;
    SetModuleRequest req;

    dev = dvb->itdev;

    printk(KERN_INFO "%s()\n", __FUNCTION__);

    dwError = g_ITEAPI_TxSetChannel(dev, SEND_FREQ, 8000);
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "set channel fail %ld\n", dwError);
        return dwError;
    }

    req.chip = 0;
    //printf("\n=> Please Input constellation (0:QPSK  1:16QAM  2:64QAM): ");
    req.constellation = (Byte) 2;

    //printf("\n=> Please Input Code Rate"); printf(" (0:1/2  1:2/3  2:3/4  3:5/6  4:7/8): ");
    req.highCodeRate = (Byte) 4;

    //printf("\n=> Please Input Interval (0:1/32  1:1/16  2:1/8  3:1/4): ");
    req.interval = (Byte) 0;
    
    //printf("\n=> Please Input Transmission Mode (0:2K  1:8K): ");
    req.transmissionMode = (Byte) 0;

    dwError = g_ITEAPI_TxSetChannelModulation(dev, &req);
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "setChannelModulation %ld\n", dwError);
        return dwError;
    }

    /*
    Byte CustomPacket_3[188]={0x47,0x10,0x03,0x1c,0x00,0x00};
    Byte CustomPacket_4[188]={0x47,0x10,0x04,0x1c,0x00,0x00};
    Byte CustomPacket_5[188]={0x47,0x10,0x05,0x1c,0x00,0x00};
    dwError = g_ITEAPI_TxSetPeridicCustomPacket(dev, 188, CustomPacket_3, 1);
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "g_ITEAPI_TxAccessFwPSITable 1 fail %ld\n", dwError);
        return dwError;
    }

    dwError = g_ITEAPI_TxSetPeridicCustomPacketTimer(dev, 1, 100);//ms
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "g_ITEAPI_TxSetFwPSITableTimer  %d fail\n", 1);
        return dwError;
    }
    */

    dwError = g_ITEAPI_StartTransfer(dev);
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "startTransfer %ld\n", dwError);
        return dwError;
    }

    queue_delayed_work(dvb->tx_queue, &dvb->tx_send_task, 50);

    return dwError;
}

static int dvb_net_open(struct net_device *dev)
{
    dvb_netdev* dvb = NULL;
    Dword dwError = 0;

    printk(KERN_INFO "dvbnet open.\n");
    dvb = netdev_priv(dev);

    g_die = 0;
    startTransfer(dvb);
    startCapture(dvb);

    // allow upper layers to call the device hard_start_xmit routin.
    netif_start_queue(dev);

    return 0;
}

static int dvb_net_stop(struct net_device *dev)
{
    dvb_netdev* dvb;

    dvb = netdev_priv(dev);
    stopCapture(dvb);

    // disallow upper layers to call the device hard_start_xmit routin.
    netif_stop_queue(dev);

    printk(KERN_INFO "dvbnet stop.\n");

    return 0;
}

/*
 * Configuration changes (passed on by ifconfig)
 */
static int dvb_config(struct net_device *dev, struct ifmap *map)
{
    if (dev->flags & IFF_UP) /* can't act on a running interface */
        return -EBUSY;

    /* Don't allow changing the I/O address */
    if (map->base_addr != dev->base_addr) {
        printk(KERN_WARNING "dvbnet: Can't change I/O address\n");
        return -EOPNOTSUPP;
    }

    /* Allow changing the IRQ */
    if (map->irq != dev->irq) {
        dev->irq = map->irq;
        /* request_irq() is delayed to open-time */
    }

    /* ignore other fields */
    return 0;
}

/*
 * Ioctl commands 
 */
int dvb_do_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    printk(KERN_DEBUG "dvbnet ioctl\n");
    return 0;
}

static void dvb_tx_timeout(struct net_device* dev) {
    printk(KERN_ERR "dvb_tx_timeout\n");
}

static const struct net_device_ops dvb_netdev_ops = {
    .ndo_open               = dvb_net_open,
    .ndo_stop               = dvb_net_stop,
    .ndo_start_xmit         = dvb_net_tx,
    /* .ndo_do_ioctl           = dvb_do_ioctl, */
    /* .ndo_set_rx_mode        = dvb_net_set_multicast_list, */
    /* .ndo_set_mac_address    = dvb_net_set_mac, */
    /* .ndo_change_mtu         = eth_change_mtu, */
    /* .ndo_set_config         = dvb_config, */
    .ndo_tx_timeout = dvb_tx_timeout,

    // cause SIOCSIFFLAGS: Cannot assign requested address, (ifconfig dvb0 ip)
    /* .ndo_validate_addr      = eth_validate_addr, */
};
 
static const struct header_ops dvb_header_ops = {
    .create     = eth_header,
    .parse      = eth_header_parse,
    .rebuild    = eth_rebuild_header,
};

static void dvb_net_setup(struct net_device *dev)
{
    printk(KERN_INFO "dvbnet setup.\n");

    ether_setup(dev);
    /* dev->header_ops     = &dvb_header_ops; */
    dev->netdev_ops     = &dvb_netdev_ops;
    dev->mtu            = 4096;
    dev->flags |= IFF_NOARP;
}

dvb_netdev* dvb_alloc_netdev(struct it950x_dev* itdev)
{
    int err = 0;
    struct net_device *netdev = NULL;
    dvb_netdev* dev;

    netdev = alloc_netdev(sizeof(dvb_netdev), "dvb%d", dvb_net_setup);
    if (!netdev) {
        printk(KERN_ERR "dvbnet alloc netdev err\n");
        return NULL;
    }

    dev = netdev_priv(netdev);
    dev->itdev = itdev;
    dev->netdev = netdev;

    // TODO clear while fail
    err = it950x_usb_rx_alloc_dev(dev->itdev);
    if (err) { 
        printk(KERN_ERR "dvbnet it950x_usb_rx_alloc_dev error\n");
        return NULL;
    }

    err = it950x_usb_tx_alloc_dev(dev->itdev);
    if (err) {
        printk(KERN_ERR "dvbnet it950x_usb_rx_alloc_dev error\n");
        return NULL;
    }

    err = register_netdev(netdev);
    if (err) {
        printk(KERN_ERR "dvbnet register err\n");
        return NULL;
    }


    ule_initDemuxCtx(&dev->demux);
    dev->demux.pid = 0x1FAF;

    atomic_set(&g_tx_count, 0);// XXX

    dev->tx_queue = create_workqueue("sendQueue");
    INIT_DELAYED_WORK(&dev->tx_send_task, intrpt_sendTask);

    dev->rx_queue = create_workqueue("readQueue"); 
    INIT_DELAYED_WORK(&dev->rx_read_task, intrpt_readTask);

    return dev;
}

void dvb_free_netdev(dvb_netdev* dev)
{
    if (dev == NULL)
        return;

    g_die = 1;

    if (dev->tx_queue) {
        destroy_workqueue(dev->tx_queue);
        dev->tx_queue = NULL;
    }
    if (dev->rx_queue) {
        destroy_workqueue(dev->rx_queue);
        dev->rx_queue = NULL;
    }

    if (dev->itdev) {
        it950x_usb_tx_free_dev(dev->itdev);
        it950x_usb_rx_free_dev(dev->itdev);
        dev->itdev = NULL;
    }
    
    if (dev->netdev) {
        unregister_netdev(dev->netdev);
        //FIXME
        //free_netdev(dev->netdev);
    }
}

