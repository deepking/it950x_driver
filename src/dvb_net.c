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
#include <linux/delay.h>

#include <linux/workqueue.h>    /* We scheduale tasks here */
#include <linux/sched.h>        /* We need to put ourselves to sleep 
                                   and wake up later */
#include <linux/slab.h>

#include "dvb_net.h"
#include "dvb_api.h"
#include "error.h"
#include "ule.h"

#define TS_SZ 188
#define TX_RING_BUF_COUNT 174
#define RX_RING_BUF_COUNT 348
#define RX_MAX_POLL_COUNT 348*16

#define SEND_FREQ 666000
#define RECV_FREQ 666000

/* timer millis */
#define TX_SEND_PERIOD 20
#define RX_RECV_PERIOD 50
#define RX_MAX_FAIL_COUNT RX_RING_BUF_COUNT * (16 + 1)

//#define PDEBUG(fmt, args...) printk(KERN_DEBUG "[DEBUG] (%s:%d) " fmt, __FUNCTION__,__LINE__, ## args)
#define PDEBUG(fmt, args...)
#define PINFO(fmt, args...) printk(KERN_INFO "[INFO] " fmt, ## args) 
#define PERROR(fmt, args...) printk(KERN_ERR "[ERROR] (%s:%d) " fmt, __FUNCTION__,__LINE__, ## args)

static void intrpt_readTask(struct work_struct*);
static void intrpt_sendTask(struct work_struct*);
static void schedule_read_task(dvb_netdev*, int);
static void schedule_send_task(dvb_netdev*, int);
static void hexdump(const unsigned char *, unsigned short);

static void intrpt_sendTask(struct work_struct* work)
{
    Dword dwError = 0;
    Byte garbage[TS_SZ] = {0x47,0x1f,0xff,0x1c, 0x00, 0x00};
    int count;
    int i;
    unsigned long cpu_flag;
    struct delayed_work* dwork;
    dvb_netdev* dvb;

    // find dvb_netdev
    dwork = to_delayed_work(work);
    dvb = container_of(dwork, dvb_netdev, tx_send_task);

    count = atomic_read(&dvb->tx_count);
    if (count == 0) {
        // idle
        spin_lock_irqsave(&dvb->tx_lock, cpu_flag);
        dwError = g_ITEAPI_TxSendTSData(dvb->itdev, garbage, TS_SZ);
        spin_unlock_irqrestore(&dvb->tx_lock, cpu_flag);

        if (dwError != TS_SZ) {
            PERROR("send garbage when idle. error=%lu\n", dwError);
        }
        else {
            dvb->netdev->stats.tx_carrier_errors++;
        }
    }
    else if (count >= TX_RING_BUF_COUNT) {
        /* handling reamining next time */
        int remaining = count % TX_RING_BUF_COUNT;
        count = count - remaining;
    }
    else {
        int sent = 0;
        spin_lock_irqsave(&dvb->tx_lock, cpu_flag);
        for (i = TX_RING_BUF_COUNT - count; i > 0; i--) {
            dwError = g_ITEAPI_TxSendTSData(dvb->itdev, garbage, TS_SZ);

            if (dwError != TS_SZ) {
                PERROR("send garbage. count=%d, sent=%d, error=%lu\n", count, sent, dwError);
                break;
            }
            else {
                dvb->netdev->stats.tx_carrier_errors++;
                sent++;
            }
        }
        spin_unlock_irqrestore(&dvb->tx_lock, cpu_flag);
    }

    atomic_sub(count, &dvb->tx_count);

    schedule_send_task(dvb, TX_SEND_PERIOD);
}

static void intrpt_readTask(struct work_struct* work)
{
    const Dword len = TS_SZ;
    Dword err = 0;
    Byte buf[len];
    Dword r;
    unsigned short pidFromBuf;
    int nFailCount = 0;
    struct sk_buff* skb;
    int errRX = NET_RX_SUCCESS;
    struct delayed_work* dwork;
    dvb_netdev* dvb;
    int recv_count = 0;

    // find dvb_netdev
    dwork = to_delayed_work(work);
    dvb = container_of(dwork, dvb_netdev, rx_read_task);

    while (dvb->rx_run) {
        memset(buf, 0, len);
        r = len;
        err = DTV_GetData(dvb->itdev, buf, &r);

        if (r<=0 || r != len) {
            nFailCount++;
            if (nFailCount > RX_MAX_FAIL_COUNT) {
                goto next;
            }
            else {
                msleep(10);
                continue;
            }
        }

        if (buf[0] != 0x47) {
            //printk(KERN_INFO "desync (%x, %x, %x, %x) - %lu\n", buf[0], buf[1], buf[2], buf[3], r);
            while (buf[0] != 0x47) {
                r = 1;
                err = DTV_GetData(dvb->itdev, buf, &r);
                if (r <= 0 || r != 1) {
                    PERROR("read=%lu\n", r);
                    goto next;
                }
            }

            // remaining
            r = 187;
            DTV_GetData(dvb->itdev, buf+1, &r);
            if (r != 187) {
                PERROR("remaining read=%lu\n", r);
                goto next;
            }
        }

        pidFromBuf = ts_getPID(buf);
        if (pidFromBuf == 0x1FFF) {
            dvb->netdev->stats.rx_frame_errors++;
            nFailCount++;
            if (nFailCount >= RX_MAX_FAIL_COUNT)
                goto next;
            else
                continue;
        }

        /* reset fail counter */
        nFailCount = 0;

        if (pidFromBuf != dvb->demux.pid) {
            dvb->netdev->stats.rx_frame_errors++;
            continue;
        }

        ule_demux(&dvb->demux, buf, len);

        // update stats
        dvb->netdev->stats.rx_errors += dvb->demux.rx_errors;
        dvb->netdev->stats.rx_frame_errors += dvb->demux.rx_frame_errors;
        dvb->netdev->stats.rx_crc_errors += dvb->demux.rx_crc_errors;
        dvb->netdev->stats.rx_dropped += dvb->demux.rx_dropped;

        if (dvb->demux.ule_sndu_outbuf) {
            PDEBUG("outbuf: len=%d\n", dvb->demux.ule_sndu_outbuf_len);
            //hexdump(dvb->demux.ule_sndu_outbuf, dvb->demux.ule_sndu_outbuf_len);

            skb = dev_alloc_skb(dvb->demux.ule_sndu_outbuf_len);
            memcpy(skb_put(skb, dvb->demux.ule_sndu_outbuf_len), dvb->demux.ule_sndu_outbuf, dvb->demux.ule_sndu_outbuf_len);
            skb->dev = dvb->netdev;
            skb->protocol = eth_type_trans(skb, dvb->netdev);
            skb->pkt_type = PACKET_HOST;
            skb->ip_summed = CHECKSUM_UNNECESSARY;

            errRX = netif_rx(skb);
            if (errRX != NET_RX_SUCCESS) {
                dvb->netdev->stats.rx_dropped++;
                PERROR("RxQueue drop packet.\n");
            }
            else {
                dvb->netdev->stats.rx_packets++;
                recv_count++;
            }

            // netdevice stats
            dvb->netdev->stats.rx_packets++;
            dvb->netdev->stats.rx_bytes += dvb->demux.ule_sndu_outbuf_len;
                        
            // clean & reset outbuf
            kfree(dvb->demux.ule_sndu_outbuf);
            dvb->demux.ule_sndu_outbuf = NULL;
            dvb->demux.ule_sndu_outbuf_len = 0;
        }

        if (recv_count > RX_MAX_POLL_COUNT) {
            break;
        }

    }// end loop

next:
    schedule_read_task(dvb, RX_RECV_PERIOD);
}

static void startCapture(dvb_netdev* dvb)
{
    Dword err = 0;
    Bool bLocked = false;
    struct it950x_dev* dev;

    dev = dvb->itdev;

    PINFO("%s()\n", __FUNCTION__);

    err = DTV_AcquireChannel(dev, RECV_FREQ, 8000);
    if (err) {
        PERROR("DTV_AcquireChannel error=%lu\n", err);
        return;
    }

    PINFO("%s() end DTV_AcquireChannel\n", __FUNCTION__);

    err = DTV_DisablePIDTbl(dev);
    if (err) {
        PERROR("DTV_DisablePIDTbl error=%lu\n", err);
        return;
    }

    PINFO("%s() end DTV_DisablePIDTbl\n", __FUNCTION__);

    err = DTV_IsLocked(dev, &bLocked);
    if (err) {
        PERROR("DTV_IsLocked error=%lu\n", err);
        return;
    }

    PINFO("%s() end DTV_IsLocked\n", __FUNCTION__);

    if (bLocked) {
        PINFO("[dvbnet] Channel locked!!\n");
    }
    else {
        PERROR("[dvbnet] Channel unlocked!!\n");
    }

    err = DTV_StartCapture(dev);
    if (err) {
        PERROR("DTV_StartCapture error=%lu\n", err);
        return;
    }

    dvb->rx_run = true;

    // init timer
    dvb->rx_queue = create_singlethread_workqueue("readQueue"); 
    INIT_DELAYED_WORK(&dvb->rx_read_task, intrpt_readTask);

    // start timer
    schedule_read_task(dvb, 1);

    PINFO("dvbnet startCapture ok!\n");
}

static void schedule_read_task(dvb_netdev* dvb, int ms)
{
    unsigned long flag;

    if (dvb == NULL || !dvb->rx_run)
        return;

    spin_lock_irqsave(&dvb->rx_lock, flag);
    if (dvb->rx_run && dvb->rx_queue)
        queue_delayed_work(dvb->rx_queue, &dvb->rx_read_task, ms);
    spin_unlock_irqrestore(&dvb->rx_lock, flag);
}

static void stopCapture(dvb_netdev* dvb)
{
    unsigned long flag;

    if (dvb == NULL)
        return;

    dvb->rx_run = false;

    /* release queue */
    if (dvb->rx_queue) {
        cancel_delayed_work(&dvb->rx_read_task);

        spin_lock_irqsave(&dvb->rx_lock, flag);
        destroy_workqueue(dvb->rx_queue);
        dvb->rx_queue = NULL;
        spin_unlock_irqrestore(&dvb->rx_lock, flag);
    }

    if (dvb->itdev) {
        DTV_StopCapture(dvb->itdev);
        dvb->itdev = NULL;
    }
}

static int dvb_net_tx(struct sk_buff *skb, struct net_device *dev)
{
    int ret = NETDEV_TX_OK;
    dvb_netdev* dvb;
    SNDUInfo snduinfo;
    uint32_t totalLength;
    ULEEncapCtx encapCtx;
    Dword len = 0;
    int count = 0;
    unsigned long cpu_flag;
    
    dvb = netdev_priv(dev);

    ule_init(&snduinfo, IPv4, skb->data, skb->len);
    totalLength = ule_getTotalLength(&snduinfo);
    unsigned char pkt[totalLength];
    ule_encode(&snduinfo, pkt, totalLength);
    
    //printk(KERN_INFO "sndu: totalLength=%d, Len=%d, type=%xd, dataLen=%d\n", 
    //    totalLength, snduinfo.length, snduinfo.type, snduinfo.pdu.length);

    ule_initEncapCtx(&encapCtx);
    encapCtx.pid = dvb->demux.pid;
    encapCtx.snduPkt = pkt;
    encapCtx.snduLen = totalLength;

    spin_lock_irqsave(&dvb->tx_lock, cpu_flag);
    while (encapCtx.snduIndex < encapCtx.snduLen) {
        ule_padding(&encapCtx);
        PDEBUG("tx %d snduIndex:%d snduLen:%d\n", count++, encapCtx.snduIndex, encapCtx.snduLen);
        //hexdump(encapCtx.tsPkt, TS_SZ);

        len = g_ITEAPI_TxSendTSData(dvb->itdev, encapCtx.tsPkt, TS_SZ);

        if (len != TS_SZ) {
            PERROR("sendTsData error %lu\n", len);
            dev->stats.tx_errors++;
            break;
        }
        else {
            atomic_inc(&dvb->tx_count);
            dev->stats.tx_packets++;
            dev->stats.tx_bytes += TS_SZ;
        }
    }
    spin_unlock_irqrestore(&dvb->tx_lock, cpu_flag);

    dev_kfree_skb(skb);
    return ret;
}

static Dword startTransfer(dvb_netdev* dvb)
{
    int i;
    Dword dwError = 0;
    struct it950x_dev* dev;
    SetModuleRequest req;

    dev = dvb->itdev;

    PINFO("%s()\n", __FUNCTION__);

    dwError = g_ITEAPI_TxSetChannel(dev, SEND_FREQ, 8000);
    if (dwError != Error_NO_ERROR) {
        PERROR("set channel fail error=%lu\n", dwError);
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
        PERROR("setChannelModulation error=%lu\n", dwError);
        return dwError;
    }

    /*
    Byte CustomPacket_3[TS_SZ]={0x47,0x10,0x03,0x1c,0x00,0x00};
    Byte CustomPacket_4[TS_SZ]={0x47,0x10,0x04,0x1c,0x00,0x00};
    Byte CustomPacket_5[TS_SZ]={0x47,0x10,0x05,0x1c,0x00,0x00};
    dwError = g_ITEAPI_TxSetPeridicCustomPacket(dev, TS_SZ, CustomPacket_3, 1);
    if (dwError != Error_NO_ERROR) {
        PERROR("g_ITEAPI_TxAccessFwPSITable 1 fail %lu\n", dwError);
        return dwError;
    }

    dwError = g_ITEAPI_TxSetPeridicCustomPacketTimer(dev, 1, 100);//ms
    if (dwError != Error_NO_ERROR) {
        PERROR("g_ITEAPI_TxSetFwPSITableTimer  %d fail\n", 1);
        return dwError;
    }
    */
    /* disable custom packet timer */
    for (i = 1; i <= 5; i++) {
        dwError = g_ITEAPI_TxSetPeridicCustomPacketTimer(dev, i, 0);
        if (dwError != Error_NO_ERROR) {
            PERROR("g_ITEAPI_TxSetFwPSITableTimer  %d fail\n", i);

            /* reset error code */
            dwError = Error_NO_ERROR;
        }
    }

    dwError = g_ITEAPI_StartTransfer(dev);
    if (dwError != Error_NO_ERROR) {
        PERROR("startTransfer error=%lu\n", dwError);
        return dwError;
    }

    dvb->tx_run = true;
    atomic_set(&dvb->tx_count, 0);

    // init timer
    dvb->tx_queue = create_singlethread_workqueue("sendQueue");
    INIT_DELAYED_WORK(&dvb->tx_send_task, intrpt_sendTask);

    // start timer
    schedule_send_task(dvb, 1);

    return dwError;
}

static void schedule_send_task(dvb_netdev* dvb, int ms)
{
    unsigned long flag;
    
    if (dvb == NULL || !dvb->tx_run)
        return;

    spin_lock_irqsave(&dvb->tx_lock, flag);
    if (dvb->tx_run && dvb->tx_queue)
        queue_delayed_work(dvb->tx_queue, &dvb->tx_send_task, ms);
    spin_unlock_irqrestore(&dvb->tx_lock, flag);
}

static Dword stopTransfer(dvb_netdev* dvb)
{
    Dword dwError = 0;
    unsigned long flag;

    if (dvb == NULL)
        return dwError;

    dvb->tx_run = false;
    atomic_set(&dvb->tx_count, 0);

    // release queue
    if (dvb->tx_queue) {
        cancel_delayed_work(&dvb->tx_send_task);

        spin_lock_irqsave(&dvb->tx_lock, flag);
        destroy_workqueue(dvb->tx_queue);
        dvb->tx_queue = NULL;
        spin_unlock_irqrestore(&dvb->tx_lock, flag);
    }

    dwError = g_ITEAPI_StopTransfer(dvb->itdev);
    if (dwError != Error_NO_ERROR) {
        PERROR("stopTransfer error=%lu\n", dwError);
        return dwError;
    }

    return dwError;
}

static int dvb_net_open(struct net_device *dev)
{
    dvb_netdev* dvb = NULL;
    Dword dwError = 0;

    PINFO("dvbnet open.\n");
    dvb = netdev_priv(dev);

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
    stopTransfer(dvb);
    stopCapture(dvb);

    // disallow upper layers to call the device hard_start_xmit routin.
    netif_stop_queue(dev);

    PINFO("dvbnet stop.\n");

    return 0;
}

#if 0
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
static int dvb_do_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
    printk(KERN_DEBUG "dvbnet ioctl\n");
    return 0;
}

#endif

static void dvb_tx_timeout(struct net_device* dev)
{
    PERROR("dvb_tx_timeout\n");
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
    PINFO("dvbnet setup.\n");

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
        PERROR("dvbnet alloc netdev err\n");
        return NULL;
    }

    dev = netdev_priv(netdev);
    dev->itdev = itdev;
    dev->netdev = netdev;

    // TODO clear while fail
    err = it950x_usb_rx_alloc_dev(dev->itdev);
    if (err) { 
        PERROR("it950x_usb_rx_alloc_dev error\n");
        return NULL;
    }

    err = it950x_usb_tx_alloc_dev(dev->itdev);
    if (err) {
        PERROR("it950x_usb_rx_alloc_dev error\n");
        return NULL;
    }

    err = register_netdev(netdev);
    if (err) {
        PERROR("register err\n");
        return NULL;
    }

    ule_initDemuxCtx(&dev->demux);
    dev->demux.pid = 0x1FAF;

    spin_lock_init(&dev->rx_lock);
    spin_lock_init(&dev->tx_lock);

    return dev;
}

void dvb_free_netdev(dvb_netdev* dev)
{
    if (dev == NULL)
        return;

    stopTransfer(dev);
    stopCapture(dev);

    if (dev->itdev) {
        it950x_usb_tx_free_dev(dev->itdev);
        it950x_usb_rx_free_dev(dev->itdev);
        dev->itdev = NULL;
    }
    
    if (dev->netdev) {
        netif_stop_queue(dev->netdev);
        unregister_netdev(dev->netdev);
        free_netdev(dev->netdev);
    }
}

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
