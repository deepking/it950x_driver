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

#include "dvb_net.h"
#include "dvb_api.h"
#include "error.h"

#define _debug(fmt, args...)

static void intrpt_routine(struct work_struct*);
static int die = 0;     /* set this to 1 for shutdown */
static struct workqueue_struct *my_workqueue;
static DECLARE_DELAYED_WORK(ReadTask, intrpt_routine);
static struct it950x_dev* itdev;

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

static void intrpt_routine(struct work_struct* work)
{
    Dword len = 188;
    Dword err = 0;
    Byte buf[len];

    if (!itdev)
        return;

    printk(KERN_INFO "[dvbnet] rx run\n");

    memset(buf, 0, len);

    err = DTV_GetData(itdev, buf, &len);
    if (err) {
        printk(KERN_ERR "DTV_GetData error %lu\n", err);
    }
    else {
        printk(KERN_INFO "[dvbnet] rx\n");
        hexdump(buf, len);
    }

    /* If cleanup wants us to die */
    if (!die)
        queue_delayed_work(my_workqueue, &ReadTask, 1000);
}

void startCapture(struct it950x_dev* dev)
{
    Dword err = 0;

    err = DTV_AcquireChannel(dev, 666000, 8000);
    if (err) {
        printk(KERN_ERR "DTV_AcquireChannel error %ld\n", err);
        return;
    }

    err = DTV_DisablePIDTbl(dev);
    if (err) {
        printk(KERN_ERR "DTV_DisablePIDTbl error %lu\n", err);
        return;
    }

    bool bLocked = false;
    err = DTV_IsLocked(dev, &bLocked);
    if (err) {
        printk(KERN_ERR "DTV_IsLocked error %lu\n", err);
        return;
    }

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

    itdev = dev;

    //my_workqueue = create_workqueue("workqueue");
    //queue_delayed_work(my_workqueue, &ReadTask, 1000);

    printk(KERN_INFO "dvbnet startCapture ok!\n");
}

void stopCapture(void)
{
    die = 1;
    if (itdev) {
        DTV_StopCapture(itdev);
        itdev = NULL;
    }
}

static int dvb_net_tx(struct sk_buff *skb, struct net_device *dev)
{
    dvb_netdev* netdev = netdev_priv(dev);
    Dword len = 0;
    Byte NullPacket[188]={0x47,0x1f,0xaf,0x1c,0x00,0x00};
    NullPacket[4] = 'h';
    NullPacket[5] = 'a';
    NullPacket[6] = 'h';
    NullPacket[7] = 'a';
    NullPacket[8] = '\n';
    NullPacket[9] = '\0';


    //printk(KERN_INFO "dvbnet tx");
    //hexdump(skb->data, skb->len);
    /* dev_kfree_skb(skb); */
    //len = g_ITEAPI_TxSendTSData(netdev->itdev, skb->data, skb->len);
    /* len = g_ITEAPI_TxSendTSData(netdev->itdev, NullPacket, 188); */
    /* if (len != 188) { */
    /*     printk(KERN_ERR "sendTsData error %ld\n", len); */
    /*     return len; //XXX */
    /* } */
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

static Dword startTransfer(struct it950x_dev* dev)
{
    Dword dwError = 0;
    SetModuleRequest req;

    dwError = g_ITEAPI_TxSetChannel(dev, 666000, 8000);
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "set channel fail %ld\n", dwError);
        return dwError;
    }

    req.chip = 0;
    //printf("\n=> Please Input constellation (0:QPSK  1:16QAM  2:64QAM): ");
    req.constellation = (Byte) 1;

    //printf("\n=> Please Input Code Rate"); printf(" (0:1/2  1:2/3  2:3/4  3:5/6  4:7/8): ");
    req.highCodeRate = (Byte) 1;

    //printf("\n=> Please Input Interval (0:1/32  1:1/16  2:1/8  3:1/4): ");
    req.interval = (Byte) 3;
    
    //printf("\n=> Please Input Transmission Mode (0:2K  1:8K): ");
    req.transmissionMode = (Byte) 1;

    dwError = g_ITEAPI_TxSetChannelModulation(dev, &req);
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "setChannelModulation %ld\n", dwError);
        return dwError;
    }

    dwError = g_ITEAPI_StartTransfer(dev);
    if (dwError != Error_NO_ERROR) {
        printk(KERN_ERR "startTransfer %ld\n", dwError);
        return dwError;
    }

    return dwError;
}

static int dvb_net_open(struct net_device *dev)
{
    dvb_netdev* netdev = NULL;
    Dword dwError = 0;
    printk(KERN_INFO "dvbnet open.\n");
    /* struct dvb_net_priv *priv = netdev_priv(dev); */

    /* priv->in_use++; */
    /* dvb_net_feed_start(dev); */

    /* memcpy(dev->dev_addr, "\0DVB0", ETH_ALEN); */
    /* netif_start_queue(dev); */

    netdev = netdev_priv(dev);

    startTransfer(netdev->itdev);
    startCapture(netdev->itdev);

    return 0;
}

static int dvb_net_stop(struct net_device *dev)
{
    printk(KERN_INFO "dvbnet stop.\n");
    stopCapture();
    /* struct dvb_net_priv *priv = netdev_priv(dev); */

    /* priv->in_use--; */
    /* return dvb_net_feed_stop(dev); */
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

static const struct net_device_ops dvb_netdev_ops = {
    .ndo_open               = dvb_net_open,
    .ndo_stop               = dvb_net_stop,
    .ndo_start_xmit         = dvb_net_tx,
    /* .ndo_do_ioctl           = dvb_do_ioctl, */
    /* .ndo_set_rx_mode        = dvb_net_set_multicast_list, */
    /* .ndo_set_mac_address    = dvb_net_set_mac, */
    /* .ndo_change_mtu         = eth_change_mtu, */
    /* .ndo_set_config         = dvb_config, */

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
    /* dev->mtu            = 4096; */
    /* dev->flags |= IFF_NOARP; */
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

    return dev;
}

void dvb_free_netdev(dvb_netdev* dev)
{
    unregister_netdev(dev->netdev);
    it950x_usb_tx_free_dev(dev->itdev);
    it950x_usb_rx_free_dev(dev->itdev);
}

/*
static __init int dvb_net_init_module(void)
{
    printk(KERN_INFO "dvbnet init.\n");

    //it950x_usb_rx_free_dev(NULL);

    //struct net_device *dev;
    int err = 0;
    dev = alloc_netdev(0, "dvb%d", dvb_net_setup);
    if (!dev) {
        printk(KERN_ERR "dvbnet alloc netdev err\n");
        goto init_module_out;
    }

    err = register_netdev(dev);
    if (err) {
        printk(KERN_ERR "dvbnet register err\n");
        goto init_module_out;
    }

    return 0;

init_module_out:
    if (dev) 
        free_netdev(dev);

    return err;
}

static __exit void dvb_net_cleanup_module(void)
{
    printk(KERN_INFO "dvb net cleanup\n");
    unregister_netdev(dev);
}

module_init(dvb_net_init_module);
module_exit(dvb_net_cleanup_module);
*/
