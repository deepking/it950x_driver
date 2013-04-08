/**
 * Copyright (c) 2013 ITE Corporation. All rights reserved. 
 *
 * Module Name:
 *   it950x-core.c
 *
 * Abstract:
 *   The core of IT950x serial driver.
 */
 
#include "it950x-core.h"
#include "modulatorIocontrol.h"
#include "dvb_net.h"

/* Get a minor range for devices from the usb maintainer */
#define USB_it913x_MINOR_RANGE 47
#define USB_it950x_MINOR_RANGE 16
#ifdef CONFIG_USB_DYNAMIC_MINORS
#define USB_it913x_MINOR_BASE	0
#define USB_it950x_MINOR_BASE	USB_it913x_MINOR_RANGE
#else
#define USB_it913x_MINOR_BASE	192
#define USB_it950x_MINOR_BASE	192 + USB_it913x_MINOR_RANGE
#endif

#if URB_TEST
unsigned int loop_cnt = 0;
unsigned int diff_time_tx_write = 0;
unsigned int min_1 = 0;
struct timeval now, start;	
#endif
	
static DEFINE_MUTEX(it950x_mutex);
static DEFINE_MUTEX(it950x_urb_kill);
static DEFINE_MUTEX(it950x_rb_mutex);

/* Structure for urb context */
typedef struct it950x_urb_context{
	struct it950x_dev *dev;
	Byte index;
} URBContext;

/* Structure to hold all of our device specific stuff */
struct it950x_dev {
	struct usb_device *	usbdev;			    /* the usb device for this device */
	struct usb_interface *	interface;		/* the interface for this device */
//	unsigned char *		bulk_in_buffer;		/* the buffer to receive data */
//	size_t			bulk_in_size;		    /* the size of the receive buffer */
//	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
//	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	struct kref		kref;
	struct file *file;
	struct file *tx_file;
	DEVICE_CONTEXT DC;	
	Byte tx_on;
	Byte rx_on;
	Byte rx_chip_minor;
	Byte tx_chip_minor;	
	Byte g_AP_use;
	atomic_t g_AP_use_tx;
	atomic_t g_AP_use_rx;
	atomic_t tx_pw_on;	
	atomic_t rx_pw_on;	
	Bool DeviceReboot, DevicePower;	
	Bool TunerInited0, TunerInited1;	
	Byte is_use_low_brate;
	atomic_t urb_counter_low_brate;

	/* USB URB Related for TX*/
	int	   tx_urb_streaming;
	struct urb *tx_urbs[URB_COUNT_TX];
	URBContext tx_urb_context[URB_COUNT_TX];
	Byte tx_urbstatus[URB_COUNT_TX];
	Byte tx_urb_index;
	atomic_t tx_urb_counter;
	spinlock_t TxRBKeyLock;		
	Byte* pTxRingBuffer;           // Tx Ring Buffer Address.
	Dword TxCurrBuffPointAddr;     // Output urb addr.
	Dword TxWriteBuffPointAddr;	   // Entry of Ring Buffer.
	Dword dwTxWriteTolBufferSize;  // Total ringbuffer size.
	Dword dwTxRemaingBufferSize;   // Remaining size in buffer.

	/* USB URB Related for TX low bitrate*/
	int	   tx_urb_streaming_low_brate;
	struct urb *tx_urbs_low_brate[URB_COUNT_TX_LOW_BRATE];
	URBContext tx_urb_context_low_brate[URB_COUNT_TX_LOW_BRATE];
	Byte tx_urbstatus_low_brate[URB_COUNT_TX_LOW_BRATE];
	Byte tx_urb_index_low_brate;
	Byte tx_urb_use_count_low_brate;	
	Byte* pTxRingBuffer_low_brate;
	Byte* pWriteFrameBuffer_low_brate;
	Dword* pTxCurrBuffPointAddr_low_brate;	
	Dword* pTxWriteBuffPointAddr_low_brate;	
	Dword dwTxWriteTolBufferSize_low_brate;
	Dword dwTxRemaingBufferSize_low_brate;	
	
	/* USB URB Related for TX_CMD */
	int	   tx_urb_streaming_cmd;
	struct urb *tx_urbs_cmd[URB_COUNT_TX_CMD];
	URBContext tx_urb_context_cmd[URB_COUNT_TX_CMD];
	Byte tx_urbstatus_cmd[URB_COUNT_TX_CMD];
	Byte tx_urb_index_cmd;
	//Byte urb_use_count_cmd;
	Byte* pTxRingBuffer_cmd;
	Byte* pWriteFrameBuffer_cmd;
	Dword* pTxCurrBuffPointAddr_cmd;	
	Dword* pTxWriteBuffPointAddr_cmd;	
	Dword dwTxWriteTolBufferSize_cmd;
	Dword dwTxRemaingBufferSize_cmd;	

	/* USB URB Related for RX */
	int    rx_urb_streaming;
	struct urb *rx_urbs[URB_COUNT_RX];
	URBContext rx_urb_context[URB_COUNT_RX];
	Byte rx_urbstatus[URB_COUNT_RX];
	atomic_t rx_urb_counter;
	Byte rx_urb_index;
	Byte rx_first_urb_reset;
	spinlock_t RxRBKeylock;
	Byte *pRxRingBuffer;
	Dword RxCurrBuffPointAddr;
	Dword RxReadBuffPointAddr;
	Dword dwRxTolBufferSize;
	Dword dwRxReadRemaingBufferSize;

        dvb_netdev* dvbdev;
};

#define to_afa_dev(d) container_of(d, struct it950x_dev, kref)

static struct usb_driver  it950x_driver;

struct usb_device_id it950x_usb_id_table[] = {
		{ USB_DEVICE(0x048D,0x9507) },
		{ 0},		/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, it950x_usb_id_table);

/* AirHD */
Dword Tx_RMRingBuffer(struct it950x_urb_context *context, Dword dwDataFrameSize)
{
	struct it950x_dev *dev = context->dev;
	Dword dwBuffLen = 0;
	unsigned long flags = 0;	

#if RB_DEBUG
	deb_data("Enter %s", __func__);
	deb_data("Tx_RMRingBuffer: dev->TxCurrBuffPointAddr %lu", dev->TxCurrBuffPointAddr);
#endif
	spin_lock_irqsave(&dev->TxRBKeyLock, flags);
		dev->TxCurrBuffPointAddr = (dev->TxCurrBuffPointAddr + dwDataFrameSize) % (dev->dwTxWriteTolBufferSize);
		dev->dwTxRemaingBufferSize += dwDataFrameSize;
	spin_unlock_irqrestore(&dev->TxRBKeyLock, flags);
		
	dev->tx_urbstatus[context->index] = 0;
	atomic_add(1, &dev->tx_urb_counter);
	return 0;
}

/* AirHD for low bitrate */
Dword Tx_RMRingBuffer_low_brate(struct it950x_urb_context *context, Dword dwDataFrameSize)
{
	struct it950x_dev *dev = context->dev;
	Dword dwBuffLen = 0;
	//deb_data("enter %s", __func__);
	//deb_data("Tx_RMRingBuffer: (*dev->pTxCurrBuffPointAddr) %d", (*dev->pTxCurrBuffPointAddr));
	
	dwBuffLen = (dev->dwTxWriteTolBufferSize_low_brate) - (*dev->pTxCurrBuffPointAddr_low_brate);
	
	if(dwBuffLen >= dwDataFrameSize){
		(*dev->pTxCurrBuffPointAddr_low_brate) += dwDataFrameSize;
		if((*dev->pTxCurrBuffPointAddr_low_brate) >= (dev->dwTxWriteTolBufferSize_low_brate)) (*dev->pTxCurrBuffPointAddr_low_brate) = 0;
	}else{
		(*dev->pTxCurrBuffPointAddr_low_brate) = 0;
		(*dev->pTxCurrBuffPointAddr_low_brate) += (dwDataFrameSize - dwBuffLen);
	}
	dev->dwTxRemaingBufferSize_low_brate += dwDataFrameSize;
	//deb_data("RMRing URB complete index: %d", context->index);
	//dev->tx_urbstatus[context->index] = 0;
	dev->tx_urbstatus_low_brate[context->index] = 0;
	atomic_add(1, &dev->urb_counter_low_brate);
	//dev->urb_use_count++;
	//if(dev->urb_use_count < 1) 
		//deb_data("urb index is %d---------- back, urb_use_count = %d\n", context->index, dev->urb_use_count);
	return 0;
}

/* AirHD for cmd */
Dword Tx_RMRingBuffer_cmd(struct it950x_urb_context *context, Dword dwDataFrameSize)
{
	struct it950x_dev *dev = context->dev;
	Dword dwBuffLen = 0;
	//deb_data("enter %s", __func__);
	//deb_data("Tx_RMRingBuffer: (*dev->pTxCurrBuffPointAddr) %d", (*dev->pTxCurrBuffPointAddr));
	
	dwBuffLen = (dev->dwTxWriteTolBufferSize_cmd) - (*dev->pTxCurrBuffPointAddr_cmd);
	
	if (dwBuffLen >= dwDataFrameSize) {
		(*dev->pTxCurrBuffPointAddr_cmd) += dwDataFrameSize;
		if((*dev->pTxCurrBuffPointAddr_cmd) >= (dev->dwTxWriteTolBufferSize_cmd)) (*dev->pTxCurrBuffPointAddr_cmd) = 0;
	} else {
		(*dev->pTxCurrBuffPointAddr_cmd) = 0;
		(*dev->pTxCurrBuffPointAddr_cmd) += (dwDataFrameSize - dwBuffLen);
	}
	dev->dwTxRemaingBufferSize_cmd += dwDataFrameSize;
	//deb_data("RMRing URB complete index: %d", context->index);
	dev->tx_urbstatus[context->index] = 0;
	atomic_add(1, &dev->tx_urb_counter);
	return 0;
}

void Rx_FillRingBuffer(
	struct it950x_dev *dev,
	Dword data_size)
{ 
	unsigned long flags = 0;
	
	/*
	 * dwRxCurrBuffPointAddr : Return_urb addr (allow reading addr)
	 * dwRxTolBufferSize : total ringbuffer size
	 */
	spin_lock_irqsave(&dev->RxRBKeylock, flags);
		dev->RxCurrBuffPointAddr = (dev->RxCurrBuffPointAddr + data_size) % (dev->dwRxTolBufferSize);
		dev->dwRxReadRemaingBufferSize += data_size;
	spin_unlock_irqrestore(&dev->RxRBKeylock, flags);
	
	atomic_add(1, &dev->rx_urb_counter);
	
	if(atomic_read(&dev->rx_urb_counter) == URB_COUNT_RX){
		deb_data("\t %s warning: RingBuffer already full\n", __func__);
	}
}

static void
rx_free_urbs(struct it950x_dev *dev)
{
	int i;
	//deb_data("Enter %s Function\n",__FUNCTION__);
	
	for (i = 0; i < URB_COUNT_RX; i++) {
		usb_free_urb(dev->rx_urbs[i]);
		dev->rx_urbs[i] = NULL;
	}
	
	deb_data("%s() end\n", __func__);
}
static void
tx_free_urbs(struct it950x_dev *dev)
{
	int i;
	//deb_data("Enter %s Function\n",__FUNCTION__);
	
	for (i = 0; i < URB_COUNT_TX; i++) {
		usb_free_urb(dev->tx_urbs[i]);
		dev->tx_urbs[i] = NULL;
	}
	for(i = 0; i < URB_COUNT_TX_CMD; i++){
		usb_free_urb(dev->tx_urbs_cmd[i]);
		dev->tx_urbs_cmd[i] = NULL;
	}
	for(i = 0; i < URB_COUNT_TX_LOW_BRATE; i++){
		usb_free_urb(dev->tx_urbs_low_brate[i]);
		dev->tx_urbs_low_brate[i] = NULL;
	}			
	deb_data("%s() end\n", __func__);
}
/*
static void
rx_kill_busy_urbs(struct it950x_dev *dev)
{
	int i;
	//deb_data("Enter %s Function\n",__FUNCTION__);
	
	for (i = 0; i < URB_COUNT_RX; i++) {
		if(dev->urbstatus_rx[i] == 1){
			usb_kill_urb(dev->urbs_rx[i]);
			deb_data("kill rx urb index %d\n", i);
		}
	}
	
	deb_data("%s() end\n", __func__);
}
static void
tx_kill_busy_urbs(struct it950x_dev *dev)
{
	int i;
	//deb_data("Enter %s Function\n",__FUNCTION__);
	
	for (i = 0; i < URB_COUNT_TX; i++) {
		if(dev->tx_urbstatus[i] == 1){
			usb_kill_urb(dev->tx_urbs[i]);
			deb_data("kill tx urb index %d\n", i);
		}
	}
	
	deb_data("%s() end\n", __func__);
}
*/
static int rx_stop_urb_transfer(struct it950x_dev *dev)
{
	int i;

	//deb_data("%s()\n", __func__);

	if (!dev->rx_urb_streaming) {
		deb_data("%s: iso xfer already stop!\n", __func__);
		return 0;
	}

	dev->rx_urb_streaming = 0;
	
	mdelay(1000);//waiting urb complete.

	/*DM368 usb bus error when using kill urb */
#if 0
	//mutex_lock(&it950x_urb_kill);	     /* if urb doesn't call back, kill it. */
	for (i = 0; i < URB_COUNT_RX; i++) {
		if(dev->rx_urbstatus[i] == 1){
			usb_kill_urb(dev->rx_urbs[i]);
			deb_data("kill rx urb index %d\n", i);
		}
	}
	//mutex_unlock(&it950x_urb_kill);		
#endif

	deb_data("%s() end\n", __func__);

	return 0;
}

static int tx_stop_urb_transfer(struct it950x_dev *dev)
{
	//deb_data("%s()\n", __func__);

	if (!dev->tx_urb_streaming && !dev->tx_urb_streaming_low_brate) {
		deb_data("%s: iso xfer already stop!\n", __func__);
		return 0;
	}
#if URB_TEST
	min_1 = 0;
#endif	
	dev->tx_urb_streaming = 0;
	dev->tx_urb_streaming_low_brate = 0;
	
	/*DM368 usb bus error when using kill urb */
#if 0
	for (i = 0; i < URB_COUNT_TX; i++) {
		if(dev->tx_urbstatus[i] == 1) {
			dev->tx_urbstatus[i] = 0;
			usb_kill_urb(dev->tx_urbs[i]);
		}
		usb_free_urb(dev->tx_urbs[i]);
		dev->tx_urbs[i] = NULL;
	}

	mutex_lock(&it950x_urb_kill);
	tx_kill_busy_urbs(dev);
	mutex_unlock(&it950x_urb_kill);
#endif

	deb_data("%s() end\n", __func__);

	return 0;
}

static int tx_stop_urb_transfer_cmd(struct it950x_dev *dev)
{
	deb_data("%s()\n", __func__);

	if (!dev->tx_urb_streaming_cmd) {
		deb_data("%s: iso xfer already stop!\n", __func__);
		return 0;
	}
	
	dev->tx_urb_streaming_cmd = 0;
	
	/*DM368 usb bus error when using kill urb */
#if 0
	for (i = 0; i < URB_COUNT_TX; i++) {
		if(dev->tx_urbstatus[i] == 1) {
			dev->tx_urbstatus[i] = 0;
			usb_kill_urb(dev->tx_urbs[i]);
		}
		usb_free_urb(dev->tx_urbs[i]);
		dev->tx_urbs[i] = NULL;
	}

	mutex_lock(&it950x_urb_kill);
	tx_kill_busy_urbs(dev);
	mutex_unlock(&it950x_urb_kill);
#endif

	deb_data("%s() end\n", __func__);

	return 0;
}

DWORD fabs_self(DWORD a, DWORD b)
{
	DWORD c = 0;
	
	c = a - b;
	if(c >= 0) return c;
	else return c * -1;
}

/**
 * Tx_RingBuffer - Eagle the core of write Ring-Buffer.
 * Description:
 * Successful submissions return 0(Error_NO_ERROR) and submised buffer length.
 * Otherwise this routine returns a negative error number.
 */
DWORD Tx_RingBuffer(
	struct it950x_dev *dev,
    Byte* pBuffer,
    Dword* pBufferLength)
{
    Dword dwBuffLen = 0;
    Dword dwCpBuffLen = *pBufferLength;
    Dword dwCurrBuffAddr = dev->TxCurrBuffPointAddr;
    Dword dwWriteBuffAddr = dev->TxWriteBuffPointAddr;
    int ret = -ENOMEM;
	unsigned long flags = 0;    

#if RB_DEBUG
	deb_data("RemaingBufferSize: {%lu}\n", dev->dwTxRemaingBufferSize);		
	deb_data("Tx_RingBuffer-CPLen %lu, dwCurrBuffAddr %lu, dwWriteBuffAddr %lu\n", dwCpBuffLen, dwCurrBuffAddr, dwWriteBuffAddr);
#endif
	/* RingBuffer full */
	if ((dev->dwTxRemaingBufferSize) == 0) {
		*pBufferLength = 0;
#if RB_DEBUG		
		deb_data("dwTxRemaingBufferSize = 0\n");
#endif
		return Error_NO_ERROR;
	}

    if ((dev->dwTxRemaingBufferSize) < dwCpBuffLen) {
		*pBufferLength = 0;
#if RB_DEBUG
		deb_data("dwTxRemaingBufferSize < dwCpBuffLen\n");
#endif
		return Error_NO_ERROR;
	}

    if (*pBufferLength == 0) {
        return -Error_BUFFER_INSUFFICIENT;
    }
	
	/* memory must enough because checking at first in this function */
	if(dwWriteBuffAddr >= dwCurrBuffAddr){      
		/* To_kernel_urb not run a cycle or both run a cycle */
		dwBuffLen = dev->dwTxWriteTolBufferSize - dwWriteBuffAddr; //remaining memory (to buffer end), not contain buffer beginning to return urb
		if(dwBuffLen >= dwCpBuffLen){    // For not cycle case.
			/* end remaining memory is enough */
			memcpy(dev->pTxRingBuffer + dwWriteBuffAddr, pBuffer, dwCpBuffLen);
		}
		else{   /* For cycle case */
			/* use all end memory, run a cycle and need use beginning memory */
			memcpy(dev->pTxRingBuffer + dwWriteBuffAddr, pBuffer, dwBuffLen); //using end memory
			memcpy(dev->pTxRingBuffer, pBuffer + dwBuffLen, dwCpBuffLen - dwBuffLen); //using begining memory
		}
	}
	else{       
		/* To_kernel_urb run a cycle and Return_urb not */
		memcpy(dev->pTxRingBuffer + dwWriteBuffAddr, pBuffer, dwCpBuffLen);
	}

	spin_lock_irqsave(&dev->TxRBKeyLock, flags);
		dev->TxWriteBuffPointAddr = (dev->TxWriteBuffPointAddr + dwCpBuffLen) % (dev->dwTxWriteTolBufferSize);
		dev->dwTxRemaingBufferSize -= dwCpBuffLen;
	spin_unlock_irqrestore(&dev->TxRBKeyLock, flags);

	if(dev->tx_urb_streaming == 1){
		/* allow submit urb */
		while((dwWriteBuffAddr - (dev->tx_urb_index * URB_BUFSIZE_TX)) >= URB_BUFSIZE_TX && atomic_read(&dev->tx_urb_counter) > 0){
			/* while urb full and not submit */
			ret = usb_submit_urb(dev->tx_urbs[dev->tx_urb_index], GFP_ATOMIC);
			if (ret != 0) {
				tx_stop_urb_transfer(dev);
#if RB_DEBUG				
				deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
#endif
				return ret;
			}
			
			dev->tx_urbstatus[dev->tx_urb_index] = 1;
			dev->tx_urb_index = (dev->tx_urb_index + 1) % URB_COUNT_TX;
			atomic_sub(1, &dev->tx_urb_counter);
		}
	}
   
    return Error_NO_ERROR;
}

/* AirHD low bitrate */
DWORD Tx_RingBuffer_low_brate(
	struct it950x_dev *dev,
    Byte*  pBuffer,
    Dword* pBufferLength)
{
    DWORD dwBuffLen = 0;
    DWORD dwCpBuffLen = *pBufferLength;
    DWORD dwCurrBuffAddr = (*dev->pTxCurrBuffPointAddr_low_brate);
    DWORD dwWriteBuffAddr = (*dev->pTxWriteBuffPointAddr_low_brate);
    int ret = -ENOMEM;
    int i;

//	deb_data("Tx_RingBuffer-CPLen %d, dwCurrBuffAddr %d, dwWriteBuffAddr %d, dev->urb_back %d\n", dwCpBuffLen, dwCurrBuffAddr, dwWriteBuffAddr, dev->urb_back);
	if(dev->tx_urb_streaming_low_brate == 1 && (dwWriteBuffAddr - (dev->tx_urb_index_low_brate * URB_BUFSIZE_TX_LOW_BRATE))>= URB_BUFSIZE_TX_LOW_BRATE && (atomic_read(&dev->urb_counter_low_brate)) > 0/*dev->urb_use_count > 0*/){
		ret = usb_submit_urb(dev->tx_urbs_low_brate[dev->tx_urb_index_low_brate], GFP_ATOMIC);
		if (ret != 0){
			tx_stop_urb_transfer(dev);
			deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
			return ret;
		}//else deb_data("usb_submit_urb ok \n");
		
		//if (atomic_read(&dev->urb_counter_low_brate) < 5) deb_data("\nless_than_10_can_urb_use_count = %d\n", atomic_read(&dev->urb_counter_low_brate));
		dev->tx_urbstatus_low_brate[dev->tx_urb_index_low_brate] = 1;
		dev->tx_urb_index_low_brate++;
		atomic_sub(1, &dev->urb_counter_low_brate);
		if(dev->tx_urb_index_low_brate == URB_COUNT_TX_LOW_BRATE) dev->tx_urb_index_low_brate = 0;
	}

	/*RingBuffer full*/
	if ((dev->dwTxRemaingBufferSize_low_brate) == 0){
		*pBufferLength = 0;
		udelay(100);
		return Error_NO_ERROR;
	}
	
    if ((dev->dwTxRemaingBufferSize_low_brate) < dwCpBuffLen){
		*pBufferLength = 0;
		udelay(100);
		return Error_NO_ERROR;
	}

    if (*pBufferLength == 0){
		udelay(100);
        return Error_BUFFER_INSUFFICIENT;
    }

    if (dwCurrBuffAddr <= (*dev->pTxWriteBuffPointAddr_low_brate)) {
		dwBuffLen = dev->dwTxWriteTolBufferSize_low_brate - (*dev->pTxWriteBuffPointAddr_low_brate);

		if(dwCpBuffLen <= dwBuffLen){ 
			/*It will NOT TOUCH end of ring buffer*/
			memcpy(dev->pWriteFrameBuffer_low_brate+(*dev->pTxWriteBuffPointAddr_low_brate), pBuffer, dwCpBuffLen);
			*pBufferLength = dwCpBuffLen;
			(*dev->pTxWriteBuffPointAddr_low_brate) = dwWriteBuffAddr; // FIX memcy will modify pTxWriteBuffPointAddr value
			(*dev->pTxWriteBuffPointAddr_low_brate) += dwCpBuffLen;
			dev->dwTxRemaingBufferSize_low_brate = dev->dwTxRemaingBufferSize_low_brate - dwCpBuffLen;
		}else{
			/*It will TOUCH end of ring buffer*/
			
#if 0
			memcpy(dev->pTxRingBuffer+(*dev->pTxWriteBuffPointAddr), pBuffer, dwBuffLen);
			*pBufferLength = dwBuffLen;
			(*dev->pTxWriteBuffPointAddr) = 0;
			//dwWriteBuffAddr = 0;
			pBuffer += dwBuffLen;
			dwCpBuffLen -= dwBuffLen;
			dev->dwTxRemaingBufferSize -= dwBuffLen;
#endif
			if(dwBuffLen > 0){
				memcpy(dev->pTxRingBuffer+(*dev->pTxWriteBuffPointAddr_low_brate), pBuffer, dwBuffLen);
				*pBufferLength = dwBuffLen;
				(*dev->pTxWriteBuffPointAddr_low_brate) = 0;
				pBuffer += dwBuffLen;
				dwCpBuffLen -= dwBuffLen;
				dev->dwTxRemaingBufferSize_low_brate -= dwBuffLen;
			}else{
				
				(*dev->pTxWriteBuffPointAddr_low_brate) = 0;
			}

			memcpy(dev->pWriteFrameBuffer_low_brate+(*dev->pTxWriteBuffPointAddr_low_brate), pBuffer, dwCpBuffLen);
			*pBufferLength += dwBuffLen;
			(*dev->pTxWriteBuffPointAddr_low_brate) = 0;
			(*dev->pTxWriteBuffPointAddr_low_brate) += dwCpBuffLen;

			dev->dwTxRemaingBufferSize_low_brate = dev->dwTxRemaingBufferSize_low_brate - dwCpBuffLen;
		}
    }else {
        dwBuffLen = dwCurrBuffAddr - (*dev->pTxWriteBuffPointAddr_low_brate);

        if (dwCpBuffLen > dwBuffLen) {
            *pBufferLength = 0;
			udelay(100);
			return Error_NO_ERROR;
        }
        else {
            memcpy(dev->pWriteFrameBuffer_low_brate+(*dev->pTxWriteBuffPointAddr_low_brate), pBuffer, dwCpBuffLen);
			*pBufferLength = dwCpBuffLen;
			(*dev->pTxWriteBuffPointAddr_low_brate) = dwWriteBuffAddr;
			(*dev->pTxWriteBuffPointAddr_low_brate) += dwCpBuffLen;
			
			dev->dwTxRemaingBufferSize_low_brate = dev->dwTxRemaingBufferSize_low_brate - dwCpBuffLen;
        }
    }
    return Error_NO_ERROR;
}

/* AirHD_CMD */
DWORD Tx_RingBuffer_cmd(
	struct it950x_dev *dev,
    Byte* pBuffer,
    Dword* pBufferLength)
{
    DWORD dwCpBuffLen = *pBufferLength;
    //DWORD dwCurrBuffAddr = (*dev->pTxCurrBuffPointAddr_cmd);
    //DWORD dwWriteBuffAddr = (*dev->pTxWriteBuffPointAddr_cmd);
    int ret = -ENOMEM;
    //int i;
         
    memcpy(dev->pWriteFrameBuffer_cmd, pBuffer, dwCpBuffLen);
    
		mutex_lock(&it950x_urb_kill);
		ret = usb_submit_urb(dev->tx_urbs_cmd[dev->tx_urb_index_cmd], GFP_ATOMIC);
		mutex_unlock(&it950x_urb_kill);
		if (ret != 0){
			tx_stop_urb_transfer_cmd(dev);
			deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
			return ret;
		}else deb_data("usb_submit_urb cmd ok \n");
		//if(dev->urb_use_count_cmd < 1) 
			//deb_data("urb index is %d-------------------send, urb_use_count = %d\n", dev->tx_urb_index_cmd, dev->urb_use_count_cmd);
		dev->tx_urbstatus_cmd[dev->tx_urb_index_cmd] = 1;

		if(dev->tx_urb_index_cmd == URB_COUNT_TX_CMD) dev->tx_urb_index_cmd = 0;

    return Error_NO_ERROR;
}

DWORD Rx_RingBuffer(
	struct it950x_dev *dev,
	Byte *pBuffer,
	Dword *pBufferLength)
{
	Dword dwCpBuffLen = *pBufferLength;
	Dword dwCurrBuffAddr = dev->RxCurrBuffPointAddr;
	Dword dwReadBuffAddr = dev->RxReadBuffPointAddr;
	Dword dwBuffLen = 0;
	unsigned long flags = 0;
	int ret = -ENOMEM;
	
	//deb_data("- Enter %s Function -\n", __func__);
	if((dev->dwRxReadRemaingBufferSize) == 0){
		*pBufferLength = 0;
		//deb_data("\t Warning: ReadRemaingBufferSize = 0\n");
		return Error_NO_ERROR;
	}

    if(*pBufferLength == 0){
		*pBufferLength = 0;
        return Error_BUFFER_INSUFFICIENT;
    }
    
    if((dev->dwRxReadRemaingBufferSize) < dwCpBuffLen){
		*pBufferLength = 0;
		//deb_data("\t Warning: ReadRemaingBufferSize(%lu) < Request(%lu)\n", dev->dwRxReadRemaingBufferSize, dwCpBuffLen);
		return Error_NO_ERROR;
	}
	
	/*
	 * dwReadBuffAddr : already read addr
	 * dwCurrBuffAddr : Return_urb addr (allow reading addr)
	 * dwTolBufferSize : total ringbuffer size
	 */
	
	/* memory must enough because already checking */
	
	//RingBuffer
	if(dwReadBuffAddr >= dwCurrBuffAddr){
		//Return_urb run a cycle and already_read not
		dwBuffLen = dev->dwRxTolBufferSize - dwReadBuffAddr; //remaining memory (to buffer end), not contain buffer beginning to Return_urb
		if(dwBuffLen >= dwCpBuffLen){
			//end remaining memory is enough
			memcpy(pBuffer, dev->pRxRingBuffer + dwReadBuffAddr, dwCpBuffLen);
		}
		else{
			//use all end memory, run a cycle and need use beginning memory
			memcpy(pBuffer, dev->pRxRingBuffer + dwReadBuffAddr, dwBuffLen); //using end memory
			memcpy(pBuffer + dwBuffLen, dev->pRxRingBuffer, dwCpBuffLen - dwBuffLen); //using begining memory
		}
	}
	else{
		//Return_urb not run a cycle or both run a cycle
		memcpy(pBuffer, dev->pRxRingBuffer + dwReadBuffAddr, dwCpBuffLen);
	}
	
	spin_lock_irqsave(&dev->RxRBKeylock, flags);
		dev->RxReadBuffPointAddr = (dev->RxReadBuffPointAddr + dwCpBuffLen) % (dev->dwRxTolBufferSize);
		dev->dwRxReadRemaingBufferSize -= dwCpBuffLen;
	spin_unlock_irqrestore(&dev->RxRBKeylock, flags);
	
	//Submit urb
	if(dev->rx_urb_streaming == 1 && dev->rx_first_urb_reset == 1){
                //deb_data("Rx_RingBuffer::submit_urb() RxReadBufferPointAddr=%lu rx_urb_index=%d rx_urb_counter=%d\n",
                //        dev->RxReadBuffPointAddr, dev->rx_urb_index, atomic_read(&dev->rx_urb_counter));
		//allow submit urb && first urb already resubmit
		while((dev->RxReadBuffPointAddr - (dev->rx_urb_index * URB_BUFSIZE_RX)) >= URB_BUFSIZE_RX && atomic_read(&dev->rx_urb_counter) > 0){
			//while urb empty and not submit
			ret = usb_submit_urb(dev->rx_urbs[dev->rx_urb_index], GFP_ATOMIC);
			if (ret != 0) {
				rx_stop_urb_transfer(dev);
				deb_data("\t Error: urb[%d] resubmit fail, err = %d\n",dev->rx_urb_index , ret);
				return ret;
			}
			
			dev->rx_urbstatus[dev->rx_urb_index] = 1;
			dev->rx_urb_index = (dev->rx_urb_index + 1) % URB_COUNT_RX;
			atomic_sub(1, &dev->rx_urb_counter);
		}
	}
	
	return Error_NO_ERROR;
}

/******************************************************************/

static void tx_urb_completion(struct urb *purb)
{
	struct it950x_urb_context *context = purb->context;
	int ptype = usb_pipetype(purb->pipe);
	
#if RB_DEBUG	
	deb_data("Enter %s", __func__);

	deb_data("urb id: %d. '%s' urb completed. status: %d, length: %d/%d, pack_num: %d, errors: %d\n",
		context->index,
		ptype == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
		purb->status,purb->actual_length,purb->transfer_buffer_length,
		purb->number_of_packets,purb->error_count);
#endif		
	//context->dev->tx_urbstatus[context->index] = 0;
	switch (purb->status) {
		case 0:         /* success */
		case -ETIMEDOUT:    /* NAK */
			break;
		case -ECONNRESET:   /* kill */
		case -ENOENT:
			//context->dev->urb_use_count++;
			deb_data("TX ENOENT-urb completition error %d.\n", purb->status);
		case -ESHUTDOWN:
			return;
		default:        /* error */
			deb_data("TX urb completition error %d.\n", purb->status);
			break;
	}

	if (!context->dev) return;
	
	if (context->dev->tx_urb_streaming == 0) return;

	if (ptype != PIPE_BULK) {
		deb_data("TX %s() Unsupported URB type %d\n", __func__, ptype);
		return;
	}
	
	//ptr = (u8 *)purb->transfer_buffer;
	/* Feed the transport payload into the kernel demux */
	//dvb_dmx_swfilter_packets(&dev->dvb.demux,
	//	purb->transfer_buffer, purb->actual_length / 188);
	//if (purb->actual_length > 0)
	
	Tx_RMRingBuffer(context, URB_BUFSIZE_TX);
	
	return;
}

static void tx_urb_completion_low_brate(struct urb *purb)
{
	struct it950x_urb_context *context = purb->context;
	int ptype = usb_pipetype(purb->pipe);
	int ret = -ENOMEM;
	int i;
	
	//deb_data("enter %s", __func__);

	//deb_data("'%s' urb completed. status: %d, length: %d/%d, pack_num: %d, errors: %d\n",
	//	ptype == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
	//	purb->status,purb->actual_length,purb->transfer_buffer_length,
	//	purb->number_of_packets,purb->error_count);
	//context->dev->tx_urbstatus[context->index] = 0;
	switch (purb->status) {
		case 0:         /* success */
		case -ETIMEDOUT:    /* NAK */
			break;
		case -ECONNRESET:   /* kill */
		case -ENOENT:
			context->dev->tx_urb_use_count_low_brate++;
			deb_data("TX ENOENT-tx_urb_completion_low_brate error %d.\n", purb->status);
		case -ESHUTDOWN:
			return;
		default:        /* error */
			deb_data("tx_urb_completion_low_brate error %d.\n", purb->status);
			break;
	}

	if (!context->dev)
		return;
	
	if (context->dev->tx_urb_streaming_low_brate == 0)
		return;

	if (ptype != PIPE_BULK) {
		deb_data("TX %s() Unsupported URB type %d\n", __func__, ptype);
		return;
	}
	
	//ptr = (u8 *)purb->transfer_buffer;

	/* Feed the transport payload into the kernel demux */
	//dvb_dmx_swfilter_packets(&dev->dvb.demux,
	//	purb->transfer_buffer, purb->actual_length / 188);
	//if (purb->actual_length > 0)
	
	Tx_RMRingBuffer_low_brate(context, URB_BUFSIZE_TX_LOW_BRATE);
	return;
}

static void tx_urb_completion_cmd(struct urb *purb)
{
	struct it950x_urb_context *context = purb->context;
	int ptype = usb_pipetype(purb->pipe);
	deb_data("enter %s", __func__);

	//deb_data("'%s' urb completed. status: %d, length: %d/%d, pack_num: %d, errors: %d\n",
	//	ptype == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
	//	purb->status,purb->actual_length,purb->transfer_buffer_length,
	//	purb->number_of_packets,purb->error_count);
	
	context->dev->tx_urbstatus_cmd[context->index] = 0;
	switch (purb->status) {
		case 0:         /* success */
		case -ETIMEDOUT:    /* NAK */
			break;
		case -ECONNRESET:   /* kill */
		case -ENOENT:
			//context->dev->urb_use_count_cmd++;
			deb_data("TX ENOENT-urb completition error %d.\n", purb->status);
		case -ESHUTDOWN:
			return;
		default:        /* error */
			deb_data("TX urb completition error %d.\n", purb->status);
			break;
	}

	if (!context->dev) return;
	
	if (context->dev->tx_urb_streaming_cmd == 0) return;

	if (ptype != PIPE_BULK) {
		deb_data("TX %s() Unsupported URB type %d\n", __func__, ptype);
		return;
	}
	
	//ptr = (u8 *)purb->transfer_buffer;
	/* Feed the transport payload into the kernel demux */
	//dvb_dmx_swfilter_packets(&dev->dvb.demux,
	//	purb->transfer_buffer, purb->actual_length / 188);
	//if (purb->actual_length > 0)
	
	Tx_RMRingBuffer_cmd(context, URB_BUFSIZE_TX_CMD);
	
	return;
}

static void rx_urb_completion(struct urb *purb)
{
	struct it950x_urb_context *context = purb->context;
	struct it950x_dev *dev = context->dev;	
	int ptype = usb_pipetype(purb->pipe);
	int ret = -ENOMEM;

	//deb_data("%s", __func__);
	//deb_data("'%s' urb completed. status: %d, length: %d/%d, pack_num: %d, errors: %d\n",
	//	ptype == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
	//	purb->status,purb->actual_length,purb->transfer_buffer_length,
	//	purb->number_of_packets,purb->error_count);
	//deb_data("urb_complete(%d)\n", context->index);

	if(purb->actual_length != URB_BUFSIZE_RX)	
		deb_data("\t Warning: Rx_urb[%d].length = %d\n", context->index, purb->actual_length);
			
		
	context->dev->rx_urbstatus[context->index] = 0;
	switch (purb->status) {
		case 0:              /* Success */
			break;
		case -ETIMEDOUT:     /* NAK */
			deb_data("RX ETIMEDOUT -urb completition error %d.\n", purb->status);
			break;
		case -ECONNRESET:    /* unlink */
			deb_data("RX ECONNRESET -urb completition error %d.\n", purb->status);
			return;
		case -ENOENT:        /* kill */
			deb_data("RX ENOENT -urb completition error %d.\n", purb->status);
			return;
		case -ESHUTDOWN:
			deb_data("RX ESHUTDOWN -urb completition error %d.\n", purb->status);
			return;
		default:             /* error */
			deb_data("RX urb completition error %d.\n", purb->status);
			break;
	}

	if (!dev){
		deb_data("dev is NULL\n");
		return;
	}

	if (ptype != PIPE_BULK) {
		deb_data("RX %s() Unsupported URB type %d\n", __func__, ptype);
		return;
	}

	/* For solve first urb not correct problem */
	if(dev->rx_urb_streaming){
		if(!dev->rx_first_urb_reset){
			ret = usb_submit_urb(dev->rx_urbs[0], GFP_ATOMIC);
			if(ret != 0){
				rx_stop_urb_transfer(dev);
				deb_data("\t%s error: urb[0] resubmit fail, err = %d\n", __func__ , ret);
				return ;
			}
			else{
				//deb_data("\t urb[0] resubmit success\n");
			}
			
			dev->rx_urbstatus[0] = 1;
			dev->rx_urb_index = (dev->rx_urb_index + 1) % URB_COUNT_RX;
			dev->rx_first_urb_reset = 1;
		}
		else{
			Rx_FillRingBuffer(dev, URB_BUFSIZE_RX);
		}
	}
	else{
		if(context->index != 0)
			Rx_FillRingBuffer(dev, URB_BUFSIZE_RX);
	}
}

/* AirHD */
static int tx_start_urb_transfer(struct it950x_dev *dev)
{
	struct urb *purb;
	int i, ret = -ENOMEM;

	//deb_data("%s()\n", __func__);
#if URB_TEST
	min_1 = 0;
#endif	
	if (dev->tx_urb_streaming || dev->tx_urb_streaming_low_brate) {
		deb_data("%s: iso xfer already running!\n", __func__);
		return 0;
	}
	
	dev->TxCurrBuffPointAddr	= 0;
	dev->TxWriteBuffPointAddr = 0;

	*dev->pTxCurrBuffPointAddr_low_brate = 0;
	*dev->pTxWriteBuffPointAddr_low_brate = 0;

	for (i = 0; i < URB_COUNT_TX; i++) {
		purb = dev->tx_urbs[i];

		purb->transfer_buffer = dev->pTxRingBuffer + (URB_BUFSIZE_TX*i);
		if (!purb->transfer_buffer) {
			usb_free_urb(purb);
			dev->tx_urbs[i] = NULL;
			goto err;
		}
		
		dev->tx_urb_context[i].index = i;
		dev->tx_urb_context[i].dev = dev;
		dev->tx_urbstatus[i] = 0;
		
		purb->status = -EINPROGRESS;
		usb_fill_bulk_urb(purb,
				  dev->usbdev,
				  usb_sndbulkpipe(dev->usbdev, 0x06),
				  purb->transfer_buffer,
				  URB_BUFSIZE_TX,
				  tx_urb_completion,
				   &dev->tx_urb_context[i]);
		
		purb->transfer_flags = 0;
	}
	for (i = 0; i < URB_COUNT_TX_LOW_BRATE; i++) {

		//dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		//if (!dev->urbs[i])
		//	goto err;

		purb = dev->tx_urbs_low_brate[i];

		purb->transfer_buffer = dev->pWriteFrameBuffer_low_brate + (URB_BUFSIZE_TX_LOW_BRATE * i);
		if (!purb->transfer_buffer) {
			usb_free_urb(purb);
			dev->tx_urbs_low_brate[i] = NULL;
			goto err;
		}
		
		dev->tx_urb_context_low_brate[i].index = i;
		dev->tx_urb_context_low_brate[i].dev = dev;
		dev->tx_urbstatus_low_brate[i] = 0;
		
		purb->status = -EINPROGRESS;
		usb_fill_bulk_urb(purb,
				  dev->usbdev,
				  usb_sndbulkpipe(dev->usbdev, 0x06),
				  purb->transfer_buffer,
				  URB_BUFSIZE_TX_LOW_BRATE,
				  tx_urb_completion_low_brate,
				  &dev->tx_urb_context_low_brate[i]);
		
		purb->transfer_flags = 0;
	}	
	dev->dwTxRemaingBufferSize = dev->dwTxWriteTolBufferSize;
	dev->tx_urb_index = 0;
	dev->tx_urb_streaming = 1;
	atomic_set(&dev->tx_urb_counter, URB_COUNT_TX);

	dev->dwTxRemaingBufferSize_low_brate = dev->dwTxWriteTolBufferSize_low_brate;
	dev->tx_urb_index_low_brate = 0;
	dev->tx_urb_use_count_low_brate = URB_COUNT_TX_LOW_BRATE;
	dev->tx_urb_streaming_low_brate = 1;

	ret = 0;

	deb_data("%s() end\n", __func__);

err:
	return ret;
}

/* AirHD_CMD */
static int tx_start_urb_transfer_cmd(struct it950x_dev *dev)
{
	struct urb *purb;
	int i, ret = -ENOMEM;

	deb_data("%s()\n", __func__);

	if (dev->tx_urb_streaming_cmd) {
		deb_data("%s: iso xfer already running!\n", __func__);
	//	return 0;
	}
	
	*dev->pTxCurrBuffPointAddr_cmd = 0;
	*dev->pTxWriteBuffPointAddr_cmd = 0;

	for (i = 0; i < URB_COUNT_TX_CMD; i++) {

		//dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		//if (!dev->urbs[i])
		//	goto err;

		purb = dev->tx_urbs_cmd[i];

		purb->transfer_buffer = dev->pWriteFrameBuffer_cmd + (URB_BUFSIZE_TX_CMD*i);
		if (!purb->transfer_buffer) {
			usb_free_urb(purb);
			dev->tx_urbs_cmd[i] = NULL;
			goto err;
		}
		
		dev->tx_urb_context_cmd[i].index = i;
		dev->tx_urb_context_cmd[i].dev = dev;
		dev->tx_urbstatus_cmd[i] = 0;
		
		purb->status = -EINPROGRESS;
		usb_fill_bulk_urb(purb,
				  dev->usbdev,
				  usb_sndbulkpipe(dev->usbdev, 0x06),
				  purb->transfer_buffer,
				  URB_BUFSIZE_TX_CMD,
				  tx_urb_completion_cmd,
				   &dev->tx_urb_context_cmd[i]);
		
		purb->transfer_flags = 0;
	}
	dev->dwTxRemaingBufferSize_cmd = dev->dwTxWriteTolBufferSize_cmd;
	dev->tx_urb_index_cmd = 0;
	dev->tx_urb_streaming_cmd = 1;
	ret = 0;

	deb_data("%s() end\n", __func__);

err:
	return ret;
}


static void rx_start_urb_transfer(struct it950x_dev *dev)
{
	int urb_index = 0, ret = -ENOMEM;

	deb_data("- Enter %s Function -\n", __func__);

	dev->rx_urb_index = 0;
	dev->RxCurrBuffPointAddr = URB_BUFSIZE_RX;	//first time skip urb[0]
	dev->RxReadBuffPointAddr = URB_BUFSIZE_RX;	//first time skip urb[0]
	dev->rx_first_urb_reset = 0;
	dev->dwRxReadRemaingBufferSize = 0;
	atomic_set(&dev->rx_urb_counter, URB_COUNT_RX);
	
	for(urb_index = 0; urb_index < URB_COUNT_RX; urb_index++){
		ret = usb_submit_urb(dev->rx_urbs[urb_index], GFP_ATOMIC);
		if(ret != 0){
			rx_stop_urb_transfer(dev);
			deb_data("\t AError: urb[%d] submit fail, err = %d\n", urb_index, ret);
			return ;
		}
		else{
			//deb_data("\t urb[%d] submit success\n", urb_index);
		}
		
		dev->rx_urbstatus[urb_index] = 1;
		dev->rx_urb_index = (dev->rx_urb_index + 1) % URB_COUNT_RX;
		atomic_sub(1, &dev->rx_urb_counter);
	}
	
	dev->rx_urb_streaming = 1;
	return ;
}
/*
static void afa_delete(struct kref *kref)
{
	struct it950x_dev *dev = to_afa_dev(kref);

	usb_put_dev(dev->usbdev);
//	kfree (dev->bulk_in_buffer);
	kfree (dev);
}
*/
static int it950x_usb_open(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev;
	struct usb_interface *interface;
	int subminor, mainsubminor;
	int retval = 0;
	int error, order, urb_index;

	deb_data("it950x_usb_rx_open function\n");
	mainsubminor = iminor(inode);
	subminor = iminor(inode) + USB_it913x_MINOR_RANGE;
	interface = usb_find_interface(&it950x_driver, subminor);

try:
	while (!interface) {
		subminor++;
		interface = usb_find_interface(&it950x_driver, subminor);
		if (subminor >= mainsubminor + USB_it913x_MINOR_RANGE)
			break;
	}	
	
	if (!interface) {
		deb_data("%s - error, can't find device for minor %d",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}
	
	dev = usb_get_intfdata(interface);
	if (!dev) {
		deb_data("usb_get_intfdata fail!\n");
		retval = -ENODEV;
		goto exit;
	}
	
	if (subminor != dev->tx_chip_minor) {
		interface = NULL;
		goto try;
	}	
	deb_data("open RX subminor: %d\n", subminor);		

	retval = it950x_usb_rx_alloc_dev(dev);
	
	/* increment our usage count for the device */
	//kref_get(&dev->kref);
	
	/* save our object in the file's private structure */
	dev->file = file;
	file->private_data = dev;
	
exit:
	return retval;
}

static int it950x_usb_tx_open(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev;
	struct usb_interface *interface;
	int mainsubminor, subminor;
	int retval = 0;
	int error;

	deb_data("it950x_usb_tx_open function\n");
	
	mainsubminor = iminor(inode);	
	subminor = iminor(inode);
	interface = usb_find_interface(&it950x_driver, subminor);

try:
	while (!interface) {
		subminor++;
		interface = usb_find_interface(&it950x_driver, subminor);
		if (subminor >= mainsubminor + USB_it950x_MINOR_RANGE)
			break;
	}		
	
	if (!interface) {
		deb_data("%s - error, can't find device for minor %d",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}
	dev = usb_get_intfdata(interface);
	
	if (!dev) {
		deb_data("usb_get_intfdata fail!\n");
		retval = -ENODEV;
		goto exit;
	}
	
	if (subminor != dev->tx_chip_minor) {
		interface = NULL;
		goto try;
	}	
	deb_data("open TX subminor: %d\n", subminor);			
	
	retval = it950x_usb_tx_alloc_dev(dev);
	
	/* save our object in the file's private structure */
	dev->tx_file = file;
	file->private_data = dev;
exit:
	return retval;
}

static int it950x_usb_release(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev;
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_rx_free_dev(dev);
}

static int it950x_usb_tx_release(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev;

	dev = (struct it950x_dev *)file->private_data;
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}	

	return it950x_usb_tx_free_dev(dev);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
static int it950x_usb_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd,  unsigned long parg)
{
	struct it950x_dev *dev;
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_rx_ioctl_dev(dev, cmd, parg);
}
#endif

int SetLowBitRateTransfer(struct it950x_dev *dev, void *parg)
{
	//unsigned char b_buf[188];
	DWORD dwError = Error_NO_ERROR;
	int act_len;
	
	PSetLowBitRateTransferRequest pRequest = (PSetLowBitRateTransferRequest) parg;
	
	//deb_data("SetLowBitRateTransfer function %d\n", pRequest->pdwBufferLength);
	deb_data("--------is_use_low_brate [%s]\n", pRequest->pdwBufferLength?"ON":"OFF");
	dev->is_use_low_brate = pRequest->pdwBufferLength;
		
	return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
static int it950x_usb_tx_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd,  unsigned long parg)
{
	struct it950x_dev *dev;
	PCmdRequest pRequest;
	
	deb_data("it950x_usb_tx_ioctl function\n");
	
	dev = (struct it950x_dev *)file->private_data;
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}

	return it950x_usb_tx_ioctl_dev(dev);
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
long it950x_usb_unlocked_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long parg)
{
	struct it950x_dev *dev;
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_rx_unlocked_ioctl_dev(dev, cmd, parg);
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
long it950x_usb_tx_unlocked_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long parg)
{
	struct it950x_dev *dev;
	PCmdRequest pRequest;	
	//deb_data("it950x_usb_ioctl function\n");

	dev = (struct it950x_dev *)file->private_data;
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}

        return it950x_usb_tx_unlocked_ioctl_dev(dev, cmd, parg);
}
#endif


static ssize_t it950x_read(
	struct file *file, 
	char __user *buf,
	size_t count, 
	loff_t *ppos)
{
	struct it950x_dev *dev = (struct it950x_dev *)file->private_data;

        return it950x_usb_rx_read_dev(dev, buf, count);
}

static ssize_t it950x_tx_write(
	struct file *file,
	const char __user *user_buffer,
	size_t count, 
	loff_t *ppos)
{
	struct it950x_dev *dev;

	Dword Len = count;
	DWORD dwError = Error_NO_ERROR;

	/*AirHD RingBuffer*/
	dev = (struct it950x_dev *)file->private_data;
	if (dev == NULL)
		return -ENODEV;
#if URB_TEST
	loop_cnt++;	
#endif
	dwError = Tx_RingBuffer(dev, (Byte*)user_buffer, &Len);
	//printk("[%lu]\n", Len);
	if(dwError != 0) return dwError;
	else return Len;

	/*AirHD Bulk Msg*/
#if 0
	copy_from_user(b_buf, user_buffer, count);

	dwError = usb_bulk_msg(usb_get_dev(dev0->DC->modulator.userData),
			usb_sndbulkpipe(usb_get_dev(dev0->DC->modulator.userData), 0x06),
			b_buf,
			count,
			&act_len,
			100000);
   
	if(dev->is_use_low_brate) {
		//deb_data("it950x_tx_write: g_is_use_low_brate is open!\n");
		dwError = Tx_RingBuffer_low_brate(dev, user_buffer, &Len);
	}else{
		//deb_data("it950x_tx_write: g_is_use_low_brate is close!\n");
		dwError = Tx_RingBuffer(dev, user_buffer, &Len);
	}   
	if (dwError) deb_data("--------Usb2_writeControlBus fail : 0x%08lx\n", dwError);
	
	return count;
#endif
}


static struct file_operations it950x_usb_fops ={
	.owner =		THIS_MODULE,
	.open =		it950x_usb_open,
	.read =		it950x_read,
	//.write =	it950x_write,
	.release =	it950x_usb_release,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
		.unlocked_ioctl = it950x_usb_unlocked_ioctl,
#else
		.ioctl = it950x_usb_ioctl,
#endif

};

static struct file_operations it950x_usb_tx_fops ={
	.owner =		THIS_MODULE,
	.open =		it950x_usb_tx_open,
	//.read =		it950x_read,
	.write =	it950x_tx_write, /*AirHD*/ 
	.release =	it950x_usb_tx_release,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
		.unlocked_ioctl = it950x_usb_tx_unlocked_ioctl,
#else
		.ioctl = it950x_usb_tx_ioctl,
#endif

};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver it950x_class = {
	.name =			"usb-it913x%d",
	.fops =			&it950x_usb_fops,
	.minor_base = 	USB_it913x_MINOR_BASE
};

static struct usb_class_driver it950x_class_tx = {
	.name =			"usb-it950x%d",
	.fops =			&it950x_usb_tx_fops,
	.minor_base = 	USB_it950x_MINOR_BASE
};


// Register a USB device node end


static int it950x_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct it950x_dev *dev = NULL;
	int retval = -ENOMEM;
	int i;

	/* allocate memory for our device state and intialize it */
	dev = kzalloc(sizeof(struct it950x_dev), GFP_KERNEL);
	dev->g_AP_use = 0;
	//dev->DC.modulator.ptrIQtableEx = kzalloc(sizeof(IQtable)*91, GFP_KERNEL);
	atomic_set(&dev->g_AP_use_tx, 0);
	atomic_set(&dev->g_AP_use_rx, 0);	

	if (dev == NULL) {
		deb_data("Out of memory\n");
		return retval;
	}
	
	dev->usbdev = interface_to_usbdev(intf);

	/* we can register the device now, as it is ready */
	usb_set_intfdata(intf, dev);

	//memset(&DC, 0, sizeof(DC));
	deb_data("===it950x usb device pluged in!! ===\n");
	retval = Device_init(interface_to_usbdev(intf), &dev->DC, true);
	if (retval) {
		deb_data("Device_init Fail: 0x%08x\n", retval);
		return retval;
	}
	
	/* we can register the device now, as it is ready */
	retval = usb_register_dev(intf, &it950x_class);
	if (retval) {
		deb_data("Not able to get a minor for this device.");
		usb_set_intfdata(intf, NULL);
		//goto error;
		return -ENOTTY;
	}
	dev->rx_chip_minor = intf->minor;
	deb_data("rx minor %d \n", dev->rx_chip_minor);
	intf->minor = -1;
	retval = usb_register_dev(intf, &it950x_class_tx);
	if (retval) {
		deb_data("Not able to get a minor for this device.");
		usb_set_intfdata(intf, NULL);
		//goto error;
		return -ENOTTY;
	}
	dev->tx_chip_minor = intf->minor;
	deb_data("tx minor %d \n", dev->tx_chip_minor);

	/*Allocate TX TX_CMD urb*/
	for (i = 0; i < URB_COUNT_TX_CMD; i++) {

		dev->tx_urbs_cmd[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->tx_urbs_cmd[i])
			retval = -ENOMEM;
	}
	for (i = 0; i < URB_COUNT_TX; i++) {

		dev->tx_urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->tx_urbs[i])
			retval = -ENOMEM;
	}
	for(i = 0; i < URB_COUNT_TX_LOW_BRATE; i++){

		dev->tx_urbs_low_brate[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->tx_urbs_low_brate[i])
			retval = -ENOMEM;
	}	
	
#if !(defined(DTVCAM_POWER_CTRL) && defined(AVSENDER_POWER_CTRL))
	atomic_set(&dev->tx_pw_on, 0);	
	atomic_set(&dev->rx_pw_on, 0);	
	DL_ApPwCtrl(&dev->DC, 1, 0);	
	DL_ApPwCtrl(&dev->DC, 0, 0);	
#endif	
	deb_data("USB ITEtech device now attached to USBSkel-%d \n", intf->minor);

	dev->dvbdev = dvb_alloc_netdev(dev);
	if (dev->dvbdev == NULL)
	    deb_data("ERROR: dvb_alloc_netdev");

	return retval;
}

static int it950x_suspend(struct usb_interface *intf, pm_message_t state)
{
	int error;
	struct it950x_dev *dev;

	dev = usb_get_intfdata(intf);
	if (!dev) 
		deb_data("dev = NULL");
	//deb_data("Enter %s Function\n", __FUNCTION__);

#ifdef EEEPC
	error = DL_Reboot();
#else
    if (dev->DevicePower) {
		error = DL_CheckTunerInited(&dev->DC, 0, &dev->TunerInited0);
		error = DL_CheckTunerInited(&dev->DC, 1, &dev->TunerInited1);

		error = DL_ApCtrl(&dev->DC, 0, 0);
		error = DL_ApCtrl(&dev->DC, 1, 0);
		if (error) {
			deb_data("DL_ApCtrl error : 0x%x\n", error);
		}

		dev->DeviceReboot = true;
    }
#endif
	
	return 0;
}

static int it950x_resume(struct usb_interface *intf)
{
	int retval = -ENOMEM;
	int error;
	struct it950x_dev *dev;

	//deb_data("Enter %s Function\n",__FUNCTION__);
	dev = usb_get_intfdata(intf);
	if (!dev) 
		deb_data("dev = NULL");


#ifdef EEEPC
#else
    if (dev->DeviceReboot == True) {
		retval = Device_init(interface_to_usbdev(intf),&dev->DC, false);
		if(retval)
			deb_data("Device_init Fail: 0x%08x\n", retval);

		if (dev->TunerInited0)
			error = DL_ApCtrl(&dev->DC, 0, 1);
		if (dev->TunerInited1)
			error = DL_ApCtrl(&dev->DC, 1, 1);
    }
#endif
	
    return 0;
}

static void it950x_disconnect(struct usb_interface *intf)
{
	struct it950x_dev *dev;
	int minor = intf->minor;

	deb_data("%s()\n", __func__);
	/* prevent afa_open() from racing afa_disconnect() */
	//lock_kernel();
	mutex_lock(&it950x_mutex);

	dev = usb_get_intfdata(intf);
	if (!dev) {
		deb_data("dev = NULL");
	}
	
	/*DM368 usb bus error when using kill urb */
#if 0	
	mutex_lock(&it950x_urb_kill);
	
	if(dev->tx_on) tx_kill_busy_urbs(dev);
	if(dev->rx_on) rx_kill_busy_urbs(dev);
	
	mutex_unlock(&it950x_urb_kill);
#endif

	tx_free_urbs(dev);
//	rx_free_urbs(dev);
	
	usb_set_intfdata(intf, NULL);

	/* give back our minor */
	
	intf->minor = dev->rx_chip_minor;
	usb_deregister_dev(intf, &it950x_class);

	intf->minor = dev->tx_chip_minor;
	usb_deregister_dev(intf, &it950x_class_tx);

	mutex_unlock(&it950x_mutex);
	//unlock_kernel();
	
	/* decrement our usage count */
	//	kref_put(&dev->kref, afa_delete);

	if (dev){
		if(dev->file) dev->file->private_data = NULL;
		if(dev->tx_file) dev->tx_file->private_data = NULL;
		if (dev->dvbdev) {
                    dvb_free_netdev(dev->dvbdev);
                }
		kfree(dev);
	}
	
	deb_data("USB ITEtech #%d now disconnected", minor);
}


static struct usb_driver it950x_driver = {
#if LINUX_VERSION_CODE <=  KERNEL_VERSION(2,6,15)
	.owner = THIS_MODULE,
#endif
	.name       = "usb-it950x",
	.probe      = it950x_probe,
	.disconnect = it950x_disconnect,//dvb_usb_device_exit,
	.id_table   = it950x_usb_id_table,
	.suspend    = it950x_suspend,
	.resume     = it950x_resume,
};

static int __init it950x_module_init(void)
{
	int result;

	//info("%s",__FUNCTION__);
	//deb_data("usb_it950x Module is loaded \n");

	if ((result = usb_register(&it950x_driver))) {
		err("usb_register failed. Error number %d",result);
		return result;
	}
	
	return 0;
}

static void __exit it950x_module_exit(void)
{
	deb_data("usb_it950x Module is unloaded!\n");
	usb_deregister(&it950x_driver);
}

int it950x_usb_rx_alloc_dev(struct it950x_dev *dev)
{
    int retval = 0;
    int error = 0;
    int urb_index;

    if (!dev) {
        deb_data("ERROR: NULL device\n");
        retval = -ENODEV;
        goto exit;
    }

    /* Ring buffer alloc & chip init */
    spin_lock_init(&dev->RxRBKeylock);
    dev->rx_urb_streaming = 0;
    dev->dwRxReadRemaingBufferSize = 0;
    dev->dwRxTolBufferSize = URB_BUFSIZE_RX * URB_COUNT_RX;
    dev->rx_urb_index = 0;
    dev->rx_first_urb_reset = 0;

    atomic_set(&dev->rx_urb_counter, URB_COUNT_RX);
    dev->pRxRingBuffer = (Byte*)__get_free_pages(GFP_KERNEL, get_order(dev->dwRxTolBufferSize));

    /*Allocate RX urb*/	
    for (urb_index = 0; urb_index < URB_COUNT_RX; urb_index++) {
        dev->rx_urbs[urb_index] = usb_alloc_urb(0, GFP_KERNEL);
        if (!dev->rx_urbs[urb_index])
            retval = -ENOMEM;

        dev->rx_urbs[urb_index]->transfer_buffer = dev->pRxRingBuffer + (URB_BUFSIZE_RX * urb_index);

        if(!dev->rx_urbs[urb_index]->transfer_buffer){
            usb_free_urb(dev->rx_urbs[urb_index]);
            dev->rx_urbs[urb_index] = NULL;
            retval = -ENOMEM;
            goto exit;
        }

        dev->rx_urb_context[urb_index].index = urb_index;
        dev->rx_urbstatus[urb_index] = 0;
        dev->rx_urb_context[urb_index].dev = dev;
        dev->rx_urbs[urb_index]->status = -EINPROGRESS;

        usb_fill_bulk_urb(dev->rx_urbs[urb_index],
                dev->usbdev,
                usb_rcvbulkpipe(dev->usbdev, 0x85),
                dev->rx_urbs[urb_index]->transfer_buffer,
                URB_BUFSIZE_RX,
                rx_urb_completion,
                &dev->rx_urb_context[urb_index]);

        dev->rx_urbs[urb_index]->transfer_flags = 0;
    }	

#if !(defined(DTVCAM_POWER_CTRL) && defined(AVSENDER_POWER_CTRL))	
    if(atomic_read(&dev->rx_pw_on) == 0) {
        if(atomic_read(&dev->tx_pw_on) == 0) {
            error = DL_ApPwCtrl(&dev, 0, 1);
            error = DL_ApPwCtrl(&dev->DC, 1, 1);
            atomic_set(&dev->tx_pw_on, 1);	
            atomic_set(&dev->rx_pw_on, 1);				
        } else {		
            error = DL_ApPwCtrl(&dev->DC, 1, 1);
            atomic_set(&dev->rx_pw_on, 1);	
        }
    }	
#endif	
    atomic_add(1, &dev->g_AP_use_rx);	

exit:
    return retval;
}

int it950x_usb_rx_free_dev(struct it950x_dev *dev)
{
	int error = 0;
	int i;

	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}
	
	if(atomic_read(&dev->g_AP_use_rx) == 1) {
		rx_stop_urb_transfer(dev);

/*		if(dev->g_AP_use == 1) {
			dev->DevicePower = 0;
			error = DL_ApCtrl(&dev->DC, 0, 0);
			error = DL_ApCtrl(&dev->DC, 1, 0);
		}
		dev->g_AP_use--;*/
		
		if (dev->pRxRingBuffer)
			free_pages((long unsigned int)dev->pRxRingBuffer, get_order(dev->dwRxTolBufferSize));
	

		for (i = 0; i < URB_COUNT_RX; i++) {   /* if urb doesn't call back, kill it. */
			if(dev->rx_urbstatus[i] == 1){
				usb_kill_urb(dev->rx_urbs[i]);
			}
		}	
		rx_free_urbs(dev);	

#if !(defined(DTVCAM_POWER_CTRL) && defined(AVSENDER_POWER_CTRL))
		if(atomic_read(&dev->rx_pw_on) == 1) {
			if(atomic_read(&dev->g_AP_use_tx) == 0) {    // normal.
				error = DL_ApPwCtrl(&dev->DC, 1, 0);
				error = DL_ApPwCtrl(&dev->DC, 0, 0);
				atomic_set(&dev->rx_pw_on, 0);	
				atomic_set(&dev->tx_pw_on, 0);				
			} else {		                                 // if tx in used. just off rx.
				error = DL_ApPwCtrl(&dev->DC, 1, 0);
				atomic_set(&dev->rx_pw_on, 0);	
			}
		}	
#endif		
	}
	atomic_sub(1, &dev->g_AP_use_rx);
	return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
int it950x_usb_rx_ioctl_dev(struct it950x_dev *dev, unsigned int cmd,  unsigned long parg)
{
    Byte temp0 = 0
        Byte temp1 = 1;
    Dword status;

    if (dev == NULL) {
        deb_data("dev is NULL\n");
        return -ENODEV;
    }

    switch (cmd)
    {	
        case IOCTL_ITE_DEMOD_STARTCAPTURE:
            rx_start_urb_transfer(dev);
            return 0;		

        case IOCTL_ITE_DEMOD_STOPCAPTURE:
            rx_stop_urb_transfer(dev);
            return 0;

            //clean 12 bytes garbage through 9507(TX)	
        case IOCTL_ITE_DEMOD_ADDPIDAT:
            status = IT9507_writeRegisters ((Modulator*)&dev->DC.modulator, Processor_OFDM, 0xF9A4, 1, &temp1);
            if(status)
                deb_data("DTV_WriteRegOFDM() return error!\n");

            status = IT9507_writeRegisters ((Modulator*)&dev->DC.modulator, Processor_OFDM, 0xF9CC, 1, &temp0);
            if(status)
                deb_data("DTV_WriteRegOFDM() return error!\n");

            status = IT9507_writeRegisters ((Modulator*)&dev->DC.modulator, Processor_OFDM, 0xF9A4, 1, &temp0);
            if(status)
                deb_data("DTV_WriteRegOFDM() return error!\n");

            status = IT9507_writeRegisters ((Modulator*)&dev->DC.modulator, Processor_OFDM, 0xF9CC, 1, &temp1);
            if(status)
                deb_data("DTV_WriteRegOFDM() return error!\n");

            break;
    }
    return DL_DemodIOCTLFun((void *)&dev->DC.demodulator, (DWORD)cmd, parg);
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
long it950x_usb_rx_unlocked_ioctl_dev(struct it950x_dev *dev, unsigned int cmd, unsigned long parg)
{
    if (dev == NULL) {
        deb_data("dev is NULL\n");
        return -ENODEV;
    }

    switch (cmd)
    {	
        case IOCTL_ITE_DEMOD_STARTCAPTURE:
            rx_start_urb_transfer(dev);
            return 0;

        case IOCTL_ITE_DEMOD_STOPCAPTURE:
            rx_stop_urb_transfer(dev);
            return 0;
    }

    return DL_DemodIOCTLFun((void *)&dev->DC.demodulator, (DWORD)cmd, parg);
}
#endif

ssize_t it950x_usb_rx_read_dev(
        struct it950x_dev *dev,
	char __user *buf,
	size_t count)
{
    Dword Len = count;

    if (dev == NULL) 
        return -ENODEV;

    Rx_RingBuffer(dev, buf, &Len);
    //deb_data("ReadRingBuffer - %d\n", Len);

    return Len;
}

int it950x_usb_tx_alloc_dev(struct it950x_dev *dev)
{
    int retval = 0;
    int order;
    int order_cmd;
    int error;

    if (!dev) {
        deb_data("usb_get_intfdata fail!\n");
        retval = -ENODEV;
        goto exit;
    }

    atomic_set(&dev->tx_urb_counter, URB_COUNT_TX);
    atomic_set(&dev->urb_counter_low_brate, URB_COUNT_TX_LOW_BRATE);

#if URB_TEST		
    do_gettimeofday(&start);	
#endif

    /*kzalloc will limit by Embedded system*/
#if 0
    dev->dwTxWriteTolBufferSize = URB_BUFSIZE_TX * URB_COUNT_TX;
    dev->pTxRingBuffer = kzalloc(dev->dwTxWriteTolBufferSize + 8, GFP_KERNEL);
    if (dev->pTxRingBuffer) {
        dev->pWriteFrameBuffer = dev->pTxRingBuffer + 8;
        dev->pTxCurrBuffPointAddr = dev->pTxRingBuffer;
        dev->pTxWriteBuffPointAddr = dev->pTxRingBuffer + 4;
        dev->dwTxRemaingBufferSize = dev->dwTxWriteTolBufferSize;
        dev->tx_urb_index = 0;
        //dev->urb_use_count = URB_COUNT_TX;
    }
#endif

    /*Write Ring buffer alloc*/
    dev->dwTxWriteTolBufferSize = URB_BUFSIZE_TX * URB_COUNT_TX;
    order = get_order(dev->dwTxWriteTolBufferSize);
    dev->pTxRingBuffer = (Byte*)__get_free_pages(GFP_KERNEL, order);
    if (dev->pTxRingBuffer) {
        //		dev->pWriteFrameBuffer = dev->pTxRingBuffer + 8;
        dev->TxCurrBuffPointAddr = 0;
        dev->TxWriteBuffPointAddr = 0;
        dev->dwTxRemaingBufferSize = dev->dwTxWriteTolBufferSize;
        dev->tx_urb_index = 0;
        //dev->urb_use_count = URB_COUNT_TX;
    }

    /*Write cmd Ring buffer alloc*/
    dev->dwTxWriteTolBufferSize_cmd = URB_BUFSIZE_TX_CMD * URB_COUNT_TX_CMD;
    order_cmd = get_order(dev->dwTxWriteTolBufferSize_cmd + 8);
    dev->pTxRingBuffer_cmd = (Byte*)__get_free_pages(GFP_KERNEL, order_cmd);
    if (dev->pTxRingBuffer_cmd) {
        dev->pWriteFrameBuffer_cmd = dev->pTxRingBuffer_cmd + 8;
        dev->pTxCurrBuffPointAddr_cmd = (Dword*)dev->pTxRingBuffer_cmd;
        dev->pTxWriteBuffPointAddr_cmd = (Dword*)(dev->pTxRingBuffer_cmd + 4);
        dev->dwTxRemaingBufferSize_cmd = dev->dwTxWriteTolBufferSize_cmd;
        dev->tx_urb_index_cmd = 0;
        //dev->urb_use_count_cmd = URB_COUNT_TX_CMD;
    }

    /*Write low bitrate Ring buffer alloc*/
    dev->dwTxWriteTolBufferSize_low_brate = URB_BUFSIZE_TX_LOW_BRATE * URB_COUNT_TX_LOW_BRATE;
    order = get_order(dev->dwTxWriteTolBufferSize_low_brate + 8);
    dev->pTxRingBuffer_low_brate = (Byte*)__get_free_pages(GFP_KERNEL, order);
    if (dev->pTxRingBuffer_low_brate) {
        dev->pWriteFrameBuffer_low_brate = dev->pTxRingBuffer_low_brate + 8;
        dev->pTxCurrBuffPointAddr_low_brate = (Dword*)dev->pTxRingBuffer_low_brate;
        dev->pTxWriteBuffPointAddr_low_brate = (Dword*)(dev->pTxRingBuffer_low_brate + 4);
        dev->dwTxRemaingBufferSize_low_brate = dev->dwTxWriteTolBufferSize_low_brate;
        dev->tx_urb_index_low_brate = 0;
        dev->tx_urb_use_count_low_brate = URB_COUNT_TX_LOW_BRATE;
    }

#if !(defined(DTVCAM_POWER_CTRL) && defined(AVSENDER_POWER_CTRL))
    if(atomic_read(&dev->tx_pw_on) == 0) {
        error = DL_ApPwCtrl(&dev->DC, 0, 1);
        atomic_set(&dev->tx_pw_on, 1);	
    }
#endif		

    atomic_add(1, &dev->g_AP_use_tx);
exit:
    return retval;
}

int it950x_usb_tx_free_dev(struct it950x_dev *dev)
{
    int error;
    int order;

    if (dev == NULL) {
        deb_data("dev is NULL\n");
        return -ENODEV;
    }	

    if(	atomic_read(&dev->g_AP_use_tx) == 1) {
        tx_stop_urb_transfer(dev);

        /* decrement the count on our device */
        //kref_put(&dev->kref, afa_delete);

        if (dev->pTxRingBuffer){
            order = get_order(dev->dwTxWriteTolBufferSize);
            free_pages((long unsigned int)dev->pTxRingBuffer, order);
        }

        if (dev->pTxRingBuffer_cmd){
            order = get_order(dev->dwTxWriteTolBufferSize_cmd + 8);
            free_pages((long unsigned int)dev->pTxRingBuffer_cmd, order);
        }

        if (dev->pTxRingBuffer_low_brate){
            order = get_order(dev->dwTxWriteTolBufferSize_low_brate + 8);
            free_pages((long unsigned int)dev->pTxRingBuffer_low_brate, order);
        }			

#if !(defined(DTVCAM_POWER_CTRL) && defined(AVSENDER_POWER_CTRL))		
        if(atomic_read(&dev->tx_pw_on) == 1) {
            if(atomic_read(&dev->g_AP_use_rx) == 0) {   // RX not used, suspend tx.
                error = DL_ApPwCtrl(&dev->DC, 0, 0);
                atomic_set(&dev->tx_pw_on, 0);	
            } else {
                deb_data("RX lock TX_PowerSaving\n");
            }
        }
#endif		

    }
    atomic_sub(1, &dev->g_AP_use_tx);

    return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,35)
int it950x_usb_tx_ioctl_dev(struct it950x_dev *dev,
	unsigned int cmd,  unsigned long parg)
{
    PCmdRequest pRequest;

    if (dev == NULL) {
        deb_data("dev is NULL\n");
        return -ENODEV;
    }

    switch (cmd)
    {
        case IOCTL_ITE_DEMOD_STARTTRANSFER_TX:
            tx_start_urb_transfer(dev);
            return 0;

        case IOCTL_ITE_DEMOD_STOPTRANSFER_TX:
            tx_stop_urb_transfer(dev);
            return 0;

        case IOCTL_ITE_DEMOD_STARTTRANSFER_CMD_TX:
            tx_start_urb_transfer_cmd(dev);
            return 0;

        case IOCTL_ITE_DEMOD_STOPTRANSFER_CMD_TX:
            tx_stop_urb_transfer_cmd(dev);
            return 0;

        case IOCTL_ITE_DEMOD_WRITECMD_TX:
            pRequest = (PCmdRequest) parg;
            Tx_RingBuffer_cmd(dev, pRequest->cmd, pRequest->len);
            return 0;		

        case IOCTL_ITE_DEMOD_SETLOWBRATETRANS_TX:
            SetLowBitRateTransfer(dev, (void*)parg);
            return 0;
    }
    return DL_DemodIOCTLFun((void *)&dev->DC.modulator, (DWORD)cmd, parg);
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
long it950x_usb_tx_unlocked_ioctl_dev(
	struct it950x_dev *dev,
	unsigned int cmd,
	unsigned long parg)
{
    PCmdRequest pRequest;	

    if (dev == NULL) {
        deb_data("dev is NULL\n");
        return -ENODEV;
    }

    switch (cmd)
    {
        case IOCTL_ITE_DEMOD_STARTTRANSFER_TX:
            tx_start_urb_transfer(dev);
            return 0;

        case IOCTL_ITE_DEMOD_STOPTRANSFER_TX:
            tx_stop_urb_transfer(dev);
            return 0;

        case IOCTL_ITE_DEMOD_STARTTRANSFER_CMD_TX:
            tx_start_urb_transfer_cmd(dev);
            return 0;

        case IOCTL_ITE_DEMOD_STOPTRANSFER_CMD_TX:
            tx_stop_urb_transfer_cmd(dev);
            return 0;

        case IOCTL_ITE_DEMOD_WRITECMD_TX:
            pRequest = (PCmdRequest) parg;
            Tx_RingBuffer_cmd(dev, pRequest->cmd, pRequest->len);
            return 0;		

        case IOCTL_ITE_DEMOD_SETLOWBRATETRANS_TX:
            SetLowBitRateTransfer(dev, (void*)parg);
            return 0;
    }
    return DL_DemodIOCTLFun((void *)&dev->DC.modulator, (DWORD)cmd, parg);
}
#endif

ssize_t it950x_usb_tx_write_dev(
        struct it950x_dev *dev,
	const char __user *user_buffer,
	size_t count)
{

    Dword Len = count;
    DWORD dwError = Error_NO_ERROR;

    /*AirHD RingBuffer*/
    if (dev == NULL)
        return -ENODEV;
#if URB_TEST
    loop_cnt++;	
#endif
    dwError = Tx_RingBuffer(dev, (Byte*)user_buffer, &Len);
    //printk("[%lu]\n", Len);
    if(dwError != 0) {
        deb_data("Tx_RingBuffer error\n");
        return dwError;
    }
    else return Len;
}

module_init (it950x_module_init);
module_exit (it950x_module_exit);

MODULE_AUTHOR("Jason Dong <jason.dong@ite.com.tw>");
MODULE_DESCRIPTION("DTV Native Driver for Device Based on ITEtech it950x");
MODULE_VERSION(DRIVER_RELEASE_VERSION);
MODULE_LICENSE("GPL");
