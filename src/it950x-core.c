#include "it950x-core.h"
#include "modulatorIocontrol.h"
#include "dvb_net.h"

// Register a USB device node

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

/*Structure for urb context*/
struct it950x_urb_context{
	struct it950x_dev *dev;
	Byte index;
};

/* Structure to hold all of our device specific stuff */
struct it950x_dev {
	struct usb_device *	usbdev;			/* the usb device for this device */
	struct usb_interface *	interface;		/* the interface for this device */
//	unsigned char *		bulk_in_buffer;		/* the buffer to receive data */
//	size_t			bulk_in_size;		/* the size of the receive buffer */
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
	int		urb_tx_streaming;
	struct urb *urbs[URB_COUNT_TX];
	struct it950x_urb_context urb_context[URB_COUNT_TX];
	Byte urbstatus[URB_COUNT_TX];

	/* USB URB Related for TX low bitrate*/
	int		urb_tx_streaming_low_brate;
	struct urb *urbs_low_brate[URB_COUNT_TX_LOW_BRATE];
	struct it950x_urb_context urb_context_low_brate[URB_COUNT_TX_LOW_BRATE];
	Byte urbstatus_low_brate[URB_COUNT_TX_LOW_BRATE];
	
	/* USB URB Related for TX_CMD*/
	int		urb_tx_streaming_cmd;
	struct urb *urbs_cmd[URB_COUNT_TX_CMD];
	struct it950x_urb_context urb_context_cmd[URB_COUNT_TX_CMD];
	Byte urbstatus_cmd[URB_COUNT_TX_CMD];

	/* USB URB Related for RX*/
	int		urb_streaming;
	struct urb *urbs_rx[URB_COUNT_RX];
	struct it950x_urb_context urb_context_rx[URB_COUNT_RX];
	Byte urbstatus_rx[URB_COUNT_RX];

	/*Read ring buffer*/
	Byte* pRingBuffer;
	Byte* pFrameBuffer;
	Dword* pCurrBuffPointAddr;	
	Dword* pReadBuffPointAddr;	
	Dword dwTolBufferSize;

	/*Write ring buffer*/
	Byte* pWriteRingBuffer;
	Byte* pWriteFrameBuffer;
	Dword* pWriteCurrBuffPointAddr;	
	Dword* pWriteBuffPointAddr;	
	Dword dwWriteTolBufferSize;
	Dword dwRemaingBufferSize;	
	Byte urb_index;
	//Byte urb_use_count;
	atomic_t urb_counter;
	
	/*Write low bitrate ring buffer*/
	Byte* pWriteRingBuffer_low_brate;
	Byte* pWriteFrameBuffer_low_brate;
	Dword* pWriteCurrBuffPointAddr_low_brate;	
	Dword* pWriteBuffPointAddr_low_brate;	
	Dword dwWriteTolBufferSize_low_brate;
	Dword dwRemaingBufferSize_low_brate;	
	Byte urb_index_low_brate;
	Byte urb_use_count_low_brate;	
	
	/*Write ring buffer cmd*/
	Byte* pWriteRingBuffer_cmd;
	Byte* pWriteFrameBuffer_cmd;
	Dword* pWriteCurrBuffPointAddr_cmd;	
	Dword* pWriteBuffPointAddr_cmd;	
	Dword dwWriteTolBufferSize_cmd;
	Dword dwRemaingBufferSize_cmd;	
	Byte urb_index_cmd;
	//Byte urb_use_count_cmd;
	
	dvb_netdev* dvbdev;
};

#define to_afa_dev(d) container_of(d, struct it950x_dev, kref)

static struct usb_driver  it950x_driver;

struct usb_device_id it950x_usb_id_table[] = {
		{ USB_DEVICE(0x048D,0x9507) },
		{ 0},		/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, it950x_usb_id_table);

//AirHD
Dword RMRingBuffer(struct it950x_urb_context *context, Dword dwDataFrameSize)
{
	struct it950x_dev *dev = context->dev;
	Dword dwBuffLen = 0;
	//deb_data("enter %s", __func__);
	//deb_data("RMRingBuffer: (*dev->pWriteCurrBuffPointAddr) %d", (*dev->pWriteCurrBuffPointAddr));
	
	//---------------------test---------------------//
	/* 
	 * dwDataFrameSize : urb size
	 * dwWriteTolBufferSize : total ringbuffer size
	 * pWriteCurrBuffPointAddr : Return_urb addr
	 */
	//mutex_lock(&it950x_rb_mutex);
	(*dev->pWriteCurrBuffPointAddr) = ((*dev->pWriteCurrBuffPointAddr) + dwDataFrameSize) % (dev->dwWriteTolBufferSize);
	dev->urbstatus[context->index] = 0;
	atomic_add(dwDataFrameSize, (atomic_t*) &dev->dwRemaingBufferSize);
//	dev->dwRemaingBufferSize += dwDataFrameSize;

	//mutex_unlock(&it950x_rb_mutex);
	atomic_add(1, &dev->urb_counter);

	//-------------------test end-------------------//
	
#if 0 //mark old
	dwBuffLen = (dev->dwWriteTolBufferSize) - (*dev->pWriteCurrBuffPointAddr);
	
	if (dwBuffLen >= dwDataFrameSize) {
		(*dev->pWriteCurrBuffPointAddr) += dwDataFrameSize;
		if((*dev->pWriteCurrBuffPointAddr) >= (dev->dwWriteTolBufferSize)) (*dev->pWriteCurrBuffPointAddr) = 0;
	} else {
		(*dev->pWriteCurrBuffPointAddr) = 0;
		(*dev->pWriteCurrBuffPointAddr) += (dwDataFrameSize - dwBuffLen);
	}
	dev->dwRemaingBufferSize += dwDataFrameSize;
	//deb_data("RMRing URB complete index: %d", context->index);
	dev->urbstatus[context->index] = 0;
	atomic_add(1, &dev->urb_counter);
#endif //mark old end
	return 0;
}

//AirHD for low bitrate
Dword RMRingBuffer_low_brate(struct it950x_urb_context *context, Dword dwDataFrameSize)
{
	struct it950x_dev *dev = context->dev;
	Dword dwBuffLen = 0;
	//deb_data("enter %s", __func__);
	//deb_data("RMRingBuffer: (*dev->pWriteCurrBuffPointAddr) %d", (*dev->pWriteCurrBuffPointAddr));
	
	dwBuffLen = (dev->dwWriteTolBufferSize_low_brate) - (*dev->pWriteCurrBuffPointAddr_low_brate);
	
	if(dwBuffLen >= dwDataFrameSize){
		(*dev->pWriteCurrBuffPointAddr_low_brate) += dwDataFrameSize;
		if((*dev->pWriteCurrBuffPointAddr_low_brate) >= (dev->dwWriteTolBufferSize_low_brate)) (*dev->pWriteCurrBuffPointAddr_low_brate) = 0;
	}else{
		(*dev->pWriteCurrBuffPointAddr_low_brate) = 0;
		(*dev->pWriteCurrBuffPointAddr_low_brate) += (dwDataFrameSize - dwBuffLen);
	}
	dev->dwRemaingBufferSize_low_brate += dwDataFrameSize;
	//deb_data("RMRing URB complete index: %d", context->index);
	//dev->urbstatus[context->index] = 0;
	dev->urbstatus_low_brate[context->index] = 0;
	atomic_add(1, &dev->urb_counter_low_brate);
	//dev->urb_use_count++;
	//if(dev->urb_use_count < 1) 
		//deb_data("urb index is %d---------- back, urb_use_count = %d\n", context->index, dev->urb_use_count);
	return 0;
}

Dword RMRingBuffer_cmd(struct it950x_urb_context *context, Dword dwDataFrameSize)
{
	struct it950x_dev *dev = context->dev;
	Dword dwBuffLen = 0;
	//deb_data("enter %s", __func__);
	//deb_data("RMRingBuffer: (*dev->pWriteCurrBuffPointAddr) %d", (*dev->pWriteCurrBuffPointAddr));
	
	dwBuffLen = (dev->dwWriteTolBufferSize_cmd) - (*dev->pWriteCurrBuffPointAddr_cmd);
	
	if (dwBuffLen >= dwDataFrameSize) {
		(*dev->pWriteCurrBuffPointAddr_cmd) += dwDataFrameSize;
		if((*dev->pWriteCurrBuffPointAddr_cmd) >= (dev->dwWriteTolBufferSize_cmd)) (*dev->pWriteCurrBuffPointAddr_cmd) = 0;
	} else {
		(*dev->pWriteCurrBuffPointAddr_cmd) = 0;
		(*dev->pWriteCurrBuffPointAddr_cmd) += (dwDataFrameSize - dwBuffLen);
	}
	dev->dwRemaingBufferSize_cmd += dwDataFrameSize;
	//deb_data("RMRing URB complete index: %d", context->index);
	dev->urbstatus[context->index] = 0;
	atomic_add(1, &dev->urb_counter);
	return 0;
}

Dword FillRingBuffer(struct it950x_urb_context *context, Dword dwDataFrameSize)
{
	struct it950x_dev *dev = context->dev;
	Dword dwReadBuffAddr = 0;

	//deb_data("FillRingBufferDVBT %d\n", *dev->pCurrBuffPointAddr);
	//deb_data("Write - w %d, r %d\n", *dev->pCurrBuffPointAddr, (*dev->pReadBuffPointAddr));

	dwReadBuffAddr = (*dev->pReadBuffPointAddr);

	if ((*dev->pCurrBuffPointAddr) >= dwReadBuffAddr) {
		if (((*dev->pCurrBuffPointAddr)+dwDataFrameSize) < dev->dwTolBufferSize) 
			(*dev->pCurrBuffPointAddr) += dwDataFrameSize;
		else {
			if (dwReadBuffAddr == 0) mdelay(50); // Overflow
			else (*dev->pCurrBuffPointAddr) = 0;
		}
	} else {
		if (((*dev->pCurrBuffPointAddr)+dwDataFrameSize) < dwReadBuffAddr)
			(*dev->pCurrBuffPointAddr) += dwDataFrameSize;
		else mdelay(50);// Overflow
	}
	//dev->urbstatus_rx[context->index] = 0;
	//deb_data("Write - w %d, r %d end\n", *dev->pCurrBuffPointAddr, (*dev->pReadBuffPointAddr));

	return dwDataFrameSize;
}

static void
rx_free_urbs(struct it950x_dev *dev)
{
	int i;
	//deb_data("Enter %s Function\n",__FUNCTION__);
	
	for (i = 0; i < URB_COUNT_RX; i++) {
		usb_free_urb(dev->urbs_rx[i]);
		dev->urbs_rx[i] = NULL;
	}
	
	deb_data("%s() end\n", __func__);
}
static void
tx_free_urbs(struct it950x_dev *dev)
{
	int i;
	//deb_data("Enter %s Function\n",__FUNCTION__);
	
	for (i = 0; i < URB_COUNT_TX; i++) {
		usb_free_urb(dev->urbs[i]);
		dev->urbs[i] = NULL;
	}
	for(i = 0; i < URB_COUNT_TX_CMD; i++){
		usb_free_urb(dev->urbs_cmd[i]);
		dev->urbs_cmd[i] = NULL;
	}
	for(i = 0; i < URB_COUNT_TX_LOW_BRATE; i++){
		usb_free_urb(dev->urbs_low_brate[i]);
		dev->urbs_low_brate[i] = NULL;
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
		if(dev->urbstatus[i] == 1){
			usb_kill_urb(dev->urbs[i]);
			deb_data("kill tx urb index %d\n", i);
		}
	}
	
	deb_data("%s() end\n", __func__);
}
*/
static int stop_urb_transfer(struct it950x_dev *dev)
{
	int i;

	//deb_data("%s()\n", __func__);

	if (!dev->urb_streaming) {
		deb_data("%s: iso xfer already stop!\n", __func__);
		return 0;
	}

	/*DM368 usb bus error when using kill urb */
#if 0
	//mutex_lock(&it950x_urb_kill);	     /* if urb doesn't call back, kill it. */
	for (i = 0; i < URB_COUNT_RX; i++) {
		if(dev->urbstatus_rx[i] == 1){
			usb_kill_urb(dev->urbs_rx[i]);
			deb_data("kill rx urb index %d\n", i);
		}
	}
	//mutex_unlock(&it950x_urb_kill);		
#endif

	dev->urb_streaming = 0;
	deb_data("%s() end\n", __func__);

	return 0;
}

static int stop_urb_write_transfer(struct it950x_dev *dev)
{
	//deb_data("%s()\n", __func__);

	if (!dev->urb_tx_streaming && !dev->urb_tx_streaming_low_brate) {
		deb_data("%s: iso xfer already stop!\n", __func__);
		return 0;
	}
#if URB_TEST
	min_1 = 0;
#endif	
	dev->urb_tx_streaming = 0;
	dev->urb_tx_streaming_low_brate = 0;
	
	/*DM368 usb bus error when using kill urb */
#if 0
	for (i = 0; i < URB_COUNT_TX; i++) {
		if(dev->urbstatus[i] == 1) {
			dev->urbstatus[i] = 0;
			usb_kill_urb(dev->urbs[i]);
		}
		usb_free_urb(dev->urbs[i]);
		dev->urbs[i] = NULL;
	}

	mutex_lock(&it950x_urb_kill);
	tx_kill_busy_urbs(dev);
	mutex_unlock(&it950x_urb_kill);
#endif

	deb_data("%s() end\n", __func__);

	return 0;
}

static int stop_urb_write_transfer_cmd(struct it950x_dev *dev)
{
	deb_data("%s()\n", __func__);

	if (!dev->urb_tx_streaming_cmd) {
		deb_data("%s: iso xfer already stop!\n", __func__);
		return 0;
	}
	
	dev->urb_tx_streaming_cmd = 0;
	
	/*DM368 usb bus error when using kill urb */
#if 0
	for (i = 0; i < URB_COUNT_TX; i++) {
		if(dev->urbstatus[i] == 1) {
			dev->urbstatus[i] = 0;
			usb_kill_urb(dev->urbs[i]);
		}
		usb_free_urb(dev->urbs[i]);
		dev->urbs[i] = NULL;
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

/* AirHD */
DWORD WriteRingBuffer(
	struct it950x_dev *dev,
    Byte* pBuffer,
    Dword* pBufferLength)
{
    DWORD dwBuffLen = 0;
    DWORD dwCpBuffLen = *pBufferLength;
    DWORD dwCurrBuffAddr = (*dev->pWriteCurrBuffPointAddr);
    DWORD dwWriteBuffAddr = (*dev->pWriteBuffPointAddr);
    int ret = -ENOMEM;
	
//	deb_data("WriteRingBuffer-CPLen %d, dwCurrBuffAddr %d, dwWriteBuffAddr %d, dev->urb_back %d\n", dwCpBuffLen, dwCurrBuffAddr, dwWriteBuffAddr, dev->urb_back);
	
	int i =0;
	/*if(kk == 0){
		printk("remaining: %lu \t num = %d writeSize = %lu\n", dev->dwRemaingBufferSize, dev->urb_counter, dwCpBuffLen);
		for(i = 0 ; i < 16; i++){
			if(dev->urbstatus[i] == 0)
				printk("[%d]\t",i, dev->urbstatus[i]);
		}
		printk("\n");
	}

	kk = (kk + 1) % 10000;
	*/
	/*RingBuffer full*/
	if ((dev->dwRemaingBufferSize) == 0) {
		*pBufferLength = 0;
		udelay(100);
		printk("dwRemaingBufferSize = 0\n");
		return Error_NO_ERROR;
	}

    if ((dev->dwRemaingBufferSize) < dwCpBuffLen) {
		*pBufferLength = 0;
		udelay(100);
		printk("dwRemaingBufferSize < dwCpBuffLen\n");
		return Error_NO_ERROR;
	}

    if (*pBufferLength == 0) {
		udelay(100);
        return Error_BUFFER_INSUFFICIENT;
    }
    //-----------------------test-----------------------//
	/*
	 * dwWriteBuffAddr : To_kernel_urb addr (prepare writing)
	 * dwCurrBuffAddr : Return_urb addr (already writing)
	 * dwWriteTolBufferSize : total ringbuffer size
	 */
	
	//memory must enough because checking at first in this function
	if(dwWriteBuffAddr >= dwCurrBuffAddr){
		//To_kernel_urb not run a cycle or both run a cycle
		dwBuffLen = dev->dwWriteTolBufferSize - dwWriteBuffAddr; //remaining memory (to buffer end), not contain buffer beginning to return urb
		
		if(dwBuffLen >= dwCpBuffLen){
			//end remaining memory is enough
			memcpy(dev->pWriteFrameBuffer + dwWriteBuffAddr, pBuffer, dwCpBuffLen);
		}
		else{
			//use all end memory, run a cycle and need use beginning memory
			memcpy(dev->pWriteFrameBuffer + dwWriteBuffAddr, pBuffer, dwBuffLen); //using end memory
			memcpy(dev->pWriteFrameBuffer, pBuffer + dwBuffLen, dwCpBuffLen - dwBuffLen); //using begining memory
		}
	}
	else{
		//To_kernel_urb run a cycle and Return_urb not
		memcpy(dev->pWriteFrameBuffer + dwWriteBuffAddr, pBuffer, dwCpBuffLen);
	}
	
	(*dev->pWriteBuffPointAddr) = ((*dev->pWriteBuffPointAddr) + dwCpBuffLen) % (dev->dwWriteTolBufferSize);

	//mutex_lock(&it950x_rb_mutex);
	atomic_sub(dwCpBuffLen, (atomic_t*) &dev->dwRemaingBufferSize);
    //dev->dwRemaingBufferSize += dwCpBuffLen;
	//mutex_unlock(&it950x_rb_mutex);
		
	if(dev->urb_tx_streaming == 1){
		//allow submit urb
		while((dwWriteBuffAddr - (dev->urb_index * URB_BUFSIZE_TX)) >= URB_BUFSIZE_TX && atomic_read(&dev->urb_counter) > 0){
			//while urb full and not submit
			ret = usb_submit_urb(dev->urbs[dev->urb_index], GFP_ATOMIC);
			if (ret != 0) {
				stop_urb_write_transfer(dev);
				deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
				return ret;
			}
			
			dev->urbstatus[dev->urb_index] = 1;
			dev->urb_index = (dev->urb_index + 1) % URB_COUNT_TX;
			atomic_sub(1, &dev->urb_counter);
		}
	}
	//-----------------------test end-----------------------//

#if 0 // mark old
    if (dwCurrBuffAddr <= (*dev->pWriteBuffPointAddr)) {
		dwBuffLen = dev->dwWriteTolBufferSize - (*dev->pWriteBuffPointAddr);

		if (dwCpBuffLen <= dwBuffLen) { 
			/*It will NOT TOUCH end of ring buffer*/
			memcpy(dev->pWriteFrameBuffer+(*dev->pWriteBuffPointAddr), pBuffer, dwCpBuffLen);
			*pBufferLength = dwCpBuffLen;
			(*dev->pWriteBuffPointAddr) = dwWriteBuffAddr; // FIX memcy will modify pWriteBuffPointAddr value
			(*dev->pWriteBuffPointAddr) += dwCpBuffLen;
			//if(*dev->pWriteBuffPointAddr >= dev->dwWriteTolBufferSize) (*dev->pWriteBuffPointAddr) = 0;
			dev->dwRemaingBufferSize = dev->dwRemaingBufferSize - dwCpBuffLen;
		} else {
			/*It will TOUCH end of ring buffer*/
#if 0
			memcpy(dev->pWriteFrameBuffer+(*dev->pWriteBuffPointAddr), pBuffer, dwBuffLen);
			*pBufferLength = dwBuffLen;
			(*dev->pWriteBuffPointAddr) = 0;
			//dwWriteBuffAddr = 0;
			pBuffer += dwBuffLen;
			dwCpBuffLen -= dwBuffLen;
			dev->dwRemaingBufferSize -= dwBuffLen;
#endif
			if (dwBuffLen > 0) {
				memcpy(dev->pWriteFrameBuffer+(*dev->pWriteBuffPointAddr), pBuffer, dwBuffLen);
				*pBufferLength = dwBuffLen;
				(*dev->pWriteBuffPointAddr) = 0;
				pBuffer += dwBuffLen;
				dwCpBuffLen -= dwBuffLen;
				dev->dwRemaingBufferSize -= dwBuffLen;
			} else (*dev->pWriteBuffPointAddr) = 0;

			memcpy(dev->pWriteFrameBuffer+(*dev->pWriteBuffPointAddr), pBuffer, dwCpBuffLen);
			*pBufferLength += dwBuffLen;
			(*dev->pWriteBuffPointAddr) = 0;
			(*dev->pWriteBuffPointAddr) += dwCpBuffLen;
			dev->dwRemaingBufferSize = dev->dwRemaingBufferSize - dwCpBuffLen;
		}
    } else {
        dwBuffLen = dwCurrBuffAddr - (*dev->pWriteBuffPointAddr);

        if (dwCpBuffLen > dwBuffLen) {
            *pBufferLength = 0;
			udelay(100);
			return Error_NO_ERROR;
        } else {
            memcpy(dev->pWriteFrameBuffer+(*dev->pWriteBuffPointAddr), pBuffer, dwCpBuffLen);
			*pBufferLength = dwCpBuffLen;
			(*dev->pWriteBuffPointAddr) = dwWriteBuffAddr;
			(*dev->pWriteBuffPointAddr) += dwCpBuffLen;
			dev->dwRemaingBufferSize = dev->dwRemaingBufferSize - dwCpBuffLen;
        }
    }

    if (dev->urb_tx_streaming == 1 && (dwWriteBuffAddr - (dev->urb_index * URB_BUFSIZE_TX))>= URB_BUFSIZE_TX && (atomic_read(&dev->urb_counter)) > 0) {
		ret = usb_submit_urb(dev->urbs[dev->urb_index], GFP_ATOMIC);
		if (ret != 0) {
			stop_urb_write_transfer(dev);
			deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
			return ret;
		}
		//if (atomic_read(&dev->urb_counter) < 10) deb_data("\nless_than_10_can_urb_use_count = %d\n", atomic_read(&dev->urb_counter));
		else {
#if URB_TEST
			if ((loop_cnt % 35000) >= 30800) {
				do_gettimeofday(&now);
				diff_time_tx_write = (now.tv_sec - start.tv_sec)*1000000 + (now.tv_usec - start.tv_usec);
				loop_cnt = 0;
				if (diff_time_tx_write >= 60000000) { 
					min_1++;
					do_gettimeofday(&start);
				}	
				printk("\nTotal_Run_Time is %d (1 mins)\n", min_1);
				printk("\nurb_can_use_count = %d\n", atomic_read(&dev->urb_counter));
			}
#endif
		}
		
		dev->urbstatus[dev->urb_index] = 1;
		dev->urb_index++;
		atomic_sub(1, &dev->urb_counter);
		if (dev->urb_index == URB_COUNT_TX) dev->urb_index = 0;
	}
#endif // mark old end
    return Error_NO_ERROR;
}

/* AirHD low bitrate*/
DWORD WriteRingBuffer_low_brate(
	struct it950x_dev *dev,
    Byte* pBuffer,
    Dword* pBufferLength)
{
    DWORD dwBuffLen = 0;
    DWORD dwCpBuffLen = *pBufferLength;
    DWORD dwCurrBuffAddr = (*dev->pWriteCurrBuffPointAddr_low_brate);
    DWORD dwWriteBuffAddr = (*dev->pWriteBuffPointAddr_low_brate);
    int ret = -ENOMEM;
    int i;

//	deb_data("WriteRingBuffer-CPLen %d, dwCurrBuffAddr %d, dwWriteBuffAddr %d, dev->urb_back %d\n", dwCpBuffLen, dwCurrBuffAddr, dwWriteBuffAddr, dev->urb_back);
	if(dev->urb_tx_streaming_low_brate == 1 && (dwWriteBuffAddr - (dev->urb_index_low_brate * URB_BUFSIZE_TX_LOW_BRATE))>= URB_BUFSIZE_TX_LOW_BRATE && (atomic_read(&dev->urb_counter_low_brate)) > 0/*dev->urb_use_count > 0*/){
		ret = usb_submit_urb(dev->urbs_low_brate[dev->urb_index_low_brate], GFP_ATOMIC);
		if (ret != 0){
			stop_urb_write_transfer(dev);
			deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
			return ret;
		}//else deb_data("usb_submit_urb ok \n");
		
		//if (atomic_read(&dev->urb_counter_low_brate) < 5) deb_data("\nless_than_10_can_urb_use_count = %d\n", atomic_read(&dev->urb_counter_low_brate));
		dev->urbstatus_low_brate[dev->urb_index_low_brate] = 1;
		dev->urb_index_low_brate++;
		atomic_sub(1, &dev->urb_counter_low_brate);
		if(dev->urb_index_low_brate == URB_COUNT_TX_LOW_BRATE) dev->urb_index_low_brate = 0;
	}

	/*RingBuffer full*/
	if ((dev->dwRemaingBufferSize_low_brate) == 0){
		*pBufferLength = 0;
		udelay(100);
		return Error_NO_ERROR;
	}
	
    if ((dev->dwRemaingBufferSize_low_brate) < dwCpBuffLen){
		*pBufferLength = 0;
		udelay(100);
		return Error_NO_ERROR;
	}

    if (*pBufferLength == 0){
		udelay(100);
        return Error_BUFFER_INSUFFICIENT;
    }

    if (dwCurrBuffAddr <= (*dev->pWriteBuffPointAddr_low_brate)) {
		dwBuffLen = dev->dwWriteTolBufferSize_low_brate - (*dev->pWriteBuffPointAddr_low_brate);

		if(dwCpBuffLen <= dwBuffLen){ 
			/*It will NOT TOUCH end of ring buffer*/
			memcpy(dev->pWriteFrameBuffer_low_brate+(*dev->pWriteBuffPointAddr_low_brate), pBuffer, dwCpBuffLen);
			*pBufferLength = dwCpBuffLen;
			(*dev->pWriteBuffPointAddr_low_brate) = dwWriteBuffAddr; // FIX memcy will modify pWriteBuffPointAddr value
			(*dev->pWriteBuffPointAddr_low_brate) += dwCpBuffLen;
			dev->dwRemaingBufferSize_low_brate = dev->dwRemaingBufferSize_low_brate - dwCpBuffLen;
		}else{
			/*It will TOUCH end of ring buffer*/
			
#if 0
			memcpy(dev->pWriteFrameBuffer+(*dev->pWriteBuffPointAddr), pBuffer, dwBuffLen);
			*pBufferLength = dwBuffLen;
			(*dev->pWriteBuffPointAddr) = 0;
			//dwWriteBuffAddr = 0;
			pBuffer += dwBuffLen;
			dwCpBuffLen -= dwBuffLen;
			dev->dwRemaingBufferSize -= dwBuffLen;
#endif
			if(dwBuffLen > 0){
				memcpy(dev->pWriteFrameBuffer_low_brate+(*dev->pWriteBuffPointAddr_low_brate), pBuffer, dwBuffLen);
				*pBufferLength = dwBuffLen;
				(*dev->pWriteBuffPointAddr_low_brate) = 0;
				pBuffer += dwBuffLen;
				dwCpBuffLen -= dwBuffLen;
				dev->dwRemaingBufferSize_low_brate -= dwBuffLen;
			}else{
				
				(*dev->pWriteBuffPointAddr_low_brate) = 0;
			}

			memcpy(dev->pWriteFrameBuffer_low_brate+(*dev->pWriteBuffPointAddr_low_brate), pBuffer, dwCpBuffLen);
			*pBufferLength += dwBuffLen;
			(*dev->pWriteBuffPointAddr_low_brate) = 0;
			(*dev->pWriteBuffPointAddr_low_brate) += dwCpBuffLen;

			dev->dwRemaingBufferSize_low_brate = dev->dwRemaingBufferSize_low_brate - dwCpBuffLen;
		}
    }else {
        dwBuffLen = dwCurrBuffAddr - (*dev->pWriteBuffPointAddr_low_brate);

        if (dwCpBuffLen > dwBuffLen) {
            *pBufferLength = 0;
			udelay(100);
			return Error_NO_ERROR;
        }
        else {
            memcpy(dev->pWriteFrameBuffer_low_brate+(*dev->pWriteBuffPointAddr_low_brate), pBuffer, dwCpBuffLen);
			*pBufferLength = dwCpBuffLen;
			(*dev->pWriteBuffPointAddr_low_brate) = dwWriteBuffAddr;
			(*dev->pWriteBuffPointAddr_low_brate) += dwCpBuffLen;
			
			dev->dwRemaingBufferSize_low_brate = dev->dwRemaingBufferSize_low_brate - dwCpBuffLen;
        }
    }
    return Error_NO_ERROR;
}

/* AirHD_CMD */
DWORD WriteRingBuffer_cmd(
	struct it950x_dev *dev,
    Byte* pBuffer,
    Dword* pBufferLength)
{
    DWORD dwCpBuffLen = *pBufferLength;
    //DWORD dwCurrBuffAddr = (*dev->pWriteCurrBuffPointAddr_cmd);
    //DWORD dwWriteBuffAddr = (*dev->pWriteBuffPointAddr_cmd);
    int ret = -ENOMEM;
    //int i;
         
    memcpy(dev->pWriteFrameBuffer_cmd, pBuffer, dwCpBuffLen);
    
		mutex_lock(&it950x_urb_kill);
		ret = usb_submit_urb(dev->urbs_cmd[dev->urb_index_cmd], GFP_ATOMIC);
		mutex_unlock(&it950x_urb_kill);
		if (ret != 0){
			stop_urb_write_transfer_cmd(dev);
			deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
			return ret;
		}else deb_data("usb_submit_urb cmd ok \n");
		//if(dev->urb_use_count_cmd < 1) 
			//deb_data("urb index is %d-------------------send, urb_use_count = %d\n", dev->urb_index_cmd, dev->urb_use_count_cmd);
		dev->urbstatus_cmd[dev->urb_index_cmd] = 1;

		if(dev->urb_index_cmd == URB_COUNT_TX_CMD) dev->urb_index_cmd = 0;

    return Error_NO_ERROR;
}

DWORD ReadRingBuffer(
	struct it950x_dev *dev,
    Byte* pBuffer,
    Dword* pBufferLength)
{

	//deb_data("ReadRingBuffer function\n");
    DWORD dwBuffLen = 0;
    DWORD dwCpBuffLen = *pBufferLength;
    DWORD dwCurrBuffAddr = (*dev->pCurrBuffPointAddr);

    if ((*dev->pReadBuffPointAddr) == dwCurrBuffAddr) {
		*pBufferLength = 0;
		return Error_NO_ERROR;
	}			
	if ((dwBuffLen + dwCurrBuffAddr) < dwCpBuffLen) {
		*pBufferLength = 0;
		return Error_NO_ERROR;
	}

    if (*pBufferLength == 0) return Error_BUFFER_INSUFFICIENT;

	//deb_data("Read - c %d, r %d\n", dwCurrBuffAddr, (*dev->pReadBuffPointAddr));

    //wsprintf(StrTmp, L"Read - w %d, r %d\r\n", dwCurrBuffAddr, (*dev->pReadBuffPointAddr));
    //SDIOLogMsg(1, StrTmp);

    if (dwCurrBuffAddr > (*dev->pReadBuffPointAddr)) {
        dwBuffLen = dwCurrBuffAddr - (*dev->pReadBuffPointAddr);
	
		if(dwBuffLen  < dwCpBuffLen){
			*pBufferLength = 0;
			return Error_NO_ERROR;
		}
		
        if (dwCpBuffLen > dwBuffLen) {
            memcpy(pBuffer, dev->pFrameBuffer+(*dev->pReadBuffPointAddr), dwBuffLen);
            *pBufferLength = dwBuffLen;
		    (*dev->pReadBuffPointAddr) = dwCurrBuffAddr;
		    deb_data("\ndwBuffLen = %lu \n", dwBuffLen);
        } else {
            memcpy(pBuffer, dev->pFrameBuffer+(*dev->pReadBuffPointAddr), dwCpBuffLen);
            *pBufferLength = dwCpBuffLen;
		    (*dev->pReadBuffPointAddr) += dwCpBuffLen;
        }
        
    } else {
        dwBuffLen = dev->dwTolBufferSize - (*dev->pReadBuffPointAddr);

        if (dwCpBuffLen > dwBuffLen) {
			
			if((dwBuffLen+dwCurrBuffAddr) < dwCpBuffLen){
				*pBufferLength = 0;
				return Error_NO_ERROR;
			}
			
            memcpy(pBuffer, dev->pFrameBuffer+(*dev->pReadBuffPointAddr), dwBuffLen);
            *pBufferLength = dwBuffLen;
		    (*dev->pReadBuffPointAddr) = 0;

		    pBuffer += dwBuffLen;
		    dwCpBuffLen -= dwBuffLen;
		    dwBuffLen = dwCurrBuffAddr - (*dev->pReadBuffPointAddr);

		    if (dwBuffLen > 0) {
		        if (dwCpBuffLen > dwBuffLen) {
                    memcpy(pBuffer, dev->pFrameBuffer+(*dev->pReadBuffPointAddr), dwBuffLen);
                    *pBufferLength += dwBuffLen;
		            (*dev->pReadBuffPointAddr) = dwCurrBuffAddr;
                } else {
                    memcpy(pBuffer, dev->pFrameBuffer+(*dev->pReadBuffPointAddr), dwCpBuffLen);
                    *pBufferLength += dwCpBuffLen;
		            (*dev->pReadBuffPointAddr) += dwCpBuffLen;
                }
            }
        } else {
            memcpy(pBuffer, dev->pFrameBuffer+(*dev->pReadBuffPointAddr), dwCpBuffLen);
            *pBufferLength = dwCpBuffLen;
		    (*dev->pReadBuffPointAddr) += dwCpBuffLen;
        }
    }

    return Error_NO_ERROR;
}

/******************************************************************/

static void write_urb_completion(struct urb *purb)
{
	struct it950x_urb_context *context = purb->context;
	int ptype = usb_pipetype(purb->pipe);
	
	//deb_data("enter %s", __func__);

	//deb_data("'%s' urb completed. status: %d, length: %d/%d, pack_num: %d, errors: %d\n",
	//	ptype == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
	//	purb->status,purb->actual_length,purb->transfer_buffer_length,
	//	purb->number_of_packets,purb->error_count);
	//context->dev->urbstatus[context->index] = 0;
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
	
	if (context->dev->urb_tx_streaming == 0) return;

	if (ptype != PIPE_BULK) {
		deb_data("TX %s() Unsupported URB type %d\n", __func__, ptype);
		return;
	}
	
	//ptr = (u8 *)purb->transfer_buffer;
	/* Feed the transport payload into the kernel demux */
	//dvb_dmx_swfilter_packets(&dev->dvb.demux,
	//	purb->transfer_buffer, purb->actual_length / 188);
	//if (purb->actual_length > 0)
	
	RMRingBuffer(context, URB_BUFSIZE_TX);
	
	return;
}

static void write_urb_completion_low_brate(struct urb *purb)
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
	//context->dev->urbstatus[context->index] = 0;
	switch (purb->status) {
		case 0:         /* success */
		case -ETIMEDOUT:    /* NAK */
			break;
		case -ECONNRESET:   /* kill */
		case -ENOENT:
			context->dev->urb_use_count_low_brate++;
			deb_data("TX ENOENT-write_urb_completion_low_brate error %d.\n", purb->status);
		case -ESHUTDOWN:
			return;
		default:        /* error */
			deb_data("write_urb_completion_low_brate error %d.\n", purb->status);
			break;
	}

	if (!context->dev)
		return;
	
	if (context->dev->urb_tx_streaming_low_brate == 0)
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
	
	RMRingBuffer_low_brate(context, URB_BUFSIZE_TX_LOW_BRATE);
	return;
}

static void write_urb_completion_cmd(struct urb *purb)
{
	struct it950x_urb_context *context = purb->context;
	int ptype = usb_pipetype(purb->pipe);
	deb_data("enter %s", __func__);

	//deb_data("'%s' urb completed. status: %d, length: %d/%d, pack_num: %d, errors: %d\n",
	//	ptype == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
	//	purb->status,purb->actual_length,purb->transfer_buffer_length,
	//	purb->number_of_packets,purb->error_count);
	
	context->dev->urbstatus_cmd[context->index] = 0;
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
	
	if (context->dev->urb_tx_streaming_cmd == 0) return;

	if (ptype != PIPE_BULK) {
		deb_data("TX %s() Unsupported URB type %d\n", __func__, ptype);
		return;
	}
	
	//ptr = (u8 *)purb->transfer_buffer;
	/* Feed the transport payload into the kernel demux */
	//dvb_dmx_swfilter_packets(&dev->dvb.demux,
	//	purb->transfer_buffer, purb->actual_length / 188);
	//if (purb->actual_length > 0)
	
	RMRingBuffer_cmd(context, URB_BUFSIZE_TX_CMD);
	
	return;
}

static void urb_completion(struct urb *purb)
{
	struct it950x_urb_context *context = purb->context;
	int ptype = usb_pipetype(purb->pipe);
	int ret = -ENOMEM;

	//deb_data("%s", __func__);
	//deb_data("'%s' urb completed. status: %d, length: %d/%d, pack_num: %d, errors: %d\n",
	//	ptype == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
	//	purb->status,purb->actual_length,purb->transfer_buffer_length,
	//	purb->number_of_packets,purb->error_count);
	//deb_data("urb_complete(%d)\n", context->index);
		
	context->dev->urbstatus_rx[context->index] = 0;
	switch (purb->status) {
		case 0:              /* Success */
			break;
		case -ETIMEDOUT:    /* NAK */
			deb_data("RX ETIMEDOUT -urb completition error %d.\n", purb->status);
			break;
		case -ECONNRESET:   /* unlink */
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

	if (!context->dev) return;

	if (context->dev->urb_streaming == 0) return;

	if (ptype != PIPE_BULK) {
		deb_data("RX %s() Unsupported URB type %d\n", __func__, ptype);
		return;
	}

	//ptr = (u8 *)purb->transfer_buffer;
	/* Feed the transport payload into the kernel demux */
	//dvb_dmx_swfilter_packets(&dev->dvb.demux,
	//	purb->transfer_buffer, purb->actual_length / 188);
	
	if (context->dev->urb_streaming == 1){
		if (purb->actual_length > 0)
			FillRingBuffer(context, URB_BUFSIZE_RX);

		/* Clean the buffer before we requeue */
		//memset(purb->transfer_buffer, 0, URB_BUFSIZE_RX);

		/* Requeue URB */
		ret = usb_submit_urb(purb, GFP_ATOMIC);
		if (ret != 0) {
			stop_urb_transfer(context->dev);
			deb_data("RX %s: failed urb submission, err = %d\n", __func__, ret);
		}
		context->dev->urbstatus_rx[context->index] = 1;
	}
}

/**      AirHD       **********************************************/
static int start_urb_write_transfer(struct it950x_dev *dev)
{
	struct urb *purb;
	int i, ret = -ENOMEM;

	deb_data("%s()\n", __func__);
#if URB_TEST
	min_1 = 0;
#endif	
	if (dev->urb_tx_streaming || dev->urb_tx_streaming_low_brate) {
		deb_data("%s: iso xfer already running!\n", __func__);
		return 0;
	}
	
	*dev->pWriteCurrBuffPointAddr = 0;
	*dev->pWriteBuffPointAddr = 0;

	*dev->pWriteCurrBuffPointAddr_low_brate = 0;
	*dev->pWriteBuffPointAddr_low_brate = 0;

	for (i = 0; i < URB_COUNT_TX; i++) {

		//dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		//if (!dev->urbs[i])
		//	goto err;

		purb = dev->urbs[i];

		purb->transfer_buffer = dev->pWriteFrameBuffer + (URB_BUFSIZE_TX*i);
		if (!purb->transfer_buffer) {
			usb_free_urb(purb);
			dev->urbs[i] = NULL;
			goto err;
		}
		
		dev->urb_context[i].index = i;
		dev->urb_context[i].dev = dev;
		dev->urbstatus[i] = 0;
		
		purb->status = -EINPROGRESS;
		usb_fill_bulk_urb(purb,
				  dev->usbdev,
				  usb_sndbulkpipe(dev->usbdev, 0x06),
				  purb->transfer_buffer,
				  URB_BUFSIZE_TX,
				  write_urb_completion,
				   &dev->urb_context[i]);
		
		purb->transfer_flags = 0;
	}
	for (i = 0; i < URB_COUNT_TX_LOW_BRATE; i++) {

		//dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		//if (!dev->urbs[i])
		//	goto err;

		purb = dev->urbs_low_brate[i];

		purb->transfer_buffer = dev->pWriteFrameBuffer_low_brate + (URB_BUFSIZE_TX_LOW_BRATE * i);
		if (!purb->transfer_buffer) {
			usb_free_urb(purb);
			dev->urbs_low_brate[i] = NULL;
			goto err;
		}
		
		dev->urb_context_low_brate[i].index = i;
		dev->urb_context_low_brate[i].dev = dev;
		dev->urbstatus_low_brate[i] = 0;
		
		purb->status = -EINPROGRESS;
		usb_fill_bulk_urb(purb,
				  dev->usbdev,
				  usb_sndbulkpipe(dev->usbdev, 0x06),
				  purb->transfer_buffer,
				  URB_BUFSIZE_TX_LOW_BRATE,
				  write_urb_completion_low_brate,
				  &dev->urb_context_low_brate[i]);
		
		purb->transfer_flags = 0;
	}	
	dev->dwRemaingBufferSize = dev->dwWriteTolBufferSize;
	dev->urb_index = 0;
	dev->urb_tx_streaming = 1;
	atomic_set(&dev->urb_counter, URB_COUNT_TX);

	dev->dwRemaingBufferSize_low_brate = dev->dwWriteTolBufferSize_low_brate;
	dev->urb_index_low_brate = 0;
	dev->urb_use_count_low_brate = URB_COUNT_TX_LOW_BRATE;
	dev->urb_tx_streaming_low_brate = 1;

	ret = 0;

	deb_data("%s() end\n", __func__);

err:
	return ret;
}

/*********************************************************************/

/**      AirHD_CMD       **********************************************/
static int start_urb_write_transfer_cmd(struct it950x_dev *dev)
{
	struct urb *purb;
	int i, ret = -ENOMEM;

	deb_data("%s()\n", __func__);

	if (dev->urb_tx_streaming_cmd) {
		deb_data("%s: iso xfer already running!\n", __func__);
	//	return 0;
	}
	
	*dev->pWriteCurrBuffPointAddr_cmd = 0;
	*dev->pWriteBuffPointAddr_cmd = 0;

	for (i = 0; i < URB_COUNT_TX_CMD; i++) {

		//dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		//if (!dev->urbs[i])
		//	goto err;

		purb = dev->urbs_cmd[i];

		purb->transfer_buffer = dev->pWriteFrameBuffer_cmd + (URB_BUFSIZE_TX_CMD*i);
		if (!purb->transfer_buffer) {
			usb_free_urb(purb);
			dev->urbs_cmd[i] = NULL;
			goto err;
		}
		
		dev->urb_context_cmd[i].index = i;
		dev->urb_context_cmd[i].dev = dev;
		dev->urbstatus_cmd[i] = 0;
		
		purb->status = -EINPROGRESS;
		usb_fill_bulk_urb(purb,
				  dev->usbdev,
				  usb_sndbulkpipe(dev->usbdev, 0x06),
				  purb->transfer_buffer,
				  URB_BUFSIZE_TX_CMD,
				  write_urb_completion_cmd,
				   &dev->urb_context_cmd[i]);
		
		purb->transfer_flags = 0;
	}
	dev->dwRemaingBufferSize_cmd = dev->dwWriteTolBufferSize_cmd;
	dev->urb_index_cmd = 0;
	dev->urb_tx_streaming_cmd = 1;
	ret = 0;

	deb_data("%s() end\n", __func__);

err:
	return ret;
}

/*********************************************************************/
static int start_urb_transfer(struct it950x_dev *dev)
{
	struct urb *purb;
	int i, nBytesRead = 0, ret = 0;
	unsigned char buffer[CLEAN_HARDWARE_BUFFER_SIZE];

	deb_data("%s()\n", __func__);

	if (dev->urb_streaming) {
		deb_data("%s: iso xfer already running!\n", __func__);
		return 0;
	}
	
	*dev->pCurrBuffPointAddr = 0;
	*dev->pReadBuffPointAddr = 0;
	
	/*Clean Hardware Buffer*/
/*
	while(1){
	    if(ret == -110)
	    break;
		ret = usb_bulk_msg(dev->usbdev,
				usb_rcvbulkpipe(dev->usbdev, 0x85), //dev->Demodulator.driver
				buffer,
				CLEAN_HARDWARE_BUFFER_SIZE,
				&nBytesRead,
				1000);	
	}
*/

	
		//memset(purb->transfer_buffer, 0, URB_BUFSIZE_RX);
	for (i = 0; i < URB_COUNT_RX; i++) {

		//dev->urbs_rx[i] = usb_alloc_urb(0, GFP_KERNEL);
		//if (!dev->urbs_rx[i])
		//	goto err;

		purb = dev->urbs_rx[i];
		
		purb->transfer_buffer = dev->pFrameBuffer + (URB_BUFSIZE_RX*i);
		//memset(purb->transfer_buffer, 0, URB_BUFSIZE_RX*i);
		
		if (!purb->transfer_buffer) {
			usb_free_urb(purb);
			dev->urbs_rx[i] = NULL;
			goto err;
		}

		dev->urb_context_rx[i].index = i;
		dev->urb_context_rx[i].dev = dev;
		dev->urbstatus_rx[i] = 0;

		purb->status = -EINPROGRESS;
		usb_fill_bulk_urb(purb,
				  dev->usbdev,
				  usb_rcvbulkpipe(dev->usbdev, 0x85),
				  purb->transfer_buffer,
				  URB_BUFSIZE_RX,
				  urb_completion,
				  &dev->urb_context_rx[i]);
		
		purb->transfer_flags = 0;
	}
		
	for (i = 0; i < URB_COUNT_RX; i++) {
		ret = usb_submit_urb(dev->urbs_rx[i], GFP_ATOMIC);
		if (ret != 0) {
			stop_urb_transfer(dev);
			deb_data("%s: failed urb submission, err = %d\n", __func__, ret);
			return ret;
		}
		dev->urbstatus_rx[i] = 1;
	}
	
	dev->urb_streaming = 1;
	ret = 0;

	deb_data("%s() end\n", __func__);

err:
	return ret;
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
int it950x_usb_rx_alloc_dev(struct it950x_dev* dev)
{
	int retval = 0;
	int error, order, i;

	if (!dev) {
		deb_data("it950x_dev is null\n");
		return -ENODEV;
	}

	/*Allocate TX RX urb*/	
	for (i = 0; i < URB_COUNT_RX; i++) {
		dev->urbs_rx[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->urbs_rx[i])
			retval = -ENOMEM;
	}	
	
	/*Read Ring buffer alloc*/
	dev->dwTolBufferSize = URB_BUFSIZE_RX * URB_COUNT_RX;
	order = get_order(dev->dwTolBufferSize + 8);
	dev->pRingBuffer = (Byte*)__get_free_pages(GFP_KERNEL, order);
	//dev->pRingBuffer = kzalloc(dev->dwTolBufferSize + 8, GFP_KERNEL);
	if (dev->pRingBuffer) {
		dev->pFrameBuffer = dev->pRingBuffer + 16;
		dev->pCurrBuffPointAddr = (Dword*)dev->pRingBuffer;
		dev->pReadBuffPointAddr = (Dword*)(dev->pRingBuffer + 4);
	}
	
	/* increment our usage count for the device */
	//kref_get(&dev->kref);
	
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
	
	return retval;
}

static int it950x_usb_open(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev = NULL;
	int subminor, mainsubminor;
	struct usb_interface *interface;
	int retval = 0;

	mainsubminor = iminor(inode);
	subminor = iminor(inode) + USB_it913x_MINOR_RANGE;

	deb_data("it950x_usb_rx_open function\n");
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
        if (retval)
            return retval;

	/* save our object in the file's private structure */
	dev->file = file;
	file->private_data = dev;

exit:
	return retval;
}

int it950x_usb_tx_alloc_dev(struct it950x_dev *dev)
{
	int retval = 0;
	int error;
	int order, order_cmd;
	
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}	
	
	atomic_set(&dev->urb_counter, URB_COUNT_TX);
	atomic_set(&dev->urb_counter_low_brate, URB_COUNT_TX_LOW_BRATE);

#if URB_TEST		
	do_gettimeofday(&start);	
#endif

	/*kzalloc will limit by Embedded system*/
#if 0
	dev->dwWriteTolBufferSize = URB_BUFSIZE_TX * URB_COUNT_TX;
	dev->pWriteRingBuffer = kzalloc(dev->dwWriteTolBufferSize + 8, GFP_KERNEL);
	if (dev->pWriteRingBuffer) {
		dev->pWriteFrameBuffer = dev->pWriteRingBuffer + 8;
		dev->pWriteCurrBuffPointAddr = dev->pWriteRingBuffer;
		dev->pWriteBuffPointAddr = dev->pWriteRingBuffer + 4;
		dev->dwRemaingBufferSize = dev->dwWriteTolBufferSize;
		dev->urb_index = 0;
		//dev->urb_use_count = URB_COUNT_TX;
	}
#endif

	/*Write Ring buffer alloc*/
	dev->dwWriteTolBufferSize = URB_BUFSIZE_TX * URB_COUNT_TX;
	order = get_order(dev->dwWriteTolBufferSize + 8);
	dev->pWriteRingBuffer = (Byte*)__get_free_pages(GFP_KERNEL, order);
	if (dev->pWriteRingBuffer) {
		dev->pWriteFrameBuffer = dev->pWriteRingBuffer + 8;
		dev->pWriteCurrBuffPointAddr = (Dword*)dev->pWriteRingBuffer;
		dev->pWriteBuffPointAddr = (Dword*)(dev->pWriteRingBuffer + 4);
		dev->dwRemaingBufferSize = dev->dwWriteTolBufferSize;
		dev->urb_index = 0;
		//dev->urb_use_count = URB_COUNT_TX;
	}
	
	/*Write cmd Ring buffer alloc*/
	dev->dwWriteTolBufferSize_cmd = URB_BUFSIZE_TX_CMD * URB_COUNT_TX_CMD;
	order_cmd = get_order(dev->dwWriteTolBufferSize_cmd + 8);
	dev->pWriteRingBuffer_cmd = (Byte*)__get_free_pages(GFP_KERNEL, order_cmd);
	if (dev->pWriteRingBuffer_cmd) {
		dev->pWriteFrameBuffer_cmd = dev->pWriteRingBuffer_cmd + 8;
		dev->pWriteCurrBuffPointAddr_cmd = (Dword*)dev->pWriteRingBuffer_cmd;
		dev->pWriteBuffPointAddr_cmd = (Dword*)(dev->pWriteRingBuffer_cmd + 4);
		dev->dwRemaingBufferSize_cmd = dev->dwWriteTolBufferSize_cmd;
		dev->urb_index_cmd = 0;
		//dev->urb_use_count_cmd = URB_COUNT_TX_CMD;
	}
	
	/*Write low bitrate Ring buffer alloc*/
	dev->dwWriteTolBufferSize_low_brate = URB_BUFSIZE_TX_LOW_BRATE * URB_COUNT_TX_LOW_BRATE;
	order = get_order(dev->dwWriteTolBufferSize_low_brate + 8);
	dev->pWriteRingBuffer_low_brate = (Byte*)__get_free_pages(GFP_KERNEL, order);
	if (dev->pWriteRingBuffer_low_brate) {
		dev->pWriteFrameBuffer_low_brate = dev->pWriteRingBuffer_low_brate + 8;
		dev->pWriteCurrBuffPointAddr_low_brate = (Dword*)dev->pWriteRingBuffer_low_brate;
		dev->pWriteBuffPointAddr_low_brate = (Dword*)(dev->pWriteRingBuffer_low_brate + 4);
		dev->dwRemaingBufferSize_low_brate = dev->dwWriteTolBufferSize_low_brate;
		dev->urb_index_low_brate = 0;
		dev->urb_use_count_low_brate = URB_COUNT_TX_LOW_BRATE;
	}
		
	/* increment our usage count for the device */
	//kref_get(&dev->kref);

#if !(defined(DTVCAM_POWER_CTRL) && defined(AVSENDER_POWER_CTRL))
	if(atomic_read(&dev->tx_pw_on) == 0) {
		error = DL_ApPwCtrl(&dev->DC, 0, 1);
		atomic_set(&dev->tx_pw_on, 1);	
	}
#endif		

	atomic_add(1, &dev->g_AP_use_tx);

	return retval;
}

static int it950x_usb_tx_open(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev = NULL;
	int mainsubminor, subminor;
	struct usb_interface *interface;
	int retval = 0;

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
	if (retval)
	    return retval;

	/* save our object in the file's private structure */
	dev->tx_file = file;
	file->private_data = dev;

exit:
        return retval;
}

int it950x_usb_rx_free_dev(struct it950x_dev* dev)
{
	int error, i;
	int order;
	//deb_data("it950x_usb_release function\n");
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}
	
	if(atomic_read(&dev->g_AP_use_rx) == 1) {
		stop_urb_transfer(dev);

/*		if(dev->g_AP_use == 1) {
			dev->DevicePower = 0;
			error = DL_ApCtrl(&dev->DC, 0, 0);
			error = DL_ApCtrl(&dev->DC, 1, 0);
		}
		dev->g_AP_use--;*/
		
		//dev = (struct it950x_dev *)file->private_data;
		//if (dev == NULL)
		//	return -ENODEV;

		/* decrement the count on our device */
		//kref_put(&dev->kref, afa_delete);

		if (dev->pRingBuffer){
			//kfree(dev->pRingBuffer);
			order = get_order(dev->dwTolBufferSize + 8);
			free_pages((long unsigned int)dev->pRingBuffer, order);
		}	

		for (i = 0; i < URB_COUNT_RX; i++) {   /* if urb doesn't call back, kill it. */
			if(dev->urbstatus_rx[i] == 1){
				usb_kill_urb(dev->urbs_rx[i]);
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

static int it950x_usb_release(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev = NULL;
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_rx_free_dev(dev);
}

int it950x_usb_tx_free_dev(struct it950x_dev* dev)
{
	int error;
	int order;
	//deb_data("it950x_usb_release function\n");	
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}	

	if(	atomic_read(&dev->g_AP_use_tx) == 1) {
		
		//dev = (struct it950x_dev *)file->private_data;
		//if (dev == NULL)
		//	return -ENODEV;

		stop_urb_write_transfer(dev);

		/* decrement the count on our device */
		//kref_put(&dev->kref, afa_delete);
		
		if (dev->pWriteRingBuffer){
			order = get_order(dev->dwWriteTolBufferSize + 8);
			free_pages((long unsigned int)dev->pWriteRingBuffer, order);
		}

		if (dev->pWriteRingBuffer_cmd){
			order = get_order(dev->dwWriteTolBufferSize_cmd + 8);
			free_pages((long unsigned int)dev->pWriteRingBuffer_cmd, order);
		}

		if (dev->pWriteRingBuffer_low_brate){
			order = get_order(dev->dwWriteTolBufferSize_low_brate + 8);
			free_pages((long unsigned int)dev->pWriteRingBuffer_low_brate, order);
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

static int it950x_usb_tx_release(struct inode *inode, struct file *file)
{
	struct it950x_dev *dev;
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_tx_free_dev(dev);
}

int it950x_usb_rx_ioctl_dev(struct it950x_dev* dev,
	unsigned int cmd,  unsigned long parg)
{
	Byte temp0 = 0, temp1 = 1;
	Dword status;
  
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/

	//if (_IOC_TYPE(cmd) != AFA_IOC_MAGIC) return -ENOTTY;
	//if (_IOC_NR(cmd) > AFA_IOC_MAXNR) return -ENOTTY;

	switch (cmd)
	{	
		case IOCTL_ITE_DEMOD_STARTCAPTURE:
			start_urb_transfer(dev);
			return 0;		
		
		case IOCTL_ITE_DEMOD_STOPCAPTURE:
			stop_urb_transfer(dev);
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

static int it950x_usb_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd,  unsigned long parg)
{
	struct it950x_dev *dev;
	deb_data("it9507_usb_ioctl function\n");
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_rx_ioctl_dev(dev, cmd, parg);
}

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

int it950x_usb_tx_ioctl_dev(struct it950x_dev* dev,
	unsigned int cmd,  unsigned long parg)
{
	PCmdRequest pRequest;
	
	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/

	//if (_IOC_TYPE(cmd) != AFA_IOC_MAGIC) return -ENOTTY;
	//if (_IOC_NR(cmd) > AFA_IOC_MAXNR) return -ENOTTY;
    
	switch (cmd)
	{
		case IOCTL_ITE_DEMOD_STARTTRANSFER_TX:
			start_urb_write_transfer(dev);
			return 0;
					
		case IOCTL_ITE_DEMOD_STOPTRANSFER_TX:
			stop_urb_write_transfer(dev);
			return 0;
			
		case IOCTL_ITE_DEMOD_STARTTRANSFER_CMD_TX:
			start_urb_write_transfer_cmd(dev);
			return 0;
			
		case IOCTL_ITE_DEMOD_STOPTRANSFER_CMD_TX:
			stop_urb_write_transfer_cmd(dev);
			return 0;
		
		case IOCTL_ITE_DEMOD_WRITECMD_TX:
			pRequest = (PCmdRequest) parg;
			WriteRingBuffer_cmd(dev, pRequest->cmd, pRequest->len);
			return 0;		
			
		case IOCTL_ITE_DEMOD_SETLOWBRATETRANS_TX:
			SetLowBitRateTransfer(dev, (void*)parg);
			return 0;
	}
	return DL_DemodIOCTLFun((void *)&dev->DC.modulator, (DWORD)cmd, parg);
}

static int it950x_usb_tx_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd,  unsigned long parg)
{
	struct it950x_dev *dev;

	deb_data("it950x_usb_tx_ioctl function\n");
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_tx_ioctl_dev(dev, cmd, parg);
}

int it950x_usb_rx_unlocked_ioctl_dev(
	struct it950x_dev *dev,
	unsigned int cmd,
	unsigned long parg)
{
	//deb_data("it950x_usb_ioctl function\n");

	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/

	//if (_IOC_TYPE(cmd) != AFA_IOC_MAGIC) return -ENOTTY;
	//if (_IOC_NR(cmd) > AFA_IOC_MAXNR) return -ENOTTY;

	switch (cmd)
	{	
		case IOCTL_ITE_DEMOD_STARTCAPTURE:
			start_urb_transfer(dev);
			return 0;

		case IOCTL_ITE_DEMOD_STOPCAPTURE:
			stop_urb_transfer(dev);
			return 0;
	}

	return DL_DemodIOCTLFun((void *)&dev->DC.demodulator, (DWORD)cmd, parg);
}

int it950x_usb_unlocked_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long parg)
{
        struct it950x_dev *dev = NULL;
	dev = (struct it950x_dev *)file->private_data;
	
	return it950x_usb_rx_unlocked_ioctl_dev(dev, cmd, parg);
}

int it950x_usb_tx_unlocked_ioctl_dev(
	struct it950x_dev *dev,
	unsigned int cmd,
	unsigned long parg)
{
	PCmdRequest pRequest;	
	deb_data("it950x_usb_ioctl function %u\n", cmd);

	if (dev == NULL) {
		deb_data("dev is NULL\n");
		return -ENODEV;
	}

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/

	//if (_IOC_TYPE(cmd) != AFA_IOC_MAGIC) return -ENOTTY;
	//if (_IOC_NR(cmd) > AFA_IOC_MAXNR) return -ENOTTY;

	switch (cmd)
	{
		case IOCTL_ITE_DEMOD_STARTTRANSFER_TX:
			start_urb_write_transfer(dev);
			return 0;
					
		case IOCTL_ITE_DEMOD_STOPTRANSFER_TX:
			stop_urb_write_transfer(dev);
			return 0;
			
		case IOCTL_ITE_DEMOD_STARTTRANSFER_CMD_TX:
			start_urb_write_transfer_cmd(dev);
			return 0;
			
		case IOCTL_ITE_DEMOD_STOPTRANSFER_CMD_TX:
			stop_urb_write_transfer_cmd(dev);
			return 0;
		
		case IOCTL_ITE_DEMOD_WRITECMD_TX:
			pRequest = (PCmdRequest) parg;
			WriteRingBuffer_cmd(dev, pRequest->cmd, pRequest->len);
			return 0;		
			
		case IOCTL_ITE_DEMOD_SETLOWBRATETRANS_TX:
			SetLowBitRateTransfer(dev, (void*)parg);
			return 0;
	}
	return DL_DemodIOCTLFun((void *)&dev->DC.modulator, (DWORD)cmd, parg);
}


int it950x_usb_tx_unlocked_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long parg)
{
        struct it950x_dev* dev = NULL;
	dev = (struct it950x_dev *)file->private_data;

        return  it950x_usb_tx_unlocked_ioctl_dev(dev, cmd, parg);
}

ssize_t it950x_usb_rx_read_dev(
	struct it950x_dev *dev0,
	char __user *buf,
	size_t count)
{
	Dword Len = count;
	if (dev0 == NULL)
		return -ENODEV;

	ReadRingBuffer(dev0, buf, &Len);

	//deb_data("ReadRingBuffer - %d\n", Len);

	return Len;

	/*AirHD*/
#if 0
	//udev = (struct usb_device *)file->private_data;
	//dev = (DEVICE_CONTEXT *)dev_get_drvdata(&udev->dev);

		ret = usb_bulk_msg(usb_get_dev(dev0->DC->modulator.userData),
				usb_rcvbulkpipe(usb_get_dev(dev0->DC->modulator.userData), 0x85), //dev->Demodulator.driver
				buffer,
				256,
				&nBytesRead,
				100000);	

		if (ret) deb_data("--------Usb2_readControlBus fail : 0x%08lx\n", ret);
	
		copy_to_user(buf, buffer, 256);

		for(i = 0; i < 256; i++)
			deb_data("---------Read data buffer[%d] 0x%x\n", i, buf[i]);

		return 0;
#endif

}

static ssize_t it950x_read(
	struct file *file, 
	char __user *buf,
	size_t count, 
	loff_t *ppos)
{
        struct it950x_dev* dev = NULL;
	dev = (struct it950x_dev *)file->private_data;

        return it950x_usb_rx_read_dev(dev, buf, count);
}

ssize_t it950x_usb_tx_write_dev(
	struct it950x_dev *dev,
	const char __user *user_buffer,
	size_t count)
{
	Dword Len = count;
	DWORD dwError = Error_NO_ERROR;

	if (dev == NULL)
		return -ENODEV;
#if URB_TEST
	loop_cnt++;	
#endif
	dwError = WriteRingBuffer(dev, (Byte*)user_buffer, &Len);
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
		dwError = WriteRingBuffer_low_brate(dev, user_buffer, &Len);
	}else{
		//deb_data("it950x_tx_write: g_is_use_low_brate is close!\n");
		dwError = WriteRingBuffer(dev, user_buffer, &Len);
	}   
	if (dwError) deb_data("--------Usb2_writeControlBus fail : 0x%08lx\n", dwError);
	
	return count;
#endif
}

static ssize_t it950x_tx_write(
	struct file *file,
	const char __user *user_buffer,
	size_t count, 
	loff_t *ppos)
{
        struct it950x_dev *dev = NULL;
	/*AirHD RingBuffer*/
	dev = (struct it950x_dev *)file->private_data;

	return it950x_usb_tx_write_dev(dev, user_buffer, count);
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
	/*
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
	*/

	/*Allocate TX TX_CMD urb*/
	for (i = 0; i < URB_COUNT_TX_CMD; i++) {

		dev->urbs_cmd[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->urbs_cmd[i])
			retval = -ENOMEM;
	}
	for (i = 0; i < URB_COUNT_TX; i++) {

		dev->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->urbs[i])
			retval = -ENOMEM;
	}
	for(i = 0; i < URB_COUNT_TX_LOW_BRATE; i++){

		dev->urbs_low_brate[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!dev->urbs_low_brate[i])
			retval = -ENOMEM;
	}	
	
#if !(defined(DTVCAM_POWER_CTRL) && defined(AVSENDER_POWER_CTRL))
	atomic_set(&dev->tx_pw_on, 0);	
	atomic_set(&dev->rx_pw_on, 0);	
	DL_ApPwCtrl(&dev->DC, 1, 0);	
	DL_ApPwCtrl(&dev->DC, 0, 0);	
#endif	
	deb_data("USB ITEtech device now attached to USBSkel-%d \n", intf->minor);

        // init dvb netdevice
        // 
        dev->dvbdev = dvb_alloc_netdev(dev);
        if (dev->dvbdev == NULL)
	    deb_data("alloc netdev fail\n");

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
    if (dev->DeviceReboot == true) {
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
	
	/* 
	intf->minor = dev->rx_chip_minor;
	usb_deregister_dev(intf, &it950x_class);

	intf->minor = dev->tx_chip_minor;
	usb_deregister_dev(intf, &it950x_class_tx);
	*/

	mutex_unlock(&it950x_mutex);
	//unlock_kernel();
	
	/* decrement our usage count */
	//	kref_put(&dev->kref, afa_delete);

	if (dev){
		if(dev->file) dev->file->private_data = NULL;
		if(dev->tx_file) dev->tx_file->private_data = NULL;
		kfree(dev);
	}

	dvb_free_netdev(dev->dvbdev);
	
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

module_init (it950x_module_init);
module_exit (it950x_module_exit);

MODULE_AUTHOR("Jason Dong <jason.dong@ite.com.tw>");
MODULE_DESCRIPTION("DTV Native Driver for Device Based on ITEtech it950x");
MODULE_VERSION(DRIVER_RELEASE_VERSION);
MODULE_LICENSE("GPL");

/*
EXPORT_SYMBOL(it950x_usb_tx_alloc_dev);
EXPORT_SYMBOL(it950x_usb_tx_free_dev);
EXPORT_SYMBOL(it950x_usb_tx_ioctl_dev);
EXPORT_SYMBOL(it950x_usb_tx_unlocked_ioctl_dev);
EXPORT_SYMBOL(it950x_usb_tx_write_dev);
EXPORT_SYMBOL(it950x_usb_rx_alloc_dev);
EXPORT_SYMBOL(it950x_usb_rx_free_dev);
EXPORT_SYMBOL(it950x_usb_rx_ioctl_dev);
EXPORT_SYMBOL(it950x_usb_rx_unlocked_ioctl_dev);
EXPORT_SYMBOL(it950x_usb_rx_read_dev);
*/
