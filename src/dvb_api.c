#include "dvb_api.h"
#include "error.h"

#define IN
#define OUT
#define INOUT

#define _debug(fmt, args...)

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
#define tx_ioctl(dev, cmd, arg) it950x_usb_tx_unlocked_ioctl_dev(dev, cmd, (unsigned long) arg)
#define rx_ioctl(dev, cmd, arg) it950x_usb_rx_unlocked_ioctl_dev(dev, cmd, (unsigned long) arg)
#else
#define tx_ioctl(dev, cmd, arg) it950x_usb_tx_ioctl_dev(dev, cmd, (unsigned long) arg)
#define rx_ioctl(dev, cmd, arg) it950x_usb_rx_ioctl_dev(dev, cmd, (unsigned long) arg)
#endif

// --------------------------------------------------------------------------
// TX
// --------------------------------------------------------------------------

/**
 * @param bfrequency frequency in KHz
 * @param bbandwidth bandwidth in KHz
 * @return error code
 */
Dword g_ITEAPI_TxSetChannel(struct it950x_dev* dev, IN Dword bfrequency, IN Word bbandwidth)
{
    Dword dwError = Error_NO_ERROR;
    TxAcquireChannelRequest request;
    request.chip = 0;
    request.bandwidth = bbandwidth;
    request.frequency = bfrequency;
    
    tx_ioctl(dev, IOCTL_ITE_DEMOD_ACQUIRECHANNEL_TX, &request);
    dwError = request.error;

    return dwError;
}

Dword g_ITEAPI_TxSetChannelModulation(struct it950x_dev* dev, PSetModuleRequest req)
{
    return tx_ioctl(dev, IOCTL_ITE_DEMOD_SETMODULE_TX, req);
}

Dword g_ITEAPI_StartTransfer(struct it950x_dev* dev)
{
    return tx_ioctl(dev, IOCTL_ITE_DEMOD_STARTTRANSFER_TX, NULL);
}

Dword g_ITEAPI_StopTransfer(struct it950x_dev* dev)
{
    return tx_ioctl(dev, IOCTL_ITE_DEMOD_STOPTRANSFER_TX, NULL);
}

Dword g_ITEAPI_GetDrvInfo(struct it950x_dev* dev, OUT PTxDemodDriverInfo pDriverInfo)
{
    return tx_ioctl(dev, IOCTL_ITE_DEMOD_GETDRIVERINFO_TX, pDriverInfo);
}

Dword g_ITEAPI_TxSendTSData(struct it950x_dev* dev, Byte* pBuffer, Dword pdwBufferLength)
{
    return it950x_usb_tx_write_dev(dev, pBuffer, pdwBufferLength);
}

Dword g_ITEAPI_TxSetPeridicCustomPacket(
        struct it950x_dev* dev,
        IN int bufferSize, 
        IN Byte *TableBuffer, 
        IN Byte index)
{
    Dword dwError = Error_NO_ERROR;
    int result;
    AccessFwPSITableRequest request; 

    if(bufferSize != 188)
        return -1;

    if (dev == NULL)
        return -1;

    request.psiTableIndex = index;
    request.pbuffer = TableBuffer;
    result = tx_ioctl(dev, IOCTL_ITE_DEMOD_ACCESSFWPSITABLE_TX, &request);
    dwError = request.error;

    return (dwError);
}

Dword g_ITEAPI_TxSetPeridicCustomPacketTimer(
        struct it950x_dev* dev,
        IN Byte index,
        IN Byte timer_interval)
{
    Dword dwError = Error_NO_ERROR;
    int result;
    SetFwPSITableTimerRequest request; 

    if (dev == NULL)
        return -1;

    request.psiTableIndex = index;
    request.timer = timer_interval;
    result = tx_ioctl(dev, IOCTL_ITE_DEMOD_SETFWPSITABLETIMER_TX, &request);
    dwError = request.error;

    return (dwError);
}

Dword g_ITEAPI_TxSendCustomPacketOnce(
        struct it950x_dev* dev,
        IN int bufferSize,
        IN Byte *TableBuffer)
{
    Dword dwError = Error_NO_ERROR;
    int result;
    SendHwPSITableRequest request; 

    if(bufferSize != 188)
        return -1;

    if (dev == NULL)
        return -1;

    request.pbuffer = TableBuffer;
    result = tx_ioctl(dev, IOCTL_ITE_DEMOD_SENDHWPSITABLE_TX, &request);
    dwError = request.error;

    return (dwError);
}

// --------------------------------------------------------------------------
// RX
// --------------------------------------------------------------------------

Dword DTV_AcquireChannel(
        struct it950x_dev* dev,
        IN  Dword dwFrequency,                // Channel Frequency (KHz)
        IN  Word  wBandwidth)                 // Channel Bandwidth (KHz)
{
    Dword dwError = Error_NO_ERROR;
    int result;
    AcquireChannelRequest request;

    if (wBandwidth < 10)
        wBandwidth *= 1000;

    request.chip = 0;
    request.frequency = dwFrequency;
    request.bandwidth = wBandwidth;
    result = rx_ioctl(dev, IOCTL_ITE_DEMOD_ACQUIRECHANNEL, &request);
    dwError = request.error;

    return (dwError);
}

Dword DTV_DisablePIDTbl(struct it950x_dev* dev)
{
    Dword dwError = Error_NO_ERROR;
    int result;
    ControlPidFilterRequest request;

    request.chip = 0;
    request.control = 0;
    result = rx_ioctl(dev, IOCTL_ITE_DEMOD_CONTROLPIDFILTER, &request);
    dwError = request.error;

    return(dwError);
}

Dword DTV_IsLocked(struct it950x_dev* dev,
        OUT Bool* pbLocked)
{
    Dword dwError = Error_NO_ERROR;
    int result;
    IsLockedRequest request;

    request.chip = 0;
    request.locked = (Bool *)pbLocked;
    result = rx_ioctl(dev, IOCTL_ITE_DEMOD_ISLOCKED, &request);
    dwError = request.error;

    return (dwError);
}

Dword DTV_StartCapture(struct it950x_dev* dev)
{
    return rx_ioctl(dev, IOCTL_ITE_DEMOD_STARTCAPTURE, NULL);
}

Dword DTV_StopCapture(struct it950x_dev* dev)
{
    return rx_ioctl(dev, IOCTL_ITE_DEMOD_STOPCAPTURE, NULL);
}

Dword DTV_GetData(
        struct it950x_dev* dev,
        OUT Byte* pBuffer,
        IN OUT Dword* pdwBufferLength)
{
    Dword dwError = Error_NO_ERROR;
    GetDatagramRequest request;

    request.bufferLength = pdwBufferLength;
    request.buffer = pBuffer;
    request.error = 0;

    *pdwBufferLength = it950x_usb_rx_read_dev(dev, pBuffer, *pdwBufferLength);
    dwError = request.error;

    return dwError;
}
