#include "modulatorIocontrol.h"
#include "it950x-core.h"

#define IN
#define OUT
#define INOUT

#define _debug(fmt, args...)

#define tx_ioctl(dev, cmd, arg)
#define rx_ioctl(dev, cmd, arg)

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

Dword g_ITEAPI_TxSendTSData(struct it950x_dev* dev, OUT Byte* pBuffer, IN OUT Dword pdwBufferLength)
{
}


Dword g_ITEAPI_StartCapture(struct it950x_dev* dev)
{
    return rx_ioctl(dev, IOCTL_ITE_DEMOD_STARTCAPTURE, NULL);
}

Dword g_ITEAPI_StopCapture(struct it950x_dev* dev)
{
    return rx_ioctl(dev, IOCTL_ITE_DEMOD_STOPCAPTURE, NULL);
}

/*
static int GetDriverInfo(unsigned char handleNum)
{
    unsigned int ChipType = 0;		
    unsigned char Tx_NumOfDev = 0;
    TxDemodDriverInfo driverInfo;
    unsigned long dwError = ERR_NO_ERROR;

    if((dwError = g_ITEAPI_TxGetNumOfDevice(&Tx_NumOfDev)) == ERR_NO_ERROR)
        _debug("%d Devices\n", Tx_NumOfDev);
    else 
        _debug("g_ITEAPI_TxGetNumOfDevice error\n");	

    if((dwError = g_ITEAPI_TxDeviceInit(handleNum)) == ERR_NO_ERROR)
        _debug("g_ITEAPI_TxDeviceInit ok\n");
    else 
        _debug("g_ITEAPI_TxDeviceInit fail\n");

    if((dwError = g_ITEAPI_TxLoadIQTableFromFile()) == ERR_NO_ERROR)
        _debug("g_ITEAPI_TxLoadIQTableFromFile ok\n");
    else
        _debug("g_ITEAPI_TxLoadIQTableFromFile fail\n");		

    if((dwError = g_ITEAPI_TxGetChipType(&ChipType)) == ERR_NO_ERROR)
        _debug("g_ITE_TxGetChipType ok\n");
    else
        _debug("g_ITE_TxGetChipType fail\n");

    if ((dwError = g_ITEAPI_GetDrvInfo(&driverInfo))) {
        _debug("Get Driver Info failed 0x%lu!\n", dwError);
    }
    else {
        _debug("g_ITEAPI_GetDrvInfo ok\n");		
        _debug("DriverVerion  = %s\n", driverInfo.DriverVerion);
        _debug("APIVerion     = %s\n", driverInfo.APIVerion);
        _debug("FWVerionLink  = %s\n", driverInfo.FWVerionLink);
        _debug("FWVerionOFDM  = %s\n", driverInfo.FWVerionOFDM);
        _debug("Company       = %s\n", driverInfo.Company);
        _debug("SupportHWInfo = %s\n", driverInfo.SupportHWInfo);
        _debug("ChipType      = 0x%x", ChipType);
    }

    return dwError;
}
*/
