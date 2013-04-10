#include "modulatorIocontrol.h"
#include "modulatorType.h"
#include "it950x-core.h"

/**
 * @param bfrequency frequency in KHz
 * @param bbandwidth bandwidth in KHz
 * @return error code
 */
Dword g_ITEAPI_TxSetChannel(struct it950x_dev* dev, IN Dword bfrequency, IN Word bbandwidth);


Dword g_ITEAPI_TxSetChannelModulation(struct it950x_dev* dev, PSetModuleRequest req);


Dword g_ITEAPI_StartTransfer(struct it950x_dev* dev);

Dword g_ITEAPI_StopTransfer(struct it950x_dev* dev);

Dword g_ITEAPI_GetDrvInfo(struct it950x_dev* dev, OUT PTxDemodDriverInfo pDriverInfo);

Dword g_ITEAPI_TxSendTSData(struct it950x_dev* dev, Byte* pBuffer, Dword pdwBufferLength);

Dword g_ITEAPI_TxSetPeridicCustomPacket(
        struct it950x_dev* dev,
        IN int bufferSize, 
        IN Byte *TableBuffer, 
        IN Byte index);

Dword g_ITEAPI_TxSetPeridicCustomPacketTimer(
        struct it950x_dev* dev,
        IN Byte index,
        IN Byte timer_interval);

Dword g_ITEAPI_TxSendCustomPacketOnce(
        struct it950x_dev* dev,
        IN int bufferSize,
        IN Byte *TableBuffer);

/**
 * @param dwFrequency frequency in KHz
 * @param dwBandwidth frequency in KHz
 */
Dword DTV_AcquireChannel(
        struct it950x_dev* dev,
        IN  Dword dwFrequency,
        IN  Word  wBandwidth);

Dword DTV_DisablePIDTbl(struct it950x_dev* dev);
Dword DTV_IsLocked(struct it950x_dev* dev, OUT Bool* pbLocked);
Dword DTV_StartCapture(struct it950x_dev* dev);
Dword DTV_StopCapture(struct it950x_dev* dev);

Dword DTV_GetData(
        struct it950x_dev* dev,
        OUT Byte* pBuffer,
        IN OUT Dword* pdwBufferLength);
