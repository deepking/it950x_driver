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


