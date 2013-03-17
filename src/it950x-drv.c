#include "it950x-core.h"
#include "modulatorFirmware.h"
#include "firmware.h"
#include "firmware_V2.h"
#include "IQ_fixed_table.h"


#define FW_VER         0x08060000
#define TURN_OFF_UNUSED_OLD_POWER_CTRL

int dvb_usb_it950x_debug;
module_param_named(debug,dvb_usb_it950x_debug, int, 0644);

static DEFINE_MUTEX(mymutex);

static DWORD DRV_NIMReset(
	void* handle);

static DWORD DRV_InitNIMSuspendRegs(
	void* handle);
	
DWORD DRV_TunerSuspend(
	void * handle,
	BYTE ucChip,
	bool bOn);

static DWORD DRV_NIMSuspend(
	void * handle,
	bool bSuspend);

static DWORD DRV_NIMReset(
	void* handle);

#ifndef TURN_OFF_UNUSED_OLD_POWER_CTRL
static DWORD DL_NIMReset(
	void* handle);
#endif

static DWORD DRV_InitNIMSuspendRegs(
	void* handle);

#ifndef TURN_OFF_UNUSED_OLD_POWER_CTRL
static DWORD DL_InitNIMSuspendRegs(
	void* handle);
#endif

static DWORD DRV_Initialize(
	void* handle);

static DWORD DL_Initialize(
	void* handle);
  
//************** DRV_ *************//
/*
static DWORD DRV_SetArchitecture(
	void* handle,
	Architecture architecture)
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	
	//deb_data("- Enter %s Function -\n ",__FUNCTION__);
	
	dwError= Eagle_setArchitecture ((Demodulator*) &pdc->Demodulator);	
	
	return(dwError);
}
*/
static DWORD DRV_IrTblDownload(IN void* handle)
{
        DWORD dwError = Error_NO_ERROR;
        PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
        struct file *filp;
        unsigned char b_buf[512] ;
        int i, fileSize;
        mm_segment_t oldfs;

        deb_data("- Enter %s Function -\n",__FUNCTION__);

        oldfs=get_fs();
        set_fs(KERNEL_DS);

        filp=filp_open("/lib/firmware/af35irtbl.bin", O_RDWR,0644);
        if ( IS_ERR(filp) ) {
                deb_data("      LoadIrTable : Can't open file\n");goto exit;}

        if ( (filp->f_op) == NULL ) {
                deb_data("      LoadIrTable : File Operation Method Error!!\n");goto exit;}

        filp->f_pos=0x00;
        fileSize = filp->f_op->read(filp,b_buf,sizeof(b_buf),&filp->f_pos);

        for(i=0; i<fileSize; i++)
        {
              //deb_data("\n Data %d",i); //
              //deb_data("0x%x",b_buf[i]);//
              // dwError = Af901xWriteReg(ucDemod2WireAddr, 0, MERC_IR_TABLE_BASE_ADDR + i, b_buf[i]);
              //if (dwError) goto exit;
        }

        dwError = IT9507_loadIrTable((Modulator*) &pdc->modulator, (Word)fileSize, b_buf);
        if (dwError) {deb_data("Demodulator_loadIrTable fail"); goto exit;}

        filp_close(filp, NULL);
        set_fs(oldfs);

exit:
		deb_data("LoadIrTable fail!\n");
        return (dwError);


}

/*
static DWORD  DRV_GetEEPROMConfig2(
        void *      handle,
        BYTE       ucSlaveDemod)
{

	DWORD dwError = Error_NO_ERROR;
    	tWORD    shift = 0;
    	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	BYTE btmp = 0;

	deb_data("- Enter %s Function -",__FUNCTION__);
	
    	if(ucSlaveDemod) shift = EEPROM_SHIFT;
    
    	dwError = Demodulator_readRegisters((Demodulator*) &pdc->Demodulator, 0, Processor_LINK, EEPROM_TUNERID+shift, 1, &btmp);
    	//if (dwError) goto exit;
	//tunerID option, in Omega, not need to read register, just assign 0x38;
    	//btmp = 0x38;		
	//btmp = 0x51;	
	if (btmp != 0x51)
		btmp = 0x38;

    	deb_data("EEPROM_TUNERID%d  = 0x%02X\n", ucSlaveDemod, btmp);		
    	PTI.TunerId = btmp;  

//exit:
 
    return(dwError);  

}  
*/
static DWORD DRV_SetFreqBw(
    	void*	handle,      
     	BYTE 	ucSlaveDemod,
      	DWORD   dwFreq,      
      	WORD	ucBw
)
{
    	DWORD dwError = Error_NO_ERROR;
    
    	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;

	//Bool bLock = true;

	deb_data("- Enter %s Function -\n ",__FUNCTION__);
	deb_data("	ucSlaveDemod = %d, Freq= %ld, BW=%d\n", ucSlaveDemod, dwFreq, ucBw);

    if (pdc->fc[ucSlaveDemod].bEnPID)
    {
        IT9507_resetPidFilter((Modulator*) &pdc->modulator);
        //Disable PID filter
        IT9507_writeRegisterBits ((Modulator*) &pdc->modulator, Processor_OFDM, p_mp2if_pid_en, mp2if_pid_en_pos, mp2if_pid_en_len, 0);
    }
	
    (pdc->fc[ucSlaveDemod].tunerinfo).bSettingFreq = true; //before acquireChannel, it is ture;  otherwise, it is false

    if(dwFreq) {
        pdc->fc[ucSlaveDemod].ulDesiredFrequency = dwFreq;
    }
    else {
        dwFreq = pdc->fc[ucSlaveDemod].ulDesiredFrequency;
    }

    if(ucBw) {
        pdc->fc[ucSlaveDemod].ucDesiredBandWidth = ucBw*1000;
	}
    else {
        ucBw = pdc->fc[ucSlaveDemod].ucDesiredBandWidth;
    	}

    deb_data("	Real Freq= %ld, BW=%d\n", pdc->fc[ucSlaveDemod].ulDesiredFrequency, pdc->fc[ucSlaveDemod].ucDesiredBandWidth);
           

    if(!pdc->fc[ucSlaveDemod].tunerinfo.bTunerInited){
        deb_data("	Skip SetFreq - Tuner is still off!\n");
        goto exit;
    }
	
    pdc->fc[ucSlaveDemod].tunerinfo.bTunerOK = false;        

    if (pdc->fc[ucSlaveDemod].ulDesiredFrequency!=0 && pdc->fc[ucSlaveDemod].ucDesiredBandWidth!=0)	
    {
	deb_data("	AcquireChannel : Real Freq= %ld, BW=%d\n", pdc->fc[ucSlaveDemod].ulDesiredFrequency, pdc->fc[ucSlaveDemod].ucDesiredBandWidth);
	//dwError = Demodulator_acquireChannel ((Demodulator*) &pdc->Demodulator, ucSlaveDemod, pdc->fc[ucSlaveDemod].ucDesiredBandWidth, pdc->fc[ucSlaveDemod].ulDesiredFrequency);  
	//PTI.bSettingFreq = false;
    	if (dwError) 
    	{
        	deb_data("	Demod_acquireChannel fail! 0x%08ld\n", dwError);
        	goto exit;
    	}	
	else //when success acquireChannel, record currentFreq/currentBW.
	{
		pdc->fc[ucSlaveDemod].ulCurrentFrequency = pdc->fc[ucSlaveDemod].ulDesiredFrequency;	
		pdc->fc[ucSlaveDemod].ucCurrentBandWidth = pdc->fc[ucSlaveDemod].ucDesiredBandWidth;  
	}
    }

    if(pdc->StreamType == StreamType_DVBT_DATAGRAM) {
        pdc->fc[ucSlaveDemod].OvrFlwChk = 5;
    }
  
    /*if (pdc->fc[ucSlaveDemod].ulDesiredFrequency!=0 && pdc->fc[ucSlaveDemod].ucDesiredBandWidth!=0)
    {
	// patch for IT9507_isLocked
	//mdelay(700);

    	dwError= IT9507_isLocked((Demodulator*) &pdc->Demodulator, ucSlaveDemod, &bLock);
    	if(dwError)  
        	deb_data("	Demodulator_isLocked is failed!\n"); 
    	else 
	{
        	deb_data("	The signal is %s Lock\n", bLock?"":"not"); 

		//patch for mce channel change lag
		if(bLock) {
			mdelay(500); 
		}
    	}
    }*/

    pdc->fc[ucSlaveDemod].tunerinfo.bTunerOK = true;

exit:

    pdc->fc[ucSlaveDemod].tunerinfo.bSettingFreq = false;

    return(dwError);  
}

static DWORD DRV_isLocked(
        void*   handle,
        BYTE    ucSlaveDemod,
        Bool*   bLock   
)
{
        DWORD dwError = Error_NO_ERROR;
        PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
        *bLock = true;

	deb_data("- Enter %s Function -\n ",__FUNCTION__);

	dwError= Demodulator_isLocked((Demodulator*) &pdc->demodulator, bLock);
        if(dwError)  
                deb_data("      IT950x_isLocked is failed!\n"); 
        else 
                deb_data("      The chip=%d signal is %s Lock\n", ucSlaveDemod, *bLock?"":"not"); 

	return(dwError);
}
static DWORD DRV_getSignalStrength(
        void*   handle,
        BYTE    ucSlaveDemod,
        BYTE*    strength  
)
{
        DWORD dwError = Error_NO_ERROR;
        PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;

        deb_data("- Enter %s Function -\n ",__FUNCTION__);

	 dwError= Demodulator_getSignalStrengthIndication((Demodulator*) &pdc->demodulator, strength);
	if(dwError)
                deb_data("      IT9507_getSignalStrength is failed!\n");
	else
                deb_data("      The signal strength is %d \n", *strength);

	return(dwError);
}

static DWORD DRV_getSignalStrengthDbm(
    void*   handle,
    BYTE    ucSlaveDemod,
    Long*   strengthDbm
)
{
    DWORD dwError = Error_NO_ERROR;
    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;

    deb_data("- Enter %s Function -\n ",__FUNCTION__);

    dwError= Demodulator_getSignalStrengthIndication((Demodulator*) &pdc->demodulator, (Byte *)strengthDbm);
    if(dwError)
    {
    	deb_data("      IT9507_getSignalStrengthDbm is failed!\n");
    }
    else
    {
    	deb_data("      The signal strengthDbm is %ld \n", *strengthDbm);
    }

    return(dwError);
}
/*
static DWORD DRV_getChannelStatistic(
    void*   handle,
    BYTE    ucSlaveDemod,
    ChannelStatistic*           channelStatistic
)
{
    DWORD dwError = Error_NO_ERROR;
    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
    
    deb_data("- Enter %s Function -\n ",__FUNCTION__);
    
    dwError= IT9507_getChannelStatistic((Demodulator*) &pdc->Demodulator, channelStatistic);
    if(dwError)
	deb_data("      Demodulator_getChannelStatistic is failed!\n");

    return(dwError);
}
*/
static DWORD DRV_getChannelModulation(
    void*   handle,
    BYTE    ucSlaveDemod,
    ChannelModulation*      channelModulation
)
{
    DWORD dwError = Error_NO_ERROR;
    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;

    deb_data("- Enter %s Function -\n ",__FUNCTION__);

    dwError= Standard_getChannelModulation((Demodulator*) &pdc->demodulator, channelModulation);
    if(dwError)
	deb_data("      Demodulator_getChannelStatistic is failed!\n");

    return(dwError);
}

static DWORD DRV_getSNRValue(
    void*   handle,
    DWORD*   snr_value
)
{
    DWORD dwError = Error_NO_ERROR;
    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
    BYTE snr_reg_23_16, snr_reg_15_8, snr_reg_7_0;
    
    deb_data("- Enter %s Function -\n ",__FUNCTION__);


    dwError = IT9507_readRegister((Modulator*) &pdc->modulator,Processor_OFDM, 0x2e,(unsigned char *) &snr_reg_23_16);
    if(dwError)
	deb_data("      Demodulator_getSNR snr_reg_23_16 is failed!\n");

     dwError = IT9507_readRegister((Modulator*) &pdc->modulator,Processor_OFDM, 0x2d,(unsigned char *) &snr_reg_15_8);
     if(dwError)
	deb_data("      IT9507_getSNR snr_reg_15_8 is failed!\n");

    dwError = IT9507_readRegister((Modulator*) &pdc->modulator,Processor_OFDM, 0x2c,(unsigned char *) &snr_reg_7_0);
    if(dwError)
	deb_data("      Demodulator_getSNR snr_reg_7_0 is failed!\n");

    *snr_value = (snr_reg_23_16&0xff)*256*256+(snr_reg_15_8&0xff)*256+(snr_reg_7_0&0xff);

    return(dwError);
}

static DWORD DRV_getFirmwareVersionFromFile( 
		void* handle,
	 	Processor	processor, 
		DWORD* 		version
)
{

	
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	BYTE chip_version = 0;
	DWORD chip_Type;
	BYTE var[2];
	DWORD error = Error_NO_ERROR;

	DWORD OFDM_VER1;
    DWORD OFDM_VER2;
    DWORD OFDM_VER3;
    DWORD OFDM_VER4;

    DWORD LINK_VER1;
    DWORD LINK_VER2;
    DWORD LINK_VER3;    
    DWORD LINK_VER4;    
    

	error = IT9507_readRegister((Modulator*) &pdc->modulator, processor, chip_version_7_0, &chip_version);
	error = IT9507_readRegisters((Modulator*) &pdc->modulator, processor, chip_version_7_0+1, 2, var);
	
	if(error) deb_data("DRV_getFirmwareVersionFromFile fail");
	
	chip_Type = var[1]<<8 | var[0];	
	if(chip_Type == 0x9135 && chip_version == 2){
		OFDM_VER1 = DVB_V2_OFDM_VERSION1;
		OFDM_VER2 = DVB_V2_OFDM_VERSION2;
		OFDM_VER3 = DVB_V2_OFDM_VERSION3;
		OFDM_VER4 = DVB_V2_OFDM_VERSION4;

		LINK_VER1 = DVB_V2_LL_VERSION1;
		LINK_VER2 = DVB_V2_LL_VERSION2;
		LINK_VER3 = DVB_V2_LL_VERSION3;    
		LINK_VER4 = DVB_V2_LL_VERSION4;
	}else{
		OFDM_VER1 = DVB_OFDM_VERSION1;
    	OFDM_VER2 = DVB_OFDM_VERSION2;
   	 	OFDM_VER3 = DVB_OFDM_VERSION3;
    	OFDM_VER4 = DVB_OFDM_VERSION4;

   		LINK_VER1 = DVB_LL_VERSION1;
    	LINK_VER2 = DVB_LL_VERSION2;
    	LINK_VER3 = DVB_LL_VERSION3;    
    	LINK_VER4 = DVB_LL_VERSION4;
	}

    if(processor == Processor_OFDM) {
        *version = (DWORD)( (OFDM_VER1 << 24) + (OFDM_VER2 << 16) + (OFDM_VER3 << 8) + OFDM_VER4);
    }
    else { //LINK
        *version = (DWORD)( (LINK_VER1 << 24) + (LINK_VER2 << 16) + (LINK_VER3 << 8) + LINK_VER4);    
    }
    
    return *version;
}

DWORD DRV_getDeviceType(void *handle)
{
	DWORD dwError = Error_NO_ERROR;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;

	dwError =  EagleUser_getDeviceType((Modulator*) &PDC->modulator, &PDC->deviceType);

    return(dwError);
}

static DWORD  DRV_Initialize(
	    void *      handle
)
{
	DWORD error = Error_NO_ERROR;
	DWORD error_rx = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	Byte temp = 0;
	//Byte usb_dma_reg;
	BYTE chip_version = 0; 
	DWORD fileVersion, cmdVersion = 0; 
	SystemConfig syscfg;

	deb_data("- Enter %s Function -\n",__FUNCTION__);

	if(EagleUser_getDeviceType((Modulator*) &pdc->modulator, &pdc->deviceType) != 0)
		printk("- EagleUser_getDeviceType fail -\n");

	if(EagleUser_getSystemConfig((Modulator*) &pdc->modulator, pdc->deviceType, &syscfg) != 0)
		printk("- EagleUser_getSystemConfig fail -\n");	
 
	pdc->modulator.systemConfig = syscfg;

	if(IT9507_setSlaveIICAddress((Modulator*) &pdc->modulator, SLAVE_DEMOD_2WIREADDR) != 0)
		printk("- IT9507_setSlaveIICAddress fail -\n");	
	
	
	if(pdc->modulator.booted) //from Standard_setBusTuner() > Standard_getFirmwareVersion()
    	{
        	//use "#define version" to get fw version (from firmware.h title)
        	error = DRV_getFirmwareVersionFromFile(handle, Processor_OFDM, &fileVersion);

        	//use "Command_QUERYINFO" to get fw version 
        	error = IT9507_getFirmwareVersion((Modulator*) &pdc->modulator, Processor_OFDM, &cmdVersion);
        	if(error) deb_data("DRV_Initialize : IT9507_getFirmwareVersion : error = 0x%08ld\n", error);

        	if(cmdVersion != fileVersion)
        	{
            		deb_data("Reboot: Outside Fw = 0x%lx, Inside Fw = 0x%lx", fileVersion, cmdVersion);
            		error = IT9507_TXreboot((Modulator*) &pdc->modulator);
            		pdc->bBootCode = true;
            		if(error) 
            		{
                		deb_data("IT9507_reboot : error = 0x%08ld\n", error);
                		return error;
            		}
            		else {
                		return Error_NOT_READY;
            		}
        	}
        	else
        	{
            		deb_data("	Fw version is the same!\n");
  	      		error = Error_NO_ERROR;
        	}
	}//pdc->IT950x.booted
//ReInit:  //Patch for NIM fail or disappear, Maggie   
#if 0   
    error = IT9507_initialize ((Demodulator*) &pdc->demodulator , StreamType_DVBT_PARALLEL, Bus_USB, EagleUser_DEVICETYPE); 
    if (error) 
    { 
        deb_data("IT9507_initialize _Device initialize fail : 0x%08x\n", error);
		/*
        if( ((error&Error_FIRMWARE_STATUS) && (error&0x10)) && (pdc->Demodulator.chipNumber>1) )
        {
            pdc->Demodulator.cmdDescription->sendCommand ((Demodulator*) &pdc->Demodulator, Command_FW_DOWNLOAD_END, 0, Processor_LINK, 0, NULL, 0, NULL);

            deb_data("	Retry to download FW with Single TS\n");
            pdc->Demodulator.chipNumber = 1;
            pdc->bDualTs = false;
            error = Demodulator_writeRegister ((Demodulator*) &pdc->Demodulator, 0, Processor_LINK, 0x417F, 0);
            goto ReInit;
       }
       */
    }
    pdc->demodulator.userData = &pdc->modulator;
    error = Demodulator_initialize ((Demodulator*) &pdc->demodulator , StreamType_DVBT_PARALLEL); 
    if (error) 
    { 
        deb_data("Demodulator_initialize_Device initialize fail : 0x%08x\n", error);
		/*
        if( ((error&Error_FIRMWARE_STATUS) && (error&0x10)) && (pdc->Demodulator.chipNumber>1) )
        {
            pdc->Demodulator.cmdDescription->sendCommand ((Demodulator*) &pdc->Demodulator, Command_FW_DOWNLOAD_END, 0, Processor_LINK, 0, NULL, 0, NULL);

            deb_data("	Retry to download FW with Single TS\n");
            pdc->Demodulator.chipNumber = 1;
            pdc->bDualTs = false;
            error = Demodulator_writeRegister ((Demodulator*) &pdc->Demodulator, 0, Processor_LINK, 0x417F, 0);
            goto ReInit;
       }
       */
    }
    else {
        deb_data("    Device initialize Ok!!\n");
    }
#endif
	error = IT9507_readRegister ((Modulator*) &pdc->modulator, Processor_LINK, 0x4979, &temp);//has eeprom ??
	if (error) deb_data("Eagle_readRegister: 0x%08ld\n", error);

    if (temp == 1){		
    error = IT9507_readRegister ((Modulator*) &pdc->modulator, Processor_LINK, 0x49CA, &temp);
    if (error) deb_data("Eagle_readRegister: 0x%08ld\n", error);
    
    switch(temp)
    {
		case 3:
		{
			deb_data("    StreamType_DVBT_DATAGRAM\n");
			
			error = IT9507_initialize ((Modulator*) &pdc->modulator , StreamType_DVBT_DATAGRAM, 2, 0);
			 
			if (error) deb_data("IT950x_initialize _Device initialize fail : 0x%08ld\n", error);
			else deb_data("    Device initialize TX Ok!!\n");
			
			pdc->demodulator.userData = &pdc->modulator;
			error_rx = Demodulator_readRegister((Demodulator*) &pdc->demodulator, Processor_LINK, chip_version_7_0, &chip_version);
			
			if (error_rx) deb_data("    Device initialize RX Fail!!\n");
			else {
				error = OMEGA_supportLNA((Demodulator*) &pdc->demodulator ,2);
				if (error) deb_data("OMEGA_supportLNAfail : 0x%08ld\n", error);
				else deb_data("    OMEGA_supportLNA Ok!!\n");
				error = Demodulator_initialize ((Demodulator*) &pdc->demodulator , StreamType_DVBT_PARALLEL); 
				if (error) deb_data("Demodulator_initialize_Device initialize fail : 0x%08ld\n", error);
				else deb_data("    Device initialize RX Ok!!\n");
			}
			break;
		}
		case 4:
		{
			deb_data("    StreamType_DVBT_PARALLEL\n");
			
			error = IT9507_initialize ((Modulator*) &pdc->modulator , StreamType_DVBT_PARALLEL, 2, 0);			
			if (error) deb_data("IT950x_initialize _Device initialize fail : 0x%08ld\n", error);
			else deb_data("    Device initialize TX Ok!!\n");
			
			pdc->demodulator.userData = &pdc->modulator;
			error_rx = Demodulator_readRegister((Demodulator*) &pdc->demodulator, Processor_LINK, chip_version_7_0, &chip_version);

			if (error_rx) deb_data("    Device initialize RX Fail!!\n");
			else {
				error = OMEGA_supportLNA((Demodulator*) &pdc->demodulator ,2);
				if (error) deb_data("OMEGA_supportLNAfail : 0x%08ld\n", error);
				else deb_data("    OMEGA_supportLNA Ok!!\n");
				error = Demodulator_initialize ((Demodulator*) &pdc->demodulator , StreamType_DVBT_PARALLEL); 
				if (error) deb_data("Demodulator_initialize_Device initialize fail : 0x%08ld\n", error);
				else deb_data("    Device initialize RX Ok!!\n");
			}
			break;
		}
		case 5:
		{
			deb_data("    StreamType_DVBT_SERIAL\n"); 
			
			error = IT9507_initialize ((Modulator*) &pdc->modulator , StreamType_DVBT_SERIAL, Bus_USB, EagleUser_DEVICETYPE);
			
			if (error) deb_data("IT9507_initialize _Device initialize fail : 0x%08ld\n", error);
			else deb_data("    Device initialize TX Ok!!\n");
			
			pdc->demodulator.userData = &pdc->modulator;
			
			error_rx = Demodulator_readRegister((Demodulator*) &pdc->demodulator, Processor_LINK, chip_version_7_0, &chip_version);
			if (error_rx) deb_data("    Device initialize RX Fail!!\n");
			else {
				error = OMEGA_supportLNA((Demodulator*) &pdc->demodulator ,2);
				if (error) deb_data("OMEGA_supportLNAfail : 0x%08ld\n", error);
				else deb_data("    OMEGA_supportLNA Ok!!\n");
				error = Demodulator_initialize ((Demodulator*) &pdc->demodulator , StreamType_DVBT_PARALLEL); 
				if (error) deb_data("Demodulator_initialize_Device initialize fail : 0x%08ld\n", error);
				else deb_data("    Device initialize RX Ok!!\n");
			}
			break;
		}
		
	} 
	
	}
	else{
		deb_data("    StreamType_DVBT_PARALLEL\n");

		error = IT9507_initialize ((Modulator*) &pdc->modulator , StreamType_DVBT_PARALLEL, Bus_USB, EagleUser_DEVICETYPE);
		
		if (error) deb_data("IT950x_initialize _Device initialize fail : 0x%08ld\n", error);
		else deb_data("    Device initialize TX Ok!!\n");


		pdc->demodulator.userData = &pdc->modulator;
		error_rx = Demodulator_readRegister((Demodulator*) &pdc->demodulator, Processor_LINK, chip_version_7_0, &chip_version);

		if (error_rx) deb_data("    Device initialize RX Fail!!\n");
		else {

			error = OMEGA_supportLNA((Demodulator*) &pdc->demodulator ,2);
			if (error) deb_data("OMEGA_supportLNAfail : 0x%08ld\n", error);
			else deb_data("    OMEGA_supportLNA Ok!!\n");
			
			error = Demodulator_initialize ((Demodulator*) &pdc->demodulator , StreamType_DVBT_PARALLEL);
			if (error) deb_data("Demodulator_initialize_Device initialize fail : 0x%08ld\n", error);
			else deb_data("    Device initialize RX Ok!!\n");
			//error_rx = Demodulator_writeRegister((Demodulator*) &pdc->demodulator, Processor_LINK, 0xd830, 1);			
		}
	}
	
    IT9507_getFirmwareVersion ((Modulator*) &pdc->modulator, Processor_OFDM, &cmdVersion);
    deb_data("    FwVer OFDM = 0x%lx, ", cmdVersion);
    IT9507_getFirmwareVersion ((Modulator*) &pdc->modulator, Processor_LINK, &cmdVersion);
    deb_data("FwVer LINK = 0x%lx\n", cmdVersion);
    
	/* Solve 0-byte packet error. write Link 0xDD8D[3] = 1 */
	//error = Demodulator_readRegister((Demodulator*) &pdc->demodulator, Processor_LINK, 0xdd8d, &usb_dma_reg);
	//usb_dma_reg |= 0x08;             /*reg_usb_min_len*/
	//error = Demodulator_writeRegister((Demodulator*) &pdc->demodulator, Processor_LINK, 0xdd8d, usb_dma_reg);
    
    return error;
	
}

static DWORD DRV_InitDevInfo(
    	void *      handle,
    	BYTE        ucSlaveDemod
)
{
    DWORD dwError = Error_NO_ERROR;    
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
    PDC->fc[ucSlaveDemod].ulCurrentFrequency = 0;  
    PDC->fc[ucSlaveDemod].ucCurrentBandWidth = 0;

    PDC->fc[ucSlaveDemod].ulDesiredFrequency = 0;	
    PDC->fc[ucSlaveDemod].ucDesiredBandWidth = 6000;	

    //For PID Filter Setting
    //PDC->fc[ucSlaveDemod].ulcPIDs = 0;    
    PDC->fc[ucSlaveDemod].bEnPID = false;

    PDC->fc[ucSlaveDemod].bApOn = false;
    
    PDC->fc[ucSlaveDemod].bResetTs = false;



    PDC->fc[ucSlaveDemod].tunerinfo.bTunerOK = false;
    PDC->fc[ucSlaveDemod].tunerinfo.bSettingFreq = false;

    return dwError;
}	

//get EEPROM_IRMODE/bIrTblDownload/bRAWIr/architecture config from EEPROM
static DWORD DRV_GetEEPROMConfig(
	void* handle)
{       
    DWORD dwError = Error_NO_ERROR;
	BYTE chip_version = 0;
	DWORD chip_Type;
	BYTE  var[2];
    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	//bIrTblDownload option
    Byte btmp = 0;
	int ucSlaveDemod;
	
	deb_data("- Enter %s Function -",__FUNCTION__);

	//patch for read eeprom valid bit
	dwError = IT9507_readRegister((Modulator*) &pdc->modulator, Processor_LINK, chip_version_7_0, &chip_version);
	dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, chip_version_7_0+1, 2, var);

	if(dwError) deb_data("DRV_GetEEPROMConfig fail---cant read chip version");

	chip_Type = var[1]<<8 | var[0];
	if(chip_Type==0x9135 && chip_version == 2) //Om2
	{
		pdc->chip_version = 2;
		dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, 0x461d, 1, &btmp);
		deb_data("Chip Version is %d---and Read 461d---valid bit = 0x%02X", chip_version, btmp);
	}
	else 
	{
		pdc->chip_version = 1; //Om1
		dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, 0x4979, 1, &btmp);
		deb_data("Chip Version is %d---and Read 4979---valid bit = 0x%02X", chip_version, btmp);
	}
	if (dwError) 
	{
		deb_data("0x461D eeprom valid bit read fail!");
		goto exit;
    }

	if(btmp == 0)
	{
		deb_data("=============No need read eeprom");
		pdc->bIrTblDownload = false;
		pdc->bProprietaryIr = false;
		pdc->bSupportSelSuspend = false;
		pdc->bDualTs = false;
    	pdc->architecture = Architecture_DCA;
    	//pdc->modulator.chipNumber = 1;    
    	pdc->bDCAPIP = false;
		pdc->fc[0].tunerinfo.TunerId = 0x38;
	}
	else
	{
		deb_data("=============Need read eeprom");
		dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, EEPROM_IRMODE, 1, &btmp);
    	if (dwError) goto exit;
    	pdc->bIrTblDownload = btmp ? true:false;
    	deb_data("EEPROM_IRMODE = 0x%02X, ", btmp);	
    	deb_data("bIrTblDownload %s\n", pdc->bIrTblDownload?"ON":"OFF");
    	pdc->bProprietaryIr = (btmp==0x05)?true:false;
    	deb_data("bRAWIr %s\n", pdc->bProprietaryIr?"ON":"OFF");
		if(pdc->bProprietaryIr)
		{
			deb_data("IT950x proprietary (raw) mode\n");
		}
		else
		{
			deb_data("IT950x HID (keyboard) mode\n");
		}		    	        	    
		//EE chose NEC RC5 RC6 threshhold table
		if(pdc->bIrTblDownload)
		{
			dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, EEPROM_IRTYPE, 1, &btmp);
    		if (dwError) goto exit;
			pdc->bIrType = btmp;
			deb_data("bIrType 0x%02X", pdc->bIrType);
		}
//selective suspend
    	pdc->bSupportSelSuspend = false;
	    //btmp = 0; //not support in v8.12.12.3
   		//dwError = IT9507_readRegisters((Demodulator*) &pdc->Demodulator, Processor_LINK, EEPROM_SELSUSPEND, 1, &btmp);
   	 	//if (dwError) goto exit;
    	//if(btmp<2)
    	//	PDC->bSupportSelSuspend = btmp ? true:false; 
    	deb_data("SelectiveSuspend = %s", pdc->bSupportSelSuspend?"ON":"OFF");
    	    
//bDualTs option   
    	pdc->bDualTs = false;
    	pdc->architecture = Architecture_DCA;
    	//pdc->modulator.chipNumber = 1;    
    	pdc->bDCAPIP = false;

    	dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, EEPROM_TSMODE, 1, &btmp);
    	if (dwError) goto exit;
    	deb_data("EEPROM_TSMODE = 0x%02X", btmp);

    	if (btmp == 0)     
    	{  
        	deb_data("TSMode = TS1 mode\n");
    	}
    	else if (btmp == 1) 
   		{
        	deb_data("TSMode = DCA+PIP mode\n");
			pdc->architecture = Architecture_DCA;
        	//pdc->modulator.chipNumber = 2;
        	pdc->bDualTs = true;
        	pdc->bDCAPIP = true;
    	}
    	else if (btmp == 2) 
    	{ 
        	deb_data("TSMode = DCA mode\n");
        	//pdc->modulator.chipNumber = 2;
    	}
    	else if (btmp == 3) 
    	{
        	deb_data("TSMode = PIP mode\n");
        	pdc->architecture = Architecture_PIP;
        	//pdc->modulator.chipNumber = 2;
        	pdc->bDualTs = true;
    	}

//tunerID option, in Omega, not need to read register, just assign 0x38;
		dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, EEPROM_TUNERID, 1, &btmp);
		if (btmp==0x51) {
			pdc->fc[0].tunerinfo.TunerId = 0x51;  	
		}
		else if (btmp==0x52) {
			pdc->fc[0].tunerinfo.TunerId = 0x52;  	
		}
		else if (btmp==0x60) {
			pdc->fc[0].tunerinfo.TunerId = 0x60;  	
		}
		else if (btmp==0x61) {
			pdc->fc[0].tunerinfo.TunerId = 0x61;  	
		}
		else if (btmp==0x62) {
			pdc->fc[0].tunerinfo.TunerId = 0x62;  	
		}
		else {
			pdc->fc[0].tunerinfo.TunerId = 0x38;  	
		}		
	
		deb_data("pdc->fc[0].tunerinfo.TunerId = 0x%x", pdc->fc[0].tunerinfo.TunerId);
		if (pdc->bDualTs) {
			pdc->fc[1].tunerinfo.TunerId = pdc->fc[0].tunerinfo.TunerId;
			deb_data("pdc->fc[1].tunerinfo.TunerId = 0x%x", pdc->fc[1].tunerinfo.TunerId);
		}

		//dwError = IT9507_writeRegister((Demodulator*) &pdc->Demodulator, 0, Processor_LINK, EEPROM_SUSPEND, 0);
		dwError = IT9507_readRegisters((Modulator*) &pdc->modulator, Processor_LINK, EEPROM_SUSPEND, 1, &btmp);
		deb_data("EEPROM susped mode=%d", btmp);
    	
    }
		//pdc->modulator.chipNumber = 1; 
//init some device info
	for(ucSlaveDemod = 0; ucSlaveDemod <= (BYTE)pdc->bDualTs; ucSlaveDemod++)
	{
		dwError = DRV_InitDevInfo(handle, ucSlaveDemod);
	}
	
exit:
    return(dwError);
}


/*
static DWORD DRV_GetEEPROMConfig(    
	void *      handle)
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
    	BYTE ucSlaveDemod = 0;
	BYTE btmp = 0;
	int cnt;
	
	deb_data("- Enter %s Function -",__FUNCTION__);	
   
	//bIrTblDownload option
	dwError =   IT9507_readRegisters((Demodulator*) &pdc->Demodulator,  Processor_LINK, EEPROM_IRMODE, 1, &btmp);
	if (dwError) return(dwError);
	PDC->bIrTblDownload = btmp ? true:false;
	deb_data(	"EEPROM_IRMODE = 0x%02X, ", btmp);
        deb_data("bIrTblDownload %s\n", PDC->bIrTblDownload?"ON":"OFF");
 
    	PDC->bDualTs = false;
    	PDC->architecture = Architecture_DCA;
    	PDC->IT950x.chipNumber = 1;   
    	PDC->bDCAPIP = false;

	//bDualTs option
	dwError = IT9507_readRegisters((Demodulator*) &pdc->Demodulator,Processor_LINK,EEPROM_TSMODE, 1, &btmp);
	if (dwError) return(dwError);
	deb_data("EEPROM_TSMODE = 0x%02X", btmp);

	if (btmp == 0)
	    deb_data("TSMode = TS1 mode\n");
	else if (btmp == 1)
	{
	    deb_data("TSMode = DCA+PIP mode\n");
	    PDC->architecture = Architecture_DCA;
	    PDC->Demodulator.chipNumber = 2;
	    PDC->bDualTs = true;
//	    PDC->bDCAPIP = true;
	}
	else if (btmp == 3)
	{
	     deb_data("TSMode = PIP mode\n");
	     PDC->architecture = Architecture_PIP;
	     PDC->Demodulator.chipNumber = 2;
	     PDC->bDualTs = true;
	}
	else
	{   
	    deb_data("TSMode = DCA mode\n");
	    PDC->architecture = Architecture_DCA;
	    PDC->Demodulator.chipNumber = 2;
	}

	if(PDC->bDualTs) {
	    cnt = 2;
	}
	else {
	    cnt = 1;
	}

	for(ucSlaveDemod; ucSlaveDemod < cnt; ucSlaveDemod++)
	{
	    dwError = DRV_GetEEPROMConfig2(pdc, ucSlaveDemod);
	    if (dwError) return(dwError);  
	    dwError = DRV_InitDevInfo(pdc, ucSlaveDemod);
	    if (dwError) return(dwError);
	}
    
   	 return(dwError);     
}   
*/
static DWORD DRV_SetBusTuner(
	 void * handle, 
	 Word busId, 
	 Word tunerId
)
{
	DWORD dwError = Error_NO_ERROR;
	DWORD 	 version = 0;

	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;

	deb_data("- Enter %s Function -",__FUNCTION__);
	deb_data("busId = 0x%x, tunerId =0x%x\n", busId, tunerId);

/*	if ((pdc->UsbMode==0x0110) && (busId==Bus_USB)) {
        busId=Bus_USB11;    
    }*/
    pdc->modulator.busId = busId;
//    	dwError = IT9507_setBus ((Modulator*) &pdc->modulator, busId);
//	if (dwError) {deb_data("Demodulator_setBusTuner error\n");return dwError;}

	dwError = IT9507_getFirmwareVersion ((Modulator*) &pdc->modulator, Processor_LINK, &version);
    	if (version != 0) {
        	pdc->modulator.booted = True;
    	} 
    	else {
        	pdc->modulator.booted = False;
    	}
	if (dwError) {deb_data("Demodulator_getFirmwareVersion  error\n");}

    	return(dwError); 
}

#ifndef TURN_OFF_UNUSED_OLD_POWER_CTRL
//set tuner power control
static DWORD DRV_TunerPowerCtrl(
	void * handle, 
	BYTE ucSlaveDemod,
	Bool bPowerOn)
{ 
	DWORD dwError = Error_NO_ERROR;	
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	
	deb_data("- Enter %s Function -chip = %d\n",__FUNCTION__, ucSlaveDemod);

	//Omega has no GPIOH7

	if (bPowerOn)	//up
	{
		//dwError = IT9507_writeRegisterBits((Demodulator*)&pdc->Demodulator, ucSlaveDemod, Processor_OFDM, p_reg_p_if_en, reg_p_if_en_pos, reg_p_if_en_len, 0x01);
		mdelay(50);
	}
	else //down
	{
		//part1
		//dwError = IT9507_writeRegisterBits((Demodulator*)&pdc->Demodulator, ucSlaveDemod, Processor_OFDM, p_reg_p_if_en, reg_p_if_en_pos, reg_p_if_en_len, 0x00);		
		//mdelay(5);

		//part2
		dwError = IT9507_writeRegister((Modulator*)&pdc->modulator, Processor_OFDM, 0xEC02, 0x3F);
		dwError = IT9507_writeRegister((Modulator*)&pdc->modulator,  Processor_OFDM, 0xEC03, 0x1F);//DCX, 110m
		dwError = IT9507_writeRegister((Modulator*)&pdc->modulator,  Processor_OFDM, 0xEC04, 0x3F);//73m, 46m
		dwError = IT9507_writeRegister((Modulator*)&pdc->modulator, Processor_OFDM, 0xEC05, 0x3F);//57m, 36m

		//part3
		dwError = IT9507_writeRegister((Modulator*)&pdc->modulator, Processor_OFDM, 0xEC3F, 0x01);
		//mdelay(5);

		if (ucSlaveDemod == 0) deb_data("--------------AP off last cmd -------------");
	}

	return(dwError);
}
#endif

/*
static DWORD DRV_TunerPowerCtrl(
    	void *	handle, 
     	BYTE	ucSlaveDemod,
     	Bool		bPowerOn
)
{ 
    DWORD dwError = Error_NO_ERROR;	

    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;

    deb_data("- Enter %s Function -",__FUNCTION__);
    deb_data("chip = %d\n", ucSlaveDemod); 

    // init gpioH7
    dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, 0, Processor_LINK,  p_reg_top_gpioh7_en, reg_top_gpioh7_en_pos, reg_top_gpioh7_en_len, 1);
    dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, 0, Processor_LINK,  p_reg_top_gpioh7_on, reg_top_gpioh7_on_pos, reg_top_gpioh7_on_len, 1);    

    if(bPowerOn)
        PTI.bTunerInited = true;
    else
        PTI.bTunerInited = false;    


    if(bPowerOn) //tuner on
    {
        dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, 0, Processor_LINK,  p_reg_top_gpioh7_o, reg_top_gpioh7_o_pos, reg_top_gpioh7_o_len, 1);    

        if(pdc->bTunerPowerOff == true) 
        {
            dwError = IT9507_initialize ((Demodulator*) &pdc->Demodulator, pdc->Demodulator.chipNumber , pdc->Demodulator.bandwidth[0], pdc->StreamType, pdc->architecture);  
            pdc->bTunerPowerOff = false;
        }              	        
    }
    else //tuner off
    {
        if(pdc->architecture == Architecture_PIP)
        {
            if(pdc->fc[0].tunerinfo.bTunerInited == false && pdc->fc[1].tunerinfo.bTunerInited == false) 
            {                                
                if(pdc->bTunerPowerOff == false) 
                {
                    dwError = IT9507_finalize((Demodulator*) &pdc->Demodulator);
                    pdc->bTunerPowerOff = true;
                }
                
                dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, 0, Processor_LINK,  p_reg_top_gpioh7_o, reg_top_gpioh7_o_pos, reg_top_gpioh7_o_len, 0);    
            }                   
        }
        else 
        {
            if(pdc->bTunerPowerOff == false) 
            {                
                dwError = IT9507_finalize((Demodulator*) &pdc->Demodulator);
                pdc->bTunerPowerOff = true;
            }      

            dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, 0, Processor_LINK,  p_reg_top_gpioh7_o, reg_top_gpioh7_o_pos, reg_top_gpioh7_o_len, 0);    
        }        	

    }
    return(dwError);
}
*/

DWORD NIM_ResetSeq(IN  void *	handle)
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	int i;
	//BYTE ucValue = 0;
	
	BYTE bootbuffer[6];
	//checksum = 0xFEDC
	bootbuffer[0] = 0x05;
	bootbuffer[1] = 0x00;
	bootbuffer[2] = 0x23;
	bootbuffer[3] = 0x01;
	bootbuffer[4] = 0xFE;
	bootbuffer[5] = 0xDC;	

	//aaa
	//reset 9133 -> boot -> demod init

	//GPIOH5 init
	dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_gpioh5_en, reg_top_gpioh5_en_pos, reg_top_gpioh5_en_len, 0);
	dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_gpioh5_on, reg_top_gpioh5_on_pos, reg_top_gpioh5_on_len, 0);
		
	//dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, 0, Processor_LINK, p_reg_top_gpioh5_o, reg_top_gpioh5_o_pos, reg_top_gpioh5_o_len, 0);
	//mdelay(100);

	deb_data("aaa start DRV_NIMReset");
	dwError = DRV_NIMReset(handle);

	dwError = DRV_InitNIMSuspendRegs(handle);
	
	deb_data("aaa start writeGenericRegisters");
	dwError = IT9507_writeGenericRegisters ((Modulator*) &pdc->modulator, 0x3a, 0x06, bootbuffer);
	
	deb_data("aaa start readGenericRegisters");
	dwError = IT9507_readGenericRegisters ((Modulator*) &pdc->modulator, 0x3a, 0x05, bootbuffer);
	deb_data("aaa print I2C reply");
	for(i=0; i<5; i++)
		deb_data("aaa bootbuffer[%d] = 0x%x", i, bootbuffer[i]);
	
	mdelay(50); //delay for Fw boot	

	//IT9507_readRegister((Demodulator*) &pdc->Demodulator, Processor_LINK, 0x4100, &ucValue);
	
	//Demod & Tuner init
	deb_data("aaa DL_Initialize");
	dwError = DRV_Initialize(handle);

	return dwError;
}


//set tuner power saving and control
static DWORD DRV_ApCtrl(
	void* handle,
	Byte ucSlaveDemod,
	Bool bOn)
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	int i;
    Dword version = 0;
	
	deb_data("- Enter %s Function -\n",__FUNCTION__);
	deb_data("ucSlaveDemod = %d, bOn = %s \n", ucSlaveDemod, bOn?"ON":"OFF");

	if(bOn) 
	{
		pdc->fc[ucSlaveDemod].tunerinfo.bTunerInited = true;
		//[OMEGA] 
		if (pdc->architecture == Architecture_PIP)
		{
			if (pdc->fc[0].bTimerOn || pdc->fc[1].bTimerOn) {				
				deb_data("Already all power on !!");
				return 0;
			}
		}
	}
	else 
	{	
		pdc->fc[ucSlaveDemod].tunerinfo.bTunerInited = false;	
		pdc->fc[ucSlaveDemod].tunerinfo.bTunerLock = false;
		
		//[OMEGA] if other one still alive, do not pwr down, just need to set freq;
		if (pdc->architecture == Architecture_PIP)
		{
			if (pdc->fc[0].bTimerOn || pdc->fc[1].bTimerOn) 
			{				
				deb_data("CLOSE 1<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
				IT9507_acquireTxChannel((Modulator*) &pdc->modulator, pdc->fc[!ucSlaveDemod].ucCurrentBandWidth, pdc->fc[!ucSlaveDemod].ulCurrentFrequency);
				return 0;
			}
		}
	}
   	//[OMEGA] clock from tuner, so close sequence demod->tuner, and 9133->9137. vice versa.
   	// BUT! demod->tuner:57mADC, tuner->demod:37mADC
	if (bOn)  //pwr on
	{
		if(pdc->chip_version == 1)
		{
			deb_data("aaa Power On\n");
			if (ucSlaveDemod == 1){
				dwError = NIM_ResetSeq(pdc);
				if(dwError)
				{								
					NIM_ResetSeq(pdc);
				}
			}
			else
			{
				//aaa			
				//CAP_KdPrint(("aaa DRV_TunerSuspend chip_%d", ucSlaveDemod));
				//dwError = DRV_TunerSuspend(handle, ucSlaveDemod, !bOn);
				//dwError = DRV_TunerPowerCtrl(handle, ucSlaveDemod, bOn);
				//if(dwError) TUNER_KdPrint(("DRV_ApCtrl::DRV_TunerSuspend Fail: 0x%04X", dwError));
				//DummyCmd
				pdc->UsbCtrlTimeOut = 1;
				for(i=0; i<5 ;i++) 
				{        
					deb_data("DRV_ApCtrl::DummyCmd %d\n", i);
					dwError = IT9507_getFirmwareVersion ((Modulator*) &pdc->modulator, Processor_LINK, &version);
					mdelay(1);
					//if (!dwError) break;
				}
				pdc->UsbCtrlTimeOut = 5;
				
				deb_data("aaa IT9507_controlTunerPowerSaving chip_%d\n", ucSlaveDemod);
	//			dwError = IT9507_controlTunerPower((Demodulator*) &pdc->Demodulator, bOn);
	//			if(dwError) deb_data("DRV_ApCtrl::IT9507_controlTunerPowerSaving error = 0x%04ld", dwError);

				deb_data("aaa IT9507_controlPowerSaving chip_%d\n", ucSlaveDemod);
				dwError = IT9507_controlPowerSaving((Modulator*) &pdc->modulator, bOn);
				if(dwError) deb_data("DRV_ApCtrl::IT9507_controlPowerSaving error = 0x%04ld", dwError);
			}
		}
		else
		{
			deb_data("aaa Power On\n");
			if (ucSlaveDemod == 1){
					deb_data("aaa GPIOH5 off\n");
					dwError = DRV_NIMSuspend(handle, false);
			}
			else
			{
				//DummyCmd
				pdc->UsbCtrlTimeOut = 1;
				for(i=0; i<5 ;i++) 
				{        
					deb_data("DRV_ApCtrl::DummyCmd %d\n", i);
					dwError = IT9507_getFirmwareVersion ((Modulator*) &pdc->modulator, Processor_LINK, &version);
					mdelay(1);
					//if (!dwError) break;
				}
				pdc->UsbCtrlTimeOut = 5;
			}
			//aaa			
			//CAP_KdPrint(("aaa DRV_TunerSuspend chip_%d", ucSlaveDemod));
			//dwError = DRV_TunerSuspend(handle, ucSlaveDemod, !bOn);
			//dwError = DRV_TunerPowerCtrl(handle, ucSlaveDemod, bOn);
			//if(dwError) TUNER_KdPrint(("DRV_ApCtrl::DRV_TunerSuspend Fail: 0x%04X", dwError));

			deb_data("aaa IT9507_controlTunerPowerSaving chip_%d\n", ucSlaveDemod);
	//		dwError = IT9507_controlTunerPower((Demodulator*) &pdc->Demodulator, bOn);
			if(dwError) deb_data("DRV_ApCtrl::IT9507_controlTunerPowerSaving error = 0x%04ld\n", dwError);

			deb_data("aaa IT9507_controlPowerSaving chip_%d\n", ucSlaveDemod);
			dwError = IT9507_controlPowerSaving((Modulator*) &pdc->modulator, bOn);
			mdelay(50);
			if(dwError) deb_data("DRV_ApCtrl::IT9507_controlPowerSaving error = 0x%04ld\n", dwError);
		}
        deb_data("aaa Power On End-----------\n");		
    }
	else //pwr down:  DCA DUT: 36(all) -> 47-159(no GPIOH5, sequence change)
	{
		deb_data("aaa Power OFF\n");
	/*	if ( (ucSlaveDemod == 0) && (pdc->Demodulator.chipNumber == 2) ){
			
			//deb_data("aaa IT9507_controlTunerLeakage for Chip_1");
			//dwError = IT9507_controlTunerLeakage((Demodulator*) &pdc->Demodulator, 1, bOn);
			//if(dwError) deb_data("DRV_ApCtrl::IT9507_controlTunerLeakage error = 0x%04ld", dwError);

			//deb_data("aaa GPIOH5 on");
			//dwError = DRV_NIMSuspend(handle, true);
			
		}*/
	
		deb_data("aaa IT9507_controlPowerSaving chip_%d\n", ucSlaveDemod);
		dwError = IT9507_controlPowerSaving((Modulator*) &pdc->modulator, bOn);
		if(dwError) deb_data("DRV_ApCtrl::IT9507_controlPowerSaving error = 0x%04ld\n", dwError);
		
		deb_data("aaa IT9507_controlTunerPowerSaving chip_%d", ucSlaveDemod);
//		dwError = IT9507_controlTunerPower((Demodulator*) &pdc->Demodulator, bOn);
		if(dwError) deb_data("DRV_ApCtrl::IT9507_controlTunerPowerSaving error = 0x%04ld\n", dwError);
		//deb_data("aaa DRV_TunerSuspend chip_%d", ucSlaveDemod);
		//dwError = DRV_TunerSuspend(handle, ucSlaveDemod, !bOn);
		//dwError = DRV_TunerPowerCtrl(handle, ucSlaveDemod, bOn);		
		//if(dwError) deb_data("DRV_ApCtrl::DRV_TunerSuspend Fail: 0x%04X", dwError);
	
		deb_data("aaa Power OFF End-----------\n");
    }

	return(dwError);
}


/*
static DWORD DRV_ApCtrl (
      void *      handle,
      Byte        ucSlaveDemod,
      Bool        bOn
)
{
	DWORD dwError = Error_NO_ERROR;

        PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;

	deb_data("- Enter %s Function -\n",__FUNCTION__);
	deb_data(" ucSlaveDemod = %d, bOn = %s \n", ucSlaveDemod, bOn?"ON":"OFF"); 

      //deb_data("enter DRV_ApCtrl: Demod[%d].GraphBuilt = %d", ucSlaveDemod, pdc->fc[ucSlaveDemod].GraphBuilt); 

	dwError = DRV_TunerPowerCtrl(pdc, ucSlaveDemod, bOn);
       	if(dwError) { deb_data("	DRV_TunerPowerCtrl Fail: 0x%08x\n", dwError); return(dwError);}

	
    	dwError = IT9507_controlPowerSaving((Demodulator*) &pdc->Demodulator, bOn);   
    	if(dwError) { deb_data("	DRV_ApCtrl: Demodulator_controlPowerSaving error = 0x%08x\n", dwError); return(dwError);}
	
    return(dwError);
}
*/

/*
static DWORD DRV_ApReset(
	void* handle,
	Byte ucSlaveDemod,
	Bool bOn)
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	
	deb_data("- Enter %s Function -\n",__FUNCTION__);
	
	pdc->fc[ucSlaveDemod].ulCurrentFrequency = 0;
	pdc->fc[ucSlaveDemod].ucCurrentBandWidth = 6000; //for BDAUtil 
	pdc->fc[ucSlaveDemod].bApOn = false;
	
	if(!bOn) { //Only for stop
		pdc->fc[ucSlaveDemod].bEnPID = false;
	}

	//init Tuner info
	pdc->fc[ucSlaveDemod].tunerinfo.bSettingFreq = false;
	
	dwError = DRV_ApCtrl(pdc, ucSlaveDemod, bOn);
	if(dwError) deb_data("DRV_ApCtrl Fail!");

	return(dwError);
}
*/

static DWORD DRV_TunerWakeup(
      void *     handle
)
{   
    	DWORD dwError = Error_NO_ERROR;

    	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT) handle;

	deb_data("- Enter %s Function -\n",__FUNCTION__);

	//tuner power on
	dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK,  p_reg_top_gpioh7_o, reg_top_gpioh7_o_pos, reg_top_gpioh7_o_len, 1);

    return(dwError);

}

/*
DWORD DRV_TunerSuspend(
	IN void * handle, 
	IN BYTE ucChip, 
	IN bool bOn)
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT) handle;
	
	deb_data("enter DRV_TunerSuspend, bOn=%d\n", bOn);	
	
	if (bOn) 
	{
//EC40 *		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_p_if_en, reg_p_if_en_pos, reg_p_if_en_len, 0);
		if(dwError) goto exit;
#if 1
//current = 190	 
//EC02~EC0F
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_pd_a, reg_pd_a_pos, reg_pd_a_len, 0);
		dwError = DRV_WriteRegister(pdc, ucChip, Processor_OFDM, 0xEC03, 0x0C);
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_pd_c, reg_pd_c_pos, reg_pd_c_len, 0);		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_pd_d, reg_pd_d_pos, reg_pd_d_len, 0);				
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_tst_a, reg_tst_a_pos, reg_tst_a_len, 0);		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_tst_b, reg_tst_b_pos, reg_tst_b_len, 0);
		if(dwError) goto exit;

//current = 172
//KEY: p_reg_ctrl_a: 0 fail/ 7x/ 1x/ 2x/ 3x/ 4x/ 5/ 6
		deb_data("aaa DRV_TunerSuspend::0xEC08 active");
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ctrl_a, reg_ctrl_a_pos, reg_ctrl_a_len, 0);				
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ctrl_b, reg_ctrl_b_pos, reg_ctrl_b_len, 0);						
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_cal_freq_a, reg_cal_freq_a_pos, reg_cal_freq_a_len, 0);			
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_cal_freq_b, reg_cal_freq_b_pos, reg_cal_freq_b_len, 0);					
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_cal_freq_c, reg_cal_freq_c_pos, reg_cal_freq_c_len, 0);					
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lo_freq_a, reg_lo_freq_a_pos, reg_lo_freq_a_len, 0);							
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lo_freq_b, reg_lo_freq_b_pos, reg_lo_freq_b_len, 0);											
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lo_freq_c, reg_lo_freq_c_pos, reg_lo_freq_c_len, 0);
		if(dwError) goto exit;

//current=139
//EC10~EC15		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lo_cap, reg_lo_cap_pos, reg_lo_cap_len, 0);		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_clk02_select, reg_clk02_select_pos, reg_clk02_select_len, 0);				
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_clk01_select, reg_clk01_select_pos, reg_clk01_select_len, 0);						
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lna_g, reg_lna_g_pos, reg_lna_g_len, 0);								
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lna_cap, reg_lna_cap_pos, reg_lna_cap_len, 0);										
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lna_band, reg_lna_band_pos, reg_lna_band_len, 0);												
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_pga, reg_pga_pos, reg_pga_len, 0);
		if(dwError) goto exit;

//current=119
//EC17 -> EC1F
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_pgc, reg_pgc_pos, reg_pgc_len, 0);	
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lpf_cap, reg_lpf_cap_pos, reg_lpf_cap_len, 0);
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lpf_bw, reg_lpf_bw_pos, reg_lpf_bw_len, 0);		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ofsi, reg_ofsi_pos, reg_ofsi_len, 0);				
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ofsq, reg_ofsq_pos, reg_ofsq_len, 0);						
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_dcxo_a, reg_dcxo_a_pos, reg_dcxo_a_len, 0);								
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_dcxo_b, reg_dcxo_b_pos, reg_dcxo_b_len, 0);								
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_tddo, reg_tddo_pos, reg_tddo_len, 0);										
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_strength_setting, reg_strength_setting_pos, reg_strength_setting_len, 0);
		if(dwError) goto exit;
//EC22~EC2B
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_gi, reg_gi_pos, reg_gi_len, 0);														
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_clk_del_sel, reg_clk_del_sel_pos, reg_clk_del_sel_len, 0);																
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_p2s_ck_sel, reg_p2s_ck_sel_pos, reg_p2s_ck_sel_len, 0);																
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_rssi_sel, reg_rssi_sel_pos, reg_rssi_sel_len, 0);																		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_tst_sel, reg_tst_sel_pos, reg_tst_sel_len, 0);																				
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ctrl_c, reg_ctrl_c_pos, reg_ctrl_c_len, 0);
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ctrl_d, reg_ctrl_d_pos, reg_ctrl_d_len, 0);
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ctrl_e, reg_ctrl_e_pos, reg_ctrl_e_len, 0);
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ctrl_f, reg_ctrl_f_pos, reg_ctrl_f_len, 0);
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_lo_bias, reg_lo_bias_pos, reg_lo_bias_len, 0);
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_ext_lna_en, reg_ext_lna_en_pos, reg_ext_lna_en_len, 0);		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_pga_bak, reg_pga_bak_pos, reg_pga_bak_len, 0);		
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_cpll_cap, reg_cpll_cap_pos, reg_cpll_cap_len, 0);
		if(dwError) goto exit;
//EC20
		dwError = DRV_WriteRegister(pdc, ucChip, Processor_OFDM, 0xEC20, 0x00);
		if(dwError) goto exit;

//current=116
#endif
		dwError = DRV_WriteRegister(pdc, ucChip, Processor_OFDM, 0xEC3F, 0x01);
		if(dwError) goto exit;
	}
	else 
	{
		dwError = DRV_WriteRegisterBits(pdc, ucChip, Processor_OFDM, p_reg_p_if_en, reg_p_if_en_pos, reg_p_if_en_len, 1);
		mdelay(50);
	}
exit:
	if(dwError) deb_data("DRV_TunerSuspend failed !!!\n");

	return(dwError);
}
*/
static DWORD DRV_Reboot(
      void *     handle
)
{
	DWORD dwError = Error_NO_ERROR;

        PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT) handle;

        deb_data("- Enter %s Function -\n",__FUNCTION__);

        dwError = IT9507_TXreboot((Modulator*) &pdc->modulator);

	return(dwError);
}


#ifndef TURN_OFF_UNUSED_OLD_POWER_CTRL
static DWORD DRV_USBSetup(
    void*	handle
)
{
    DWORD dwError = Error_NO_ERROR;

    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT) handle;
    int i;

    deb_data("- Enter %s Function -\n",__FUNCTION__);



    /*if (pdc->Demodulator.chipNumber == 2)
    {
	//timing
	for (i=0; i<2; i++)
	{
	    //dwError = IT9507_writeRegisterBits ((Demodulator*) &pdc->Demodulator, Processor_OFDM, p_reg_dca_fpga_latch, reg_dca_fpga_latch_pos, reg_dca_fpga_latch_len, 0x66);
	    if(dwError) return (dwError);
	    //dwError = IT9507_writeRegisterBits ((Demodulator*) &pdc->Demodulator, Processor_OFDM, p_reg_dca_platch, reg_dca_platch_pos, reg_dca_platch_len, 1);
	    if(dwError) return (dwError);
	}
    }*/
    return(dwError);
}
#endif
/*
DWORD DRV_GPIOCtrl(
	IN void * handle, 
	IN BYTE ucIndex, 
	IN bool bHigh)
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT) handle;

	deb_data("enter DRV_GPIOCtrl, Idx=%d, High=%d\n", ucIndex, bHigh);	

DWORD gpio_x_en = p_reg_top_gpiohIdx_en; 
DWORD gpio_x_en_pos;
DWORD gpio_x_en_len;
p_reg_top_gpioh5_en
reg_top_gpioh5_en_pos	
reg_top_gpioh5_en_len

DWORD gpio_x_on;
DWORD gpio_x_on_pos;
DWORD gpio_x_on_len;
p_reg_top_gpioh5_on
reg_top_gpioh5_on_pos	
reg_top_gpioh5_on_len

DWORD gpio_x_o;
DWORD gpio_x_o_pos;
DWORD gpio_x_o_len;
p_reg_top_gpioh5_o
reg_top_gpioh5_o_pos	
reg_top_gpioh5_o_len

	//output
	dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, Processor_LINK, p_reg_top_gpioh3_en, reg_top_gpioh3_en_pos, reg_top_gpioh3_en_len, 1);
	dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, Processor_LINK, p_reg_top_gpioh3_on, reg_top_gpioh3_on_pos, reg_top_gpioh3_on_len, 1);
	//control
	dwError = IT9507_writeRegisterBits((Demodulator*) &pdc->Demodulator, Processor_LINK, p_reg_top_gpioh3_o, reg_top_gpioh3_o_pos, reg_top_gpioh3_o_len, 0);
	if(dwError) deb_data("DRV_GPIOCtrl failed !!!\n");

	return(dwError);
}
*/

static DWORD DRV_NIMSuspend(
    void *      handle,
    bool        bSuspend

)
{
    DWORD dwError = Error_NO_ERROR;

    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT) handle;

    deb_data("- Enter DRV_NIMSuspend : bSuspend = %s\n", bSuspend ? "ON":"OFF");

    if(bSuspend) { //sleep
    	dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_gpioh5_o, reg_top_gpioh5_o_pos, reg_top_gpioh5_o_len, 1);
		if(dwError) return (dwError);
    } else {        //resume 
		dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_gpioh5_o, reg_top_gpioh5_o_pos, reg_top_gpioh5_o_len, 0);
		if(dwError) return (dwError);
    }

    return(dwError);
}

static DWORD DRV_InitNIMSuspendRegs(
    void *      handle
)
{
    DWORD dwError = Error_NO_ERROR;

    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT) handle;
    deb_data("- Enter %s Function -\n",__FUNCTION__);

    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_gpioh5_en, reg_top_gpioh5_en_pos, reg_top_gpioh5_en_len, 1);
    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_gpioh5_on, reg_top_gpioh5_on_pos, reg_top_gpioh5_on_len, 1);
    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_gpioh5_o, reg_top_gpioh5_o_pos, reg_top_gpioh5_o_len, 0);

    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_pwrdw, reg_top_pwrdw_pos, reg_top_pwrdw_len, 1);

    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK, p_reg_top_pwrdw_hwen, reg_top_pwrdw_hwen_pos, reg_top_pwrdw_hwen_len, 1);

    return(dwError);
}

static DWORD DRV_NIMReset(
    void *      handle
)
{


    DWORD   dwError = Error_NO_ERROR;

    PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
    deb_data("- Enter %s Function -\n",__FUNCTION__);
    //Set AF0350 GPIOH1 to 0 to reset AF0351

    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK,  p_reg_top_gpioh1_en, reg_top_gpioh1_en_pos, reg_top_gpioh1_en_len, 1);
    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK,  p_reg_top_gpioh1_on, reg_top_gpioh1_on_pos, reg_top_gpioh1_on_len, 1);
    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK,  p_reg_top_gpioh1_o, reg_top_gpioh1_o_pos, reg_top_gpioh1_o_len, 0);

    mdelay(50);

    dwError = IT9507_writeRegisterBits((Modulator*) &pdc->modulator, Processor_LINK,  p_reg_top_gpioh1_o, reg_top_gpioh1_o_pos, reg_top_gpioh1_o_len, 1);

    return(dwError);
}

/*
DWORD DRV_LoadIQtable_Fromfile(
           IN    void *      handle
) {
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	ssize_t error = Error_NO_ERROR;
	BYTE FileName_buffer[200] = {0};
	BYTE  Readbuf[20] = {0};
	LONGLONG fileSize = 0;
	int LoadingLength = 8;
	int i = 0;
	struct file *hTsFile;
	struct inode *inode = NULL;  
    mm_segment_t oldfs;
	char Default_IQtable_Path[200] = "/lib/firmware/bin/IQtable000.bin";
	

	oldfs=get_fs();
	set_fs(KERNEL_DS);
	snprintf(FileName_buffer, 200,  "/lib/firmware/bin/IQtable00%d.bin", 1); 
	hTsFile = filp_open(FileName_buffer, O_RDWR, 0644);
	
	if (IS_ERR(hTsFile)) {
		//printk("Load IQtable fail with status %u !!!!\n", hTsFile);
		//printk("Load Default TABLE: %s\n", Default_IQtable_Path);		   
		hTsFile = filp_open(Default_IQtable_Path, O_RDWR, 0644);
		if (IS_ERR(hTsFile)) {
			   error = Error_OPEN_FILE_FAIL;
			   printk("Load default IQtable fail with status %u !", error);
			   goto exit;
		}else
			printk("Load default IQtable000 ");
	}else
		printk("Load IQtable00%d ", 1);		

	inode = hTsFile->f_dentry->d_inode;
	fileSize = inode->i_size;

	if ( (hTsFile->f_op) == NULL ) {
			deb_data("LoadIQtable : File Operation Method Error!\n");goto exit;}

	hTsFile->f_pos=0x00;
	vfs_read(hTsFile, Readbuf, LoadingLength*2, &hTsFile->f_pos);
	fileSize -= LoadingLength*2;   //skip filename & index

	while(fileSize >= 8){
		vfs_read(hTsFile, Readbuf, LoadingLength, &hTsFile->f_pos);
		pdc->modulator.ptrIQtableEx[i].frequency = (Readbuf[0] + Readbuf[1]*256 
              									+ Readbuf[2]*256*256 + Readbuf[3]*256*256*256);
		pdc->modulator.ptrIQtableEx[i].dAmp = (short)(Readbuf[4] + Readbuf[5]*256);
		pdc->modulator.ptrIQtableEx[i].dPhi = (short)(Readbuf[6] + Readbuf[7]*256);                   

		fileSize -= LoadingLength;
		i++;
	}
	printk("remaining file size = %d", fileSize);
		   
//	for(i = 0; i < 91; i++) {
//		 printk("IQ_tableEx[%d][0] = %d\n",i, pdc->modulator.ptrIQtableEx[i].frequency);
//		 printk("IQ_tableEx[%d][1] = %d\n",i, pdc->modulator.ptrIQtableEx[i].dAmp);
//		 printk("IQ_tableEx[%d][2] = %d\n",i, pdc->modulator.ptrIQtableEx[i].dPhi);
//	}
	if (hTsFile)        
		filp_close(hTsFile, NULL);
exit:

    set_fs(oldfs);
	return error;
}
*/
//************** DL_ *************//
#ifndef TURN_OFF_UNUSED_OLD_POWER_CTRL
static DWORD DL_NIMReset(
    void *      handle
)
{
	DWORD dwError = Error_NO_ERROR;

    mutex_lock(&mymutex);
    
    dwError = DRV_NIMReset(handle);

    mutex_unlock(&mymutex);

    return (dwError);
}
#endif

#ifndef TURN_OFF_UNUSED_OLD_POWER_CTRL
static DWORD DL_USBSetup(
    void *      handle
)
{
	DWORD dwError = Error_NO_ERROR;

	mutex_lock(&mymutex);

	dwError = DRV_USBSetup(handle);

	mutex_unlock(&mymutex);

    return (dwError);
}
#endif

static DWORD DL_NIMSuspend(
    void *      handle,
    bool	bSuspend
)
{
	DWORD dwError = Error_NO_ERROR;

	mutex_lock(&mymutex);

    dwError = DRV_NIMSuspend(handle, bSuspend);

    mutex_unlock(&mymutex);

    return (dwError);
}

#ifndef TURN_OFF_UNUSED_OLD_POWER_CTRL
static DWORD DL_InitNIMSuspendRegs(
    void *      handle
)
{
	DWORD dwError = Error_NO_ERROR;
    
	mutex_lock(&mymutex);

    dwError = DRV_InitNIMSuspendRegs(handle);

    mutex_unlock(&mymutex);

    return (dwError);
}
#endif

static DWORD DL_Initialize(
	    void *      handle
)
{
	DWORD dwError = Error_NO_ERROR;
    mutex_lock(&mymutex);

    dwError = DRV_Initialize(handle);

	mutex_unlock(&mymutex);

	return (dwError); 
    
}

static DWORD DL_SetBusTuner(
	 void * handle, 
	 Word busId, 
	 Word tunerId
)
{
	DWORD dwError = Error_NO_ERROR;
	
	mutex_lock(&mymutex);

    dwError = DRV_SetBusTuner(handle, busId, tunerId);

    mutex_unlock(&mymutex);

	return (dwError);

}

static DWORD  DL_GetEEPROMConfig(
	 void *      handle
)
{   
	DWORD dwError = Error_NO_ERROR;
    mutex_lock(&mymutex);

    dwError = DRV_GetEEPROMConfig(handle);

    mutex_unlock(&mymutex);

    return(dwError);
} 

static DWORD DL_TunerWakeup(
      void *     handle
)
{    
	DWORD dwError = Error_NO_ERROR;
    mutex_lock(&mymutex);

    dwError = DRV_TunerWakeup(handle);

    mutex_unlock(&mymutex);
   
    	return(dwError);
}
static DWORD  DL_IrTblDownload(
      void *     handle
)
{
	DWORD dwError = Error_NO_ERROR;

    mutex_lock(&mymutex);

	dwError = DRV_IrTblDownload(handle);

    mutex_unlock(&mymutex);

    return(dwError);
}


DWORD DL_TunerPowerCtrl(void* handle, u8 bPowerOn)
{
	DWORD dwError = Error_NO_ERROR;
	BYTE    ucSlaveDemod=0;
  	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;

    mutex_lock(&mymutex);

	deb_data("enter DL_TunerPowerCtrl:  bOn = %s\n", bPowerOn?"ON":"OFF");

/*    for (ucSlaveDemod=0; ucSlaveDemod<PDC->modulator.chipNumber; ucSlaveDemod++)
    {
    	dwError = DRV_TunerPowerCtrl(PDC, ucSlaveDemod, bPowerOn);
    	if(dwError) deb_data("  DRV_TunerPowerCtrl Fail: 0x%08ld\n", dwError);
    }*/

    mutex_unlock(&mymutex);

    return (dwError);
}

DWORD DL_ApPwCtrl (
	void* handle,
    Bool  bChipCtl,
    Bool  bOn
)
{
    DWORD dwError = Error_NO_ERROR;
	BYTE    i = 0;
	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT)handle;
	
	mutex_lock(&mymutex);

	deb_data("- Enter %s Function -",__FUNCTION__);
	deb_data("  chip =  %d  bOn = %s\n", bChipCtl, bOn?"ON":"OFF");

	if(bChipCtl) {    // 913x
		if(bOn) {		// resume
			deb_data("IT9130x Power ON\n");		
			dwError = IT9507_writeRegisterBits((Modulator*) &PDC->modulator, Processor_LINK, p_reg_top_gpioh5_o, reg_top_gpioh5_o_pos, reg_top_gpioh5_o_len, 0);
			dwError = Demodulator_controlPowerSaving ((Demodulator*) &PDC->demodulator, bOn);				
			if(dwError) { 
				deb_data("ApCtrl::IT913x chip resume error = 0x%04ld\n", dwError); 
				goto exit;
			}
		} else {       // suspend
			deb_data("IT9130x Power OFF\n");	
			dwError = IT9507_writeRegisterBits((Modulator*) &PDC->modulator, Processor_LINK, p_reg_top_gpioh5_o, reg_top_gpioh5_o_pos, reg_top_gpioh5_o_len, 1);
			dwError = Demodulator_controlPowerSaving ((Demodulator*) &PDC->demodulator, bOn);	
			if(dwError) { 
				deb_data("ApCtrl::IT913x chip suspend error = 0x%04ld\n", dwError); 
				goto exit;
			}			
		}
	} else {          // 9507
		if(bOn) {	  // resume
			deb_data("IT950x Power ON\n");				
			dwError = IT9507_controlPowerSaving ((Modulator*) &PDC->modulator, bOn);				
			if(dwError) { 
				deb_data("ApCtrl::IT9507_controlPowerSaving error = 0x%04ld\n", dwError); 
				goto exit;
			}
		} else {      // suspend
			//deb_data("IT950x TxMode RF OFF\n");				
			dwError = IT9507_setTxModeEnable((Modulator*) &PDC->modulator, 0);
			if(dwError) {
				deb_data("ApCtrl::IT9507_setTxModeEnable error = 0x%04ld\n", dwError);
				goto exit;				
			}
			deb_data("IT950x Power OFF\n");							
			dwError = IT9507_controlPowerSaving ((Modulator*) &PDC->modulator, bOn);			
			if(dwError) {
				deb_data("ApCtrl::IT9507_controlPowerSaving error = 0x%04ld\n", dwError);
				goto exit;
			}
		}			
	}

exit:
    mutex_unlock(&mymutex);
    	return(dwError);
}

DWORD DL_ApCtrl (
	void* handle,
    Byte  ucSlaveDemod,
    Bool  bOn
)
{
    DWORD dwError = Error_NO_ERROR;
	BYTE    i = 0;
	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT)handle;
	
	mutex_lock(&mymutex);

	deb_data("- Enter %s Function -",__FUNCTION__);
	deb_data("  chip =  %d  bOn = %s\n", ucSlaveDemod, bOn?"ON":"OFF");

    if(PDC->architecture != Architecture_PIP)
    {

	//	if ( PDC->modulator.chipNumber == 2 && bOn) dwError = DL_NIMSuspend(PDC, false);

		/*for (i=0; i<PDC->modulator.chipNumber; i++)
		{	 
		   if (bOn) 
			dwError = DRV_ApCtrl (PDC, i, bOn);
		   else 
			if (PDC->bTunerPowerOff != true) dwError = DRV_ApCtrl (PDC, i, bOn);

			if(!bOn)
			{
					PDC->fc[i].ulDesiredFrequency = 0;
			PDC->fc[i].ucDesiredBandWidth = 0;
			}
		}*/

	//	if(PDC->modulator.chipNumber == 2 && !bOn) dwError = DL_NIMSuspend(PDC, true);
	}
    else
    {
		if (bOn) {

			PDC->fc[ucSlaveDemod].GraphBuilt = 1;

			if (PDC->fc[0].GraphBuilt == 0 ||  PDC->fc[1].GraphBuilt == 0)
			dwError = DL_NIMSuspend(PDC, false);

			dwError = DRV_ApCtrl (PDC, ucSlaveDemod, bOn);
		} else {

			PDC->fc[ucSlaveDemod].GraphBuilt = 0;

			if (PDC->bTunerPowerOff != true) dwError = DRV_ApCtrl (PDC, ucSlaveDemod, bOn);

			if (PDC->fc[0].GraphBuilt == 0 && PDC->fc[1].GraphBuilt == 0 && PDC->bTunerPowerOff == true)
			dwError = DL_NIMSuspend(PDC, true);
		}
    }
    mutex_unlock(&mymutex);

   	return(dwError);
}


DWORD DL_Tuner_SetFreqBw(void *handle, BYTE ucSlaveDemod, u32 dwFreq,u8 ucBw)
{

	DWORD dwError = Error_NO_ERROR;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;	
	
	mutex_lock(&mymutex);

	deb_data("- Enter %s Function -\n",__FUNCTION__);
	if (PDC->fc[ucSlaveDemod].ulDesiredFrequency!=dwFreq || PDC->fc[ucSlaveDemod].ucDesiredBandWidth!=ucBw*1000) 
	 	dwError = DRV_SetFreqBw(PDC, ucSlaveDemod, dwFreq, ucBw);

    mutex_unlock(&mymutex);
    	return(dwError);	
}

/*
DWORD  DL_LoadIQtable_Fromfile(
      void *     handle
)
{
	DWORD dwError = Error_NO_ERROR;

    mutex_lock(&mymutex);

	dwError = DRV_LoadIQtable_Fromfile(handle);

    mutex_unlock(&mymutex);

    return(dwError);
}*/

DWORD DL_isLocked(void *handle, BYTE ucSlaveDemod, Bool *bLock )
{
	DWORD dwError = Error_NO_ERROR;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
    mutex_lock(&mymutex);

    deb_data("- Enter %s Function -\n",__FUNCTION__);
	
	dwError =  DRV_isLocked(PDC, ucSlaveDemod, bLock);	

    mutex_unlock(&mymutex);
	return(dwError);
}

DWORD DL_getSignalStrength(void *handle, BYTE ucSlaveDemod, BYTE* strength)
{
	DWORD dwError = Error_NO_ERROR;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
    mutex_lock(&mymutex);

    deb_data("- Enter %s Function -\n",__FUNCTION__);

    dwError =  DRV_getSignalStrength(PDC, ucSlaveDemod, strength);

//	deb_data("      The signal strength is %d \n", *strength);
    mutex_unlock(&mymutex);

        return(dwError);
}

DWORD DL_getSignalStrengthDbm(void *handle, BYTE ucSlaveDemod, Long* strengthDbm)
{
	DWORD dwError = Error_NO_ERROR;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
	mutex_lock(&mymutex);

    deb_data("- Enter %s Function -\n",__FUNCTION__);

    dwError =  DRV_getSignalStrengthDbm(PDC, ucSlaveDemod, strengthDbm);

mutex_unlock(&mymutex);
    return(dwError);
}

DWORD DL_getDeviceType(void *handle)
{
	DWORD dwError = Error_NO_ERROR;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
	mutex_lock(&mymutex);

	dwError =  DRV_getDeviceType(PDC);

	mutex_unlock(&mymutex);
    return(dwError);
}

/*
DWORD DL_getChannelStatistic(BYTE ucSlaveDemod, ChannelStatistic*	channelStatistic)
{

    mutex_lock(&mymutex);

    DWORD dwError = Error_NO_ERROR;

    deb_data("- Enter %s Function -\n",__FUNCTION__);

    dwError = DRV_getChannelStatistic(PDC, ucSlaveDemod, channelStatistic);

    mutex_unlock(&mymutex);

    return(dwError);
}
*/
DWORD DL_getChannelModulation(void *handle, BYTE ucSlaveDemod, ChannelModulation*    channelModulation)
{
	DWORD dwError = Error_NO_ERROR;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
	mutex_lock(&mymutex);

	deb_data("- Enter %s Function -\n",__FUNCTION__);

    dwError = DRV_getChannelModulation(PDC, ucSlaveDemod, channelModulation);

    mutex_unlock(&mymutex);

    return(dwError);
}

DWORD DL_getSNR(void *handle, BYTE ucSlaveDemod, Constellation* constellation, BYTE* snr)
{
	DWORD dwError = Error_NO_ERROR;
	ChannelModulation    channelModulation;
	DWORD   snr_value;
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
   	
	mutex_lock(&mymutex);

    deb_data("- Enter %s Function -\n",__FUNCTION__);

    dwError = DRV_getChannelModulation(PDC, ucSlaveDemod, &channelModulation);
    if (dwError)    return(dwError);

    dwError = DRV_getSNRValue(PDC, &snr_value);
    if (dwError)    return(dwError);

    *constellation = channelModulation.constellation;

    if(channelModulation.constellation == 0) //Constellation_QPSK 
    {
	if(snr_value < 0xB4771)    *snr = 0;
	else if(snr_value < 0xC1AED)	*snr = 1;
	else if(snr_value < 0xD0D27)   *snr = 2;
	else if(snr_value < 0xE4D19)   *snr = 3;
	else if(snr_value < 0xE5DA8)   *snr = 4;
	else if(snr_value < 0x107097)   *snr = 5;
	else if(snr_value < 0x116975)   *snr = 6;
	else if(snr_value < 0x1252D9)   *snr = 7;
	else if(snr_value < 0x131FA4)   *snr = 8;
	else if(snr_value < 0x13D5E1)   *snr = 9;
	else if(snr_value < 0x148E53)   *snr = 10;
	else if(snr_value < 0x15358B)   *snr = 11;
	else if(snr_value < 0x15DD29)   *snr = 12;
	else if(snr_value < 0x168112)   *snr = 13;
	else if(snr_value < 0x170B61)   *snr = 14;
	else if(snr_value < 0x17A532)   *snr = 15;
	else if(snr_value < 0x180F94)   *snr = 16;
	else if(snr_value < 0x186ED2)   *snr = 17;
	else if(snr_value < 0x18B271)   *snr = 18;
	else if(snr_value < 0x18E118)   *snr = 19;
	else if(snr_value < 0x18FF4B)   *snr = 20;
	else if(snr_value < 0x190AF1)   *snr = 21;
	else if(snr_value < 0x191451)   *snr = 22;
	else	*snr = 23;
    }

    else if ( channelModulation.constellation == 1) //Constellation_16QAM
    {
        if(snr_value < 0x4F0D5)    *snr = 0;
	else if(snr_value < 0x5387A)   *snr = 1;
	else if(snr_value < 0x573A4)   *snr = 2;
	else if(snr_value < 0x5A99E)   *snr = 3;
	else if(snr_value < 0x5CC80)   *snr = 4;
	else if(snr_value < 0x5EB62)   *snr = 5;
	else if(snr_value < 0x5FECF)   *snr = 6;
	else if(snr_value < 0x60B80)   *snr = 7;
	else if(snr_value < 0x62501)   *snr = 8;
	else if(snr_value < 0x64865)   *snr = 9;
	else if(snr_value < 0x69604)   *snr = 10;
	else if(snr_value < 0x6F356)   *snr = 11;
	else if(snr_value < 0x7706A)   *snr = 12;
	else if(snr_value < 0x804D3)   *snr = 13;
	else if(snr_value < 0x89D1A)   *snr = 14;
	else if(snr_value < 0x93E3D)   *snr = 15;
	else if(snr_value < 0x9E35D)   *snr = 16;
	else if(snr_value < 0xA7C3C)   *snr = 17;
	else if(snr_value < 0xAFAF8)   *snr = 18;
	else if(snr_value < 0xB719D)   *snr = 19;
	else if(snr_value < 0xBDA6A)   *snr = 20;
	else if(snr_value < 0xC0C75)   *snr = 21;
	else if(snr_value < 0xC3F7D)   *snr = 22;
	else if(snr_value < 0xC5E62)   *snr = 23;
	else if(snr_value < 0xC6C31)   *snr = 24;
	else if(snr_value < 0xC7925)   *snr = 25;
	else    *snr = 26;
    }

    else if ( channelModulation.constellation == 2) //Constellation_64QAM
    {
        if(snr_value < 0x256D0)    *snr = 0;
	else if(snr_value < 0x27A65)   *snr = 1;
	else if(snr_value < 0x29873)   *snr = 2;
	else if(snr_value < 0x2B7FE)   *snr = 3;
	else if(snr_value < 0x2CF1E)   *snr = 4;
	else if(snr_value < 0x2E234)   *snr = 5;
	else if(snr_value < 0x2F409)   *snr = 6;
	else if(snr_value < 0x30046)   *snr = 7;
	else if(snr_value < 0x30844)   *snr = 8;
	else if(snr_value < 0x30A02)   *snr = 9;
	else if(snr_value < 0x30CDE)   *snr = 10;
	else if(snr_value < 0x31031)   *snr = 11;
	else if(snr_value < 0x3144C)   *snr = 12;
	else if(snr_value < 0x315DD)   *snr = 13;
	else if(snr_value < 0x31920)   *snr = 14;
	else if(snr_value < 0x322D0)   *snr = 15;
	else if(snr_value < 0x339FC)   *snr = 16;
	else if(snr_value < 0x364A1)   *snr = 17;
	else if(snr_value < 0x38BCC)   *snr = 18;
	else if(snr_value < 0x3C7D3)   *snr = 19;
	else if(snr_value < 0x408CC)   *snr = 20;
	else if(snr_value < 0x43BED)   *snr = 21;
	else if(snr_value < 0x48061)   *snr = 22;
	else if(snr_value < 0x4BE95)   *snr = 23;
	else if(snr_value < 0x4FA7D)   *snr = 24;
	else if(snr_value < 0x52405)   *snr = 25;
	else if(snr_value < 0x5570D)   *snr = 26;
	else if(snr_value < 0x59FEB)   *snr = 27;
	else if(snr_value < 0x5BF38)   *snr = 28;
	else    *snr = 29;
    }

    else 
	deb_data("      Get constellation is failed!\n");

    mutex_unlock(&mymutex);	

    return(dwError);
}

DWORD DL_ReSetInterval(void)
{
    DWORD dwError = Error_NO_ERROR;

    mutex_lock(&mymutex);

    mutex_unlock(&mymutex);

    return(dwError);
}

DWORD DL_Reboot(void *handle) 
{
   	PDEVICE_CONTEXT PDC = (PDEVICE_CONTEXT) handle;
	DWORD dwError = Error_NO_ERROR;
    mutex_lock(&mymutex);

	deb_data("- Enter %s Function -\n",__FUNCTION__);

	dwError = DRV_Reboot(PDC);

    mutex_unlock(&mymutex);

    return(dwError);
}

DWORD DL_CheckTunerInited(
	void* handle,
	BYTE ucSlaveDemod,
	Bool *bOn )
{
	DWORD dwError = Error_NO_ERROR;
	PDEVICE_CONTEXT pdc = (PDEVICE_CONTEXT)handle;
	
    mutex_lock(&mymutex);

    deb_data("- Enter %s Function -\n",__FUNCTION__);

    *bOn = pdc->fc[ucSlaveDemod].tunerinfo.bTunerInited;

    mutex_unlock(&mymutex);

    return(dwError);
}

DWORD DL_DemodIOCTLFun(void* demodulator, DWORD IOCTLCode, unsigned long pIOBuffer)
{
    DWORD dwError = Error_NO_ERROR;
	//int error;

	mutex_lock(&mymutex);

    //deb_data("- Enter %s Function -\n",__FUNCTION__);


    dwError = DemodIOCTLFun(demodulator, IOCTLCode, pIOBuffer);

	//if (IOCTLCode == IOCTL_ITE_DEMOD_FINALIZE) {
	//	error = DL_ApCtrl(0, 0);
	//	error = DL_ApCtrl(1, 0);
	//}

    mutex_unlock(&mymutex);

    return(dwError);
}

DWORD Device_init(struct usb_device *udev, PDEVICE_CONTEXT PDC, Bool bBoot)
{
	DWORD error = Error_NO_ERROR;
	BYTE filterIdx=0;
	int errcount=0;

	PDC->modulator.userData = (void *)udev;
	dev_set_drvdata(&udev->dev, PDC);

	deb_data("- Enter %s Function -\n",__FUNCTION__);

// define in it950x-core.h
#ifdef QuantaMID
	printk("    === AfaDTV on Quanta  ===\n");
#endif
#ifdef EEEPC
	printk("    === AfaDTV on EEEPC ===\n");
#endif

#ifdef DRIVER_RELEASE_VERSION
        printk("        DRIVER_RELEASE_VERSION  : %s\n", DRIVER_RELEASE_VERSION);
#else
	printk("        DRIVER_RELEASE_VERSION  : v0.0-0\n");
#endif

#ifdef __EAGLEFIRMWARE_H__
	printk("        EAGLE_FW_RELEASE_LINK_VERSION: %X.%X.%X.%X\n", DVB_LL_VERSION1, DVB_LL_VERSION2, DVB_LL_VERSION3, DVB_LL_VERSION4);
	printk("        EAGLE_FW_RELEASE_OFDM_VERSION: %X.%X.%X.%X\n", DVB_OFDM_VERSION1, DVB_OFDM_VERSION2, DVB_OFDM_VERSION3, DVB_OFDM_VERSION4);	
#else
	printk("        EAGLE_FW_RELEASE_LINK_VERSION: v0_0_0_0\n");	
	printk("        EAGLE_FW_RELEASE_OFDM_VERSION: v0_0_0_0\n");		
#endif

#ifdef __FIRMWARE_H__
	printk("        FW_RELEASE_LINK_VERSION: %X.%X.%X.%X\n", LL_VERSION1, LL_VERSION2, LL_VERSION3, LL_VERSION4);
	printk("        FW_RELEASE_OFDM_VERSION: %X.%X.%X.%X\n", OFDM_VERSION1, OFDM_VERSION2, OFDM_VERSION3, OFDM_VERSION4);	
#else
	printk("        FW_RELEASE_LINK_VERSION: v0_0_0_0\n");	
	printk("        FW_RELEASE_OFDM_VERSION: v0_0_0_0\n");		
#endif

#ifdef __FIRMWAREV2_H__
	printk("        FW_V2_RELEASE_LINK_VERSION: %X.%X.%X.%X\n", DVB_V2_LL_VERSION1, DVB_V2_LL_VERSION2, DVB_V2_LL_VERSION3, DVB_V2_LL_VERSION4);
	printk("        FW_V2_RELEASE_OFDM_VERSION: %X.%X.%X.%X\n", DVB_V2_OFDM_VERSION1, DVB_V2_OFDM_VERSION2, DVB_V2_OFDM_VERSION3, DVB_V2_OFDM_VERSION4);	
#else
	printk("        FW_V2_RELEASE_LINK_VERSION: v0_0_0_0\n");	
	printk("        FW_V2_RELEASE_OFDM_VERSION: v0_0_0_0\n");		
#endif

#ifdef Eagle_Version_NUMBER 
	printk("        API_TX_RELEASE_VERSION  : %X.%X.%X\n", Eagle_Version_NUMBER, Eagle_Version_DATE, Eagle_Version_BUILD);
#else
	printk("        API_TX_RELEASE_VERSION  :000.00000000.0\n");
#endif

#ifdef Version_NUMBER
	printk("        API_RX_RELEASE_VERSION  : %X.%X.%X\n", Version_NUMBER, Version_DATE, Version_BUILD);
#else
	printk("        API_RX_RELEASE_VERSION  :000.00000000.0\n");
#endif


//	printk("	FW_RELEASE_VERSION : %s\n", FW_RELEASE_VERSION);
//	printk("	API_RELEASE_VERSION : %X.%X.%X\n", Version_NUMBER, Version_DATE, Version_BUILD);


	//************* Set Device init Info *************//
	PDC->bEnterSuspend = false;
    	PDC->bSurpriseRemoval = false;
    	PDC->bDevNotResp = false;
    	PDC->bSelectiveSuspend = false; 
	PDC->bTunerPowerOff = false;

	if (bBoot)
	{
		PDC->bSupportSelSuspend = false;
//		PDC->modulator.userData = (Handle)PDC;
//		PDC->modulator.chipNumber = 1; 
		PDC->demodulator.userData = (Handle)PDC;
		PDC->architecture=Architecture_DCA;
		PDC->modulator.frequency = 666000;
		PDC->modulator.bandwidth = 8000;
		PDC->bIrTblDownload = false;
		PDC->fc[0].tunerinfo.TunerId = 0;
		PDC->fc[1].tunerinfo.TunerId = 0;
		PDC->bDualTs=false;	
        	PDC->FilterCnt = 0;
		PDC->StreamType = StreamType_DVBT_DATAGRAM;
		PDC->UsbCtrlTimeOut = 1;
	}
	else {
        	PDC->UsbCtrlTimeOut = 5;
    	}//bBoot


#ifdef AFA_USB_DEVICE 	
	if (bBoot) {
		//************* Set USB Info *************//
		PDC->MaxPacketSize = 0x0200; //default USB2.0
		PDC->UsbMode = (PDC->MaxPacketSize == 0x200)?0x0200:0x0110;  
		deb_data("USB mode= 0x%x\n", PDC->UsbMode);

		PDC->TsPacketCount = (PDC->UsbMode == 0x200)?TS_PACKET_COUNT_HI:TS_PACKET_COUNT_FU;
		PDC->TsFrames = (PDC->UsbMode == 0x200)?TS_FRAMES_HI:TS_FRAMES_FU;
		PDC->TsFrameSize = TS_PACKET_SIZE*PDC->TsPacketCount;
		PDC->TsFrameSizeDw = PDC->TsFrameSize/4;
	}
	PDC->bEP12Error = false;
    	PDC->bEP45Error = false; 
    	PDC->ForceWrite = false;    
    	PDC->ulActiveFilter = 0;
#else
    	PDC->bSupportSuspend = false; 
#endif//AFA_USB_DEVICE
	
#ifdef AFA_USB_DEVICE
	if(bBoot)
    	{
		//patch for eeepc
        	//error = DL_SetBusTuner (PDC, Bus_USB, 0x38);
        	//PDC->UsbCtrlTimeOut = 5;
        
        	error = DL_SetBusTuner (PDC, Bus_USB, 0x38);
        	if (error)
        	{ 
            		deb_data("First DL_SetBusTuner fail : 0x%08ld\n",error );
			errcount++;
            		goto Exit; 
        	}

        	error =DL_GetEEPROMConfig(PDC);
        	if (error)
        	{
            		deb_data("DL_GetEEPROMConfig fail : 0x%08ld\n", error);
			errcount++;
            		goto Exit;
        	}
	}//bBoot
	
	error = DL_SetBusTuner(PDC, Bus_USB, PDC->fc[0].tunerinfo.TunerId);
	
    	if (error)
    	{
        	deb_data("DL_SetBusTuner fail!\n");
		errcount++;
        	goto Exit;
    	}

	
	 /*if (PDC->modulator.chipNumber == 2 && !PDC->modulator.booted) //plug/cold-boot/S4
    	{
        	error = DL_NIMReset(PDC);            
    	}
    	else if(PDC->modulator.chipNumber == 2 && PDC->modulator.booted) //warm-boot/(S1)
    	{
        	error = DL_NIMSuspend(PDC, false); 
		error = DL_TunerWakeup(PDC); //actually for mt2266
    	}*/
	
	
	//if(PDC->modulator.chipNumber == 1 && PDC->modulator.booted) //warm-boot/(S1)
	if(PDC->modulator.booted) //warm-boot/(S1)
	{
		error = DL_TunerWakeup(PDC);
	}
	if(error) deb_data("DL_NIMReset or DL_NIMSuspend or DL_TunerWakeup fail!\n"); 

	error = DL_Initialize(PDC);
	if (error) {
		deb_data("DL_Initialize fail! 0x%08ld\n", error);
		errcount++;
		goto Exit;
	}
	
	if (PDC->bIrTblDownload) 
    	{
        	error = DL_IrTblDownload(PDC);
       	 	if (error) {deb_data("DL_IrTblDownload fail");errcount++;}
    	}

   	 /*if (PDC->modulator.chipNumber == 2)
    	{
        	error = DL_USBSetup(PDC);
        	if (error) deb_data("DRV_SDIOSetup fail!");
    	}

    if (PDC->modulator.chipNumber == 2)
    	{
        	error = DL_InitNIMSuspendRegs(PDC);
        	if (error) deb_data("DL_InitNIMSuspendRegs fail!");
    	}	
	*/
//    for (filterIdx=0; filterIdx< PDC->modulator.chipNumber; filterIdx++) 
//    	{  
/*			filterIdx = 0;
        	if (bBoot || !PDC->fc[filterIdx].GraphBuilt)
        	{
            		error = DRV_ApCtrl(PDC, filterIdx, false);
            		if (error) {deb_data("%d: DRV_ApCtrl Fail!\n", filterIdx);errcount++;}
        	} */
//    	}        
	
/*    	if(PDC->modulator.chipNumber == 2)
    	{
       	 if(PDC->fc[0].GraphBuilt==0 && PDC->fc[1].GraphBuilt==0)
        	{
            		error = DL_NIMSuspend(PDC, true);            
            		if(error) deb_data("DL_NIMSuspend fail!");   
        	}
    	}
*/
	deb_data("	%s success!! \n",__FUNCTION__);

	/*AirHD need to init some regs only for ep6-ep4 loop back*/
/*
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, Processor_OFDM, 0xF714, 0x0);	
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD88, 0x40);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD89, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD0E, 0x80);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD0C, 0x80);
	
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD11, 0x7, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD11, 0x5, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD13, 0x7, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD13, 0x5, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD11, 0x7, 0x1, 0x1);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, Processor_LINK, 0xDD11, 0x5, 0x1, 0x1);
		
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDDA9, 0x04);

	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF714, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF731, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xD91E, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF732, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xD91F, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF730, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF778, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF73C, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF776, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF99D, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF9A4, 0x1);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD11, 0x5, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD11, 0x6, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD13, 0x5, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD13, 0x6, 0x1, 0x0);

	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD11, 0x5, 0x1, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD88, 0xE4);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD89, 0x3F);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD0C, 0x80);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD11, 0x6, 0x1, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD8A, 0xE4);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD8B, 0x3F);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD0D, 0x80);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF985, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF986, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF9A3, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF9CC, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF9CD, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF99D, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF9A4, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF9A5, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF9B5, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xD920, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF12B, 0xA);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF128, 0x9);

	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD11, 0x7, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD13, 0x7, 0x1, 0x0);
	error = IT9507_writeRegisterBits((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD11, 0x7, 0x1, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDD0E, 0x80);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_LINK, 0xDDA9, 0x16);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF721, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF723, 0x4);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF726, 0x1);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF710, 0x0);
	error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF702, 0x0);
	//error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF99A, 0x0);
*/
	//error = IT9507_writeRegister((Demodulator*) &PDC->Demodulator, 0, Processor_OFDM, 0xF7C1, 0x1);
	error = IT9507_writeRegister((Modulator*) &PDC->modulator, Processor_OFDM, 0xF7C6, 0x1);
	if(error)	printk( "AirHD Reg Write fail!\n");
	else printk( "AirHD Reg Write ok!\n");
	
	
	//error = EagleUser_getDeviceType((Modulator*) &PDC->modulator, &PDC->deviceType);
	/*if(DL_LoadIQtable_Fromfile(PDC) == Error_NO_ERROR)
		printk( "done!\n");	
	else
		printk( "fail! num: %lu\n", error);	
	*/
Exit:
#endif //AFA_USB_DEVICE
	
	if(errcount)
        printk( "[Device_init] Error %d\n", errcount);
	return (error);
}
