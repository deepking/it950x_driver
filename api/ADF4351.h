#include "IT9507.h"

Dword ADF4351_busInit (Modulator*    modulator);
Dword ADF4351_writeData (
		Modulator*  modulator,
		Byte		count,
		Byte*		buf		
);

Dword ADF4351_test (
		Modulator*  modulator		
);

Dword ADF4351_setFrequency (
		Modulator*  modulator,
		Dword freq_KHz	//KHz
);