/* sbgCommonLib headers */
#include <sbgCommon.h>

/* Local headers */
#include "sbgInterface.h"

//----------------------------------------------------------------------//
//- Private definitions                                                -//
//----------------------------------------------------------------------//

static const char *gInterfaceType[] = 
{
	[SBG_IF_TYPE_UNKNOW]		= "unknown",
	[SBG_IF_TYPE_SERIAL]		= "serial",
	[SBG_IF_TYPE_ETH_UDP]		= "eth UDP",
	[SBG_IF_TYPE_ETH_TCP_IP]	= "eth TCP",
	[SBG_IF_TYPE_FILE]			= "file"
};

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

void sbgInterfaceZeroInit(SbgInterface *pHandle)
{
	assert(pHandle);

	//
	// Make sure the whole struct is zero init
	//
	memset(pHandle, 0x00, sizeof(*pHandle));

	//
	// Initialize all fields to default values
	//
	pHandle->handle			= NULL;
	pHandle->type			= SBG_IF_TYPE_UNKNOW;
	pHandle->name[0]		= '\0';

	pHandle->pWriteFunc		= NULL;
	pHandle->pReadFunc		= NULL;
	pHandle->pFlushFunc		= NULL;
	pHandle->pDelayFunc		= NULL;
}

const char *sbgInterfaceTypeGetAsString(const SbgInterface *pInterface)
{
	assert(pInterface);

	if (pInterface->type < SBG_ARRAY_SIZE(gInterfaceType))
	{
		return gInterfaceType[pInterface->type];
	}
	else
	{
		SBG_LOG_ERROR(SBG_INVALID_PARAMETER, "Unknown interface type: %u", pInterface->type);
		return gInterfaceType[SBG_IF_TYPE_UNKNOW];
	}
}

void sbgInterfaceNameSet(SbgInterface *pInterface, const char *pName)
{
	size_t	nameLen;

	assert(pInterface);
	assert(pName);

	//
	// Only keep the end of the name that can fit in the interface name buffer
	//
	nameLen = strlen(pName);

	if (nameLen < SBG_ARRAY_SIZE(pInterface->name))
	{
		strcpy(pInterface->name, pName);
	}
	else
	{
		strcpy(pInterface->name, pName+(nameLen-(SBG_ARRAY_SIZE(pInterface->name)-1)));
	}
}
