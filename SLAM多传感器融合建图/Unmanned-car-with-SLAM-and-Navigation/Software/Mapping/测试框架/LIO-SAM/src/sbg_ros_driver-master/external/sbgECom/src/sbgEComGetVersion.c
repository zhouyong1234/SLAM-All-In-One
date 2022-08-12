// sbgCommonLib headers
#include <sbgCommon.h>
#include <version/sbgVersion.h>

// Project headers
#include "sbgEComVersion.h"

// Local headers
#include "sbgEComGetVersion.h"

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

uint32_t sbgEComGetVersion(void)
{
	return SBG_E_COM_VERSION;
}

const char *sbgEComGetVersionAsString(void)
{
	return SBG_E_COM_VERSION_STR;
}
