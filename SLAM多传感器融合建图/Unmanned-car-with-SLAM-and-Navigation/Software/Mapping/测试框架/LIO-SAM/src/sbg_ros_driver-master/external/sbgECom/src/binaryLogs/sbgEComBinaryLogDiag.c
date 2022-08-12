// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Local headers
#include "sbgEComBinaryLogDiag.h"

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComBinaryLogParseDiagData(SbgStreamBuffer *pInputStream, SbgLogDiagData *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	pOutputData->timestamp		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->type			= (SbgDebugLogType)sbgStreamBufferReadUint8(pInputStream);
	pOutputData->errorCode		= (SbgErrorCode)sbgStreamBufferReadUint8(pInputStream);

	sbgStreamBufferReadBuffer(pInputStream, pOutputData->string, sbgStreamBufferGetSpace(pInputStream));
	pOutputData->string[sizeof(pOutputData->string) - 1] = '\0';

	return sbgStreamBufferGetLastError(pInputStream);
}

SbgErrorCode sbgEComBinaryLogWriteDiagData(SbgStreamBuffer *pOutputStream, const SbgLogDiagData *pInputData)
{
	size_t								 length;

	assert(pOutputStream);
	assert(pInputData);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timestamp);
	sbgStreamBufferWriteUint8(pOutputStream, pInputData->type);
	sbgStreamBufferWriteUint8(pOutputStream, pInputData->errorCode);

	length = strlen(pInputData->string);

	if (length >= sizeof(pInputData->string))
	{
		length = sizeof(pInputData->string) - 1;
	}

	sbgStreamBufferWriteBuffer(pOutputStream, pInputData->string, length);
	sbgStreamBufferWriteUint8(pOutputStream, 0);

	return sbgStreamBufferGetLastError(pOutputStream);
}
