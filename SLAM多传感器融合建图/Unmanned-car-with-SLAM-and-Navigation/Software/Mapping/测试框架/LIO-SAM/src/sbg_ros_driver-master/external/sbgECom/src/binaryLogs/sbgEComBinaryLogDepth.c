#include "sbgEComBinaryLogDepth.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComBinaryLogParseDepth(SbgStreamBuffer *pInputStream, SbgLogDepth *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status			= sbgStreamBufferReadUint16LE(pInputStream);

	pOutputData->pressureAbs	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->altitude		= sbgStreamBufferReadFloatLE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

SbgErrorCode sbgEComBinaryLogWriteDepth(SbgStreamBuffer *pOutputStream, const SbgLogDepth *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->pressureAbs);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->altitude);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
