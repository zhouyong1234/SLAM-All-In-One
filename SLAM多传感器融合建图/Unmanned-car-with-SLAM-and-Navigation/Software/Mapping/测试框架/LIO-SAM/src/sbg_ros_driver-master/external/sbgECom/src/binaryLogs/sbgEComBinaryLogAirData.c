#include "sbgEComBinaryLogAirData.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComBinaryLogParseAirData(SbgStreamBuffer *pInputStream, SbgLogAirData *pOutputData)
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
	// The true airspeed fields have been added in version 2.0
	//
	if (sbgStreamBufferGetSpace(pInputStream) > 0)
	{
		pOutputData->pressureDiff	= sbgStreamBufferReadFloatLE(pInputStream);
		pOutputData->trueAirspeed	= sbgStreamBufferReadFloatLE(pInputStream);
		pOutputData->airTemperature	= sbgStreamBufferReadFloatLE(pInputStream);
	}
	else
	{
		pOutputData->pressureDiff	= 0.0f;
		pOutputData->trueAirspeed	= 0.0f;
		pOutputData->airTemperature = 0.0f;
	}

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

SbgErrorCode sbgEComBinaryLogWriteAirData(SbgStreamBuffer *pOutputStream, const SbgLogAirData *pInputData)
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

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->pressureDiff);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->trueAirspeed);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->airTemperature);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
