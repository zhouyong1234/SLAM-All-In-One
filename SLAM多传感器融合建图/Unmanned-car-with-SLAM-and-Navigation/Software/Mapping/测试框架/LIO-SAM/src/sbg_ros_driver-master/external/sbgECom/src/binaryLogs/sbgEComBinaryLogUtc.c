#include "sbgEComBinaryLogUtc.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_UTC_DATA message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseUtcData(SbgStreamBuffer *pInputStream, SbgLogUtcData *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status			= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->year			= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->month			= sbgStreamBufferReadInt8LE(pInputStream);
	pOutputData->day			= sbgStreamBufferReadInt8LE(pInputStream);
	pOutputData->hour			= sbgStreamBufferReadInt8LE(pInputStream);
	pOutputData->minute			= sbgStreamBufferReadInt8LE(pInputStream);
	pOutputData->second			= sbgStreamBufferReadInt8LE(pInputStream);
	pOutputData->nanoSecond		= sbgStreamBufferReadInt32LE(pInputStream);
	pOutputData->gpsTimeOfWeek	= sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_UTC_DATA message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteUtcData(SbgStreamBuffer *pOutputStream, const SbgLogUtcData *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->year);
	sbgStreamBufferWriteInt8LE(pOutputStream, pInputData->month);
	sbgStreamBufferWriteInt8LE(pOutputStream, pInputData->day);
	sbgStreamBufferWriteInt8LE(pOutputStream, pInputData->hour);
	sbgStreamBufferWriteInt8LE(pOutputStream, pInputData->minute);
	sbgStreamBufferWriteInt8LE(pOutputStream, pInputData->second);
	sbgStreamBufferWriteInt32LE(pOutputStream, pInputData->nanoSecond);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->gpsTimeOfWeek);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
