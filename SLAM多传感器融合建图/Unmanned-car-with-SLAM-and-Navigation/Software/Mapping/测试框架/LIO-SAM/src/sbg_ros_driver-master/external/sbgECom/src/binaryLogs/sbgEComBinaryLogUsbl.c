#include "sbgEComBinaryLogUsbl.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_USBL message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseUsblData(SbgStreamBuffer *pInputStream, SbgLogUsblData *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status				= sbgStreamBufferReadUint16LE(pInputStream);

	pOutputData->latitude			= sbgStreamBufferReadDoubleLE(pInputStream);
	pOutputData->longitude			= sbgStreamBufferReadDoubleLE(pInputStream);

	pOutputData->depth				= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->latitudeAccuracy	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->longitudeAccuracy	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->depthAccuracy		= sbgStreamBufferReadFloatLE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_USBL message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteUsblData(SbgStreamBuffer *pOutputStream, const SbgLogUsblData *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);

	sbgStreamBufferWriteDoubleLE(pOutputStream, pInputData->latitude);
	sbgStreamBufferWriteDoubleLE(pOutputStream, pInputData->longitude);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->depth);
	
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->latitudeAccuracy);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->longitudeAccuracy);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->depthAccuracy);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
