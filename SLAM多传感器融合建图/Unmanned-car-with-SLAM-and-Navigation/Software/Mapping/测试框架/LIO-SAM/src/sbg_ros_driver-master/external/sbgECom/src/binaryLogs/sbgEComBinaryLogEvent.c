#include "sbgEComBinaryLogEvent.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_EVENT_# message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseEvent(SbgStreamBuffer *pInputStream, SbgLogEvent *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status			= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->timeOffset0	= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->timeOffset1	= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->timeOffset2	= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->timeOffset3	= sbgStreamBufferReadUint16LE(pInputStream);
	
	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_EVENT_# message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteEvent(SbgStreamBuffer *pOutputStream, const SbgLogEvent *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->timeOffset0);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->timeOffset1);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->timeOffset2);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->timeOffset3);
	
	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
