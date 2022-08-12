#include "sbgEComBinaryLogDvl.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK / SBG_ECOM_LOG_DVL_WATER_TRACK message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDvlData(SbgStreamBuffer *pInputStream, SbgLogDvlData *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status				= sbgStreamBufferReadUint16LE(pInputStream);

	pOutputData->velocity[0]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocity[1]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocity[2]		= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->velocityQuality[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocityQuality[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->velocityQuality[2]	= sbgStreamBufferReadFloatLE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK / SBG_ECOM_LOG_DVL_WATER_TRACK message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDvlData(SbgStreamBuffer *pOutputStream, const SbgLogDvlData *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocity[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocity[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocity[2]);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocityQuality[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocityQuality[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->velocityQuality[2]);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
