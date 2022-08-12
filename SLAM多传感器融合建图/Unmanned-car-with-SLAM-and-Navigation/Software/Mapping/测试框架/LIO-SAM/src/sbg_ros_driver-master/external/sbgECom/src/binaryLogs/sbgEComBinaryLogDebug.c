#include "sbgEComBinaryLogDebug.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for SBG_ECOM_LOG_DEBUG_X messages and fill the corresponding structure.
 *	\param[in]	pInputStream				Input stream buffer to read the payload from.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDebugData(SbgStreamBuffer *pInputStream, SbgLogDebugData *pOutputData)
{
	SbgErrorCode	 errorCode;

	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload and return
	//
	pOutputData->id			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->offset		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->size		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->totalSize	= sbgStreamBufferReadUint32LE(pInputStream);

	errorCode = sbgStreamBufferGetLastError(pInputStream);

	if (errorCode == SBG_NO_ERROR)
	{
		if (pOutputData->size <= SBG_ARRAY_SIZE(pOutputData->data))
		{
			sbgStreamBufferReadBuffer(pInputStream, pOutputData->data, pOutputData->size);
			errorCode = sbgStreamBufferGetLastError(pInputStream);
		}
		else
		{
			errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	return errorCode;
}

/*!
 * Write data for SBG_ECOM_LOG_DEBUG_X messages to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDebugData(SbgStreamBuffer *pOutputStream, const SbgLogDebugData *pInputData)
{
	SbgErrorCode	 errorCode;

	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->id);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->offset);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->size);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->totalSize);

	if (pInputData->size <= SBG_ARRAY_SIZE(pInputData->data))
	{
		sbgStreamBufferWriteBuffer(pOutputStream, pInputData->data, pInputData->size);
		errorCode = sbgStreamBufferGetLastError(pOutputStream);
	}
	else
	{
		errorCode = SBG_BUFFER_OVERFLOW;
	}

	return errorCode;
}
