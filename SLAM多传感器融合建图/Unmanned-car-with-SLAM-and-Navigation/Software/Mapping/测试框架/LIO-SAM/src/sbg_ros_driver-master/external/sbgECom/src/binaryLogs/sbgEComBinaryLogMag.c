#include "sbgEComBinaryLogMag.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_MAG message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagData(SbgStreamBuffer *pInputStream, SbgLogMag *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status				= sbgStreamBufferReadUint16LE(pInputStream);

	pOutputData->magnetometers[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->magnetometers[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->magnetometers[2]	= sbgStreamBufferReadFloatLE(pInputStream);
				
	pOutputData->accelerometers[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->accelerometers[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->accelerometers[2]	= sbgStreamBufferReadFloatLE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_MAG message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteMagData(SbgStreamBuffer *pOutputStream, const SbgLogMag *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);
	
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->magnetometers[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->magnetometers[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->magnetometers[2]);
		
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->accelerometers[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->accelerometers[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->accelerometers[2]);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}

/*!
 * Parse data for the SBG_ECOM_LOG_MAG_CALIB message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagCalibData(SbgStreamBuffer *pInputStream, SbgLogMagCalib *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->reserved	= sbgStreamBufferReadUint16LE(pInputStream);

	//
	// Read the raw magnetic calibration data buffer
	//
	return sbgStreamBufferReadBuffer(pInputStream, pOutputData->magData, sizeof(pOutputData->magData));
}

/*!
 * Write data for the SBG_ECOM_LOG_MAG_CALIB message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteMagCalibData(SbgStreamBuffer *pOutputStream, const SbgLogMagCalib *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->reserved);

	//
	// Write the raw magnetic calibration data buffer
	//
	return sbgStreamBufferWriteBuffer(pOutputStream, pInputData->magData, sizeof(pInputData->magData));
}
