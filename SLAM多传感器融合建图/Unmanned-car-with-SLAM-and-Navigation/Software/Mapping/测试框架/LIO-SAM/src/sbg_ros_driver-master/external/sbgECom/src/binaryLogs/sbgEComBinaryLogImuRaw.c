#include "sbgEComBinaryLogImuRaw.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_IMU_RAW_DATA message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseImuRawData(SbgStreamBuffer *pInputStream, SbgLogImuRawData *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp				= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status					= sbgStreamBufferReadUint16LE(pInputStream);
				
	pOutputData->rawAccelerometers[0]	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawAccelerometers[1]	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawAccelerometers[2]	= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->rawGyroscopes[0]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawGyroscopes[1]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawGyroscopes[2]		= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->rawMagnetometers[0]	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawMagnetometers[1]	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawMagnetometers[2]	= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->rawTempAccels[0]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempAccels[1]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempAccels[2]		= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->rawTempGyros[0]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempGyros[1]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempGyros[2]		= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->rawTempMags[0]			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempMags[1]			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempMags[2]			= sbgStreamBufferReadUint32LE(pInputStream);
	
	pOutputData->rawAuxValues[0]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawAuxValues[1]		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawAuxValues[2]		= sbgStreamBufferReadUint32LE(pInputStream);

	pOutputData->rawTempAuxValues[0]	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempAuxValues[1]	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->rawTempAuxValues[2]	= sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_IMU_RAW_DATA message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteImuRawData(SbgStreamBuffer *pOutputStream, const SbgLogImuRawData *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);
				
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawAccelerometers[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawAccelerometers[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawAccelerometers[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawGyroscopes[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawGyroscopes[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawGyroscopes[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawMagnetometers[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawMagnetometers[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawMagnetometers[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempAccels[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempAccels[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempAccels[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempGyros[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempGyros[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempGyros[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempMags[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempMags[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempMags[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawAuxValues[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawAuxValues[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawAuxValues[2]);

	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempAuxValues[0]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempAuxValues[1]);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->rawTempAuxValues[2]);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
