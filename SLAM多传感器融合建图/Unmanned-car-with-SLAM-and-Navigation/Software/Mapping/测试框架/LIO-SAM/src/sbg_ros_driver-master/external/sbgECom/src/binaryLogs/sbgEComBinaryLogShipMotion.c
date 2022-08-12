#include "sbgEComBinaryLogShipMotion.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP  message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseShipMotionData(SbgStreamBuffer *pInputStream, SbgLogShipMotionData *pOutputData)
{
	assert(pInputStream);
	assert(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Read the main heave period in seconds
	//
	pOutputData->mainHeavePeriod	= sbgStreamBufferReadFloatLE(pInputStream);

	//
	// Read the surge, sway and heave ship motion
	//
	pOutputData->shipMotion[0]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->shipMotion[1]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->shipMotion[2]		= sbgStreamBufferReadFloatLE(pInputStream);
	
	//
	// Read the ship accelerations
	//
	pOutputData->shipAccel[0]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->shipAccel[1]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->shipAccel[2]		= sbgStreamBufferReadFloatLE(pInputStream);

	//
	// Test if we have a additional information such as ship velocity and status (since version 1.4)
	//
	if (sbgStreamBufferGetSpace(pInputStream) >= 14)
	{
		//
		// Read new outputs
		//
		pOutputData->shipVel[0]		= sbgStreamBufferReadFloatLE(pInputStream);
		pOutputData->shipVel[1]		= sbgStreamBufferReadFloatLE(pInputStream);
		pOutputData->shipVel[2]		= sbgStreamBufferReadFloatLE(pInputStream);

		pOutputData->status			= sbgStreamBufferReadUint16LE(pInputStream);
	}
	else
	{
		//
		// Those outputs are not available in previous versions
		//
		pOutputData->shipVel[0]		= 0.0f;
		pOutputData->shipVel[1]		= 0.0f;
		pOutputData->shipVel[2]		= 0.0f;

		pOutputData->status			= 0;
	}

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteShipMotionData(SbgStreamBuffer *pOutputStream, const SbgLogShipMotionData *pInputData)
{
	assert(pOutputStream);
	assert(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);

	//
	// Write the main heave period in seconds
	//
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->mainHeavePeriod);

	//
	// Write the surge, sway and heave ship motion
	//
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipMotion[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipMotion[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipMotion[2]);
	
	//
	// Write the ship accelerations
	//
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipAccel[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipAccel[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipAccel[2]);

	//
	// Write additional inforamtion added in version 1.4
	//
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipVel[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipVel[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->shipVel[2]);

	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
