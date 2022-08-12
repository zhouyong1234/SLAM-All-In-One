/* sbgCommonLib headers */
#include <streamBuffer/sbgStreamBuffer.h>

/* Project headers */
#include "transfer/sbgEComTransfer.h"

/* Local headers */
#include "sbgEComCmdAirData.h"

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComCmdAirDataSetModelId(SbgEComHandle *pHandle, SbgEComAirDataModelsIds modelId)
{
	assert(pHandle);

	return sbgEComCmdGenericSetModelId(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_MODEL_ID, modelId);
}

SbgErrorCode sbgEComCmdAirDataGetModelId(SbgEComHandle *pHandle, SbgEComAirDataModelsIds *pModelId)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	uint32_t		modelIdAsUint;

	assert(pHandle);
	assert(pModelId);

	errorCode = sbgEComCmdGenericGetModelId(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_MODEL_ID, &modelIdAsUint);

	if (errorCode == SBG_NO_ERROR)
	{
		*pModelId = (SbgEComAirDataModelsIds)modelIdAsUint;
	}

	return errorCode;
}

SbgErrorCode sbgEComCmdAirDataSetLeverArm(SbgEComHandle *pHandle, const float *pLeverArm)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[64];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pLeverArm);

	//
	// Create the command payload
	//
	sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

	//
	// Build payload
	//
	sbgStreamBufferWriteFloatLE(&outputStream, pLeverArm[0]);
	sbgStreamBufferWriteFloatLE(&outputStream, pLeverArm[1]);
	sbgStreamBufferWriteFloatLE(&outputStream, pLeverArm[2]);

	//
	// Make sure the payload has been build correctly
	//
	errorCode = sbgStreamBufferGetLastError(&outputStream);

	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_LEVER_ARM, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_LEVER_ARM, pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}

	return errorCode;
}

SbgErrorCode sbgEComCmdAirDataGetLeverArm(SbgEComHandle *pHandle, float *pLeverArm)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[64];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pLeverArm);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_LEVER_ARM, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_LEVER_ARM, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT command
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to parse the payload
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				pLeverArm[0] = sbgStreamBufferReadFloatLE(&inputStream);
				pLeverArm[1] = sbgStreamBufferReadFloatLE(&inputStream);
				pLeverArm[2] = sbgStreamBufferReadFloatLE(&inputStream);

				//
				// The command has been executed successfully so return if an error has occurred during payload parsing
				//
				errorCode = sbgStreamBufferGetLastError(&inputStream);
				break;
			}
		}
		else
		{
			//
			// We have a write error so exit the try loop
			//
			break;
		}
	}

	return errorCode;
}

SbgErrorCode sbgEComCmdAirDataSetRejection(SbgEComHandle *pHandle, const SbgEComAirDataRejectionConf *pRejectConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[2 * sizeof(uint8_t)];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pRejectConf);

	//
	// Create the command payload
	//
	sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

	//
	// Build payload
	//
	sbgStreamBufferWriteUint8LE(&outputStream, pRejectConf->airspeed);
	sbgStreamBufferWriteUint8LE(&outputStream, pRejectConf->altitude);

	//
	// Make sure the payload has been build correctly
	//
	errorCode = sbgStreamBufferGetLastError(&outputStream);

	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_REJECT_MODES, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_REJECT_MODES, pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}

	return errorCode;
}

SbgErrorCode sbgEComCmdAirDataGetRejection(SbgEComHandle *pHandle, SbgEComAirDataRejectionConf *pRejectConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[2 * sizeof(uint8_t)];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pRejectConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_REJECT_MODES, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIRDATA_REJECT_MODES, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_GNSS_1_REJECT_MODES command
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to parse payload
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				//
				// Parse the payload
				//
				pRejectConf->airspeed	= (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);
				pRejectConf->altitude	= (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);

				//
				// The command has been executed successfully so return if an error has occurred during payload parsing
				//
				errorCode = sbgStreamBufferGetLastError(&inputStream);
				break;
			}
		}
		else
		{
			//
			// We have a write error so exit the try loop
			//
			break;
		}
	}

	return errorCode;
}
