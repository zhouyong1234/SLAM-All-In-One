#include "sbgEComCmdInfo.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Info commands		                                               -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the device information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pInfo						A pointer to a structure to hold device information.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGetInfo(SbgEComHandle *pHandle, SbgEComDeviceInfo *pInfo)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pInfo);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have correctly received a message
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Make sure we have received a payload
				//
				if (receivedSize > 0)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					//
					sbgStreamBufferReadBuffer(&inputStream, pInfo->productCode, SBG_ECOM_INFO_PRODUCT_CODE_LENGTH);
					pInfo->serialNumber		= sbgStreamBufferReadUint32LE(&inputStream);
					pInfo->calibationRev	= sbgStreamBufferReadUint32LE(&inputStream);
					pInfo->calibrationYear	= sbgStreamBufferReadUint16LE(&inputStream);
					pInfo->calibrationMonth	= sbgStreamBufferReadUint8LE(&inputStream);
					pInfo->calibrationDay	= sbgStreamBufferReadUint8LE(&inputStream);
					pInfo->hardwareRev		= sbgStreamBufferReadUint32LE(&inputStream);
					pInfo->firmwareRev		= sbgStreamBufferReadUint32LE(&inputStream);

					//
					// We have parsed a message so return immediately but report any error during payload parsing
					//
					errorCode = sbgStreamBufferGetLastError(&inputStream);

					break;
				}
				else
				{
					//
					// We should have received a non empty payload so we have received an invalid frame
					//
					errorCode = SBG_INVALID_FRAME;
				}
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
