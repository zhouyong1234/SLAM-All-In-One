#include "sbgEComCmdOutput.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Output commands		                                               -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[in]	classId						The class of the concerned log.
 *	\param[in]	msgId						The id of the concerned log.
 *	\param[out]	pConf						Pointer to a SbgEComOutputMode to contain the current output mode of the message.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputGetConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComClass classId, SbgEComMsgId msgId, SbgEComOutputMode *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{

			//
			// Initialize output stream buffer
			//
			sbgStreamBufferInitForWrite(&outputStream, &outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)outputPort);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)msgId);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)classId);

			//
			// Send the command and the prepared payload
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a SBG_ECOM_CMD_OUTPUT_CONF command
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// First is returned outputPort, then messageId, classId rate and the output configuration at last.
					//
					outputPort = (SbgEComOutputPort)sbgStreamBufferReadUint8LE(&inputStream);
					msgId = sbgStreamBufferReadUint8LE(&inputStream);
					classId = (SbgEComClass)sbgStreamBufferReadUint8LE(&inputStream);
					*pConf = (SbgEComOutputMode)sbgStreamBufferReadUint16LE(&inputStream);

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
	else
	{
		//
		// Null pointer.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Set the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[in]	classId						The class of the concerned log.
 *	\param[in]	msgId						The id of the concerned log.
 *	\param[in]	conf						New output mode to set.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputSetConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComClass classId, SbgEComMsgId msgId, SbgEComOutputMode conf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if (pHandle)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)outputPort);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)msgId);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)classId);
			sbgStreamBufferWriteUint16LE(&outputStream, (uint16_t)conf);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CONF, pHandle->cmdDefaultTimeOut);

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
	else
	{
		//
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Retrieve the enable of one of the output class message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port.
 *	\param[in]	classId						The class to enable or disable.
 *	\param[out]	pEnable						TRUE to enable message output of this class, FALSE to disable it.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputClassGetEnable(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComClass classId, bool *pEnable)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pEnable))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{

			//
			// Initialize output stream buffer
			//
			sbgStreamBufferInitForWrite(&outputStream, &outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)outputPort);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)classId);

			//
			// Send the command and the prepared payload
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CLASS_ENABLE, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CLASS_ENABLE, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a correct answer
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// First is returned outputPort, then messageId, classId rate and the output configuration at last.
					//
					outputPort = (SbgEComOutputPort)sbgStreamBufferReadUint8LE(&inputStream);
					classId = (SbgEComClass)sbgStreamBufferReadUint8LE(&inputStream);
					*pEnable = (bool)sbgStreamBufferReadUint8LE(&inputStream);

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
	else
	{
		//
		// Null pointer.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Set the enable of one of the output class message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port.
 *	\param[in]	classId						The class to enable or disable.
 *	\param[in]	enable						TRUE to enable message output of this class, FALSE to disable it.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputClassSetEnable(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComClass classId, bool enable)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if (pHandle)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)outputPort);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)classId);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)enable);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CLASS_ENABLE, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_OUTPUT_CLASS_ENABLE, pHandle->cmdDefaultTimeOut);

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
	else
	{
		//
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Retrieve the configuration of one the message on the CAN interface.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	internalId					The internal message id.
 *	\param[out]	pMode						Pointer to a SbgEComOutputMode to contain the current output mode of the message.
 *	\param[out]	pUserId						The user defined message id.
 *	\param[out]	pExtended					TRUE if the user id uses the extended format.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdCanOutputGetConf(SbgEComHandle *pHandle, SbgECanMessageId internalId, SbgEComOutputMode *pMode, uint32_t *pUserId, bool *pExtended)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	uint8_t				outputBuffer[2];
	SbgStreamBuffer		outputStream;
	SbgStreamBuffer		inputStream;
	
	assert(pHandle);
	assert(pMode);
	assert(pUserId);
	assert(pExtended);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command and a 2-byte payload
		//
		sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
		sbgStreamBufferWriteUint16LE(&outputStream, internalId);
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_OUTPUT_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_OUTPUT_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a correct answer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to read parameters
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				//
				// Read parameters
				// First is returned outputPort, then messageId, classId rate and the output configuration at last.
				//
				internalId = (SbgECanMessageId)sbgStreamBufferReadUint16LE(&inputStream);
				*pMode = (SbgEComOutputMode)sbgStreamBufferReadUint16LE(&inputStream);
				*pUserId = sbgStreamBufferReadUint32LE(&inputStream);
				*pExtended = (bool)sbgStreamBufferReadUint8LE(&inputStream);
					
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
	
	return errorCode;
}

/*!
 *	Set the configuration of one the message on the CAN interface
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	internalId					The internal message id.
 *	\param[in]	pMode						Pointer to a SbgEComOutputMode containing the new output mode of the message.
 *	\param[in]	pUserId						The user defined message id.
 *	\param[in]	pExtended					TRUE if the user id uses the extended format.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdCanOutputSetConf(SbgEComHandle *pHandle, SbgECanMessageId internalId, SbgEComOutputMode mode, uint32_t userId, bool extended)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	assert(pHandle);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Init stream buffer for output
		//
		sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

		//
		// Build payload
		//
		sbgStreamBufferWriteUint16LE(&outputStream, (uint16_t)internalId);
		sbgStreamBufferWriteUint16LE(&outputStream, (uint16_t)mode);
		sbgStreamBufferWriteUint32LE(&outputStream, userId);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)extended);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_OUTPUT_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_OUTPUT_CONF, pHandle->cmdDefaultTimeOut);

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

	return errorCode;
}

/*!
 *	Retrieve the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *  \param[out] pConf						Pointer to a SbgEComLegacyConf structure to contain legacy configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputGetLegacyConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, SbgEComLegacyConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	uint8_t				outputBuffer[1];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Initialize output stream buffer
		//
		sbgStreamBufferInitForWrite(&outputStream, &outputBuffer, sizeof(outputBuffer));

		//
		// Build payload
		//
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)outputPort);

		//
		// Send the command and the prepared payload
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_LEGACY_CONT_OUTPUT_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_LEGACY_CONT_OUTPUT_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a correct command
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to read parameters
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				//
				// Read parameters
				// First is returned outputPort, then messageId, classId rate and the output configuration at last.
				//
				outputPort = (SbgEComOutputPort)sbgStreamBufferReadUint8LE(&inputStream);
				pConf->mask = sbgStreamBufferReadUint32LE(&inputStream);
				pConf->format = (SbgEComLegacyFormat)sbgStreamBufferReadUint8LE(&inputStream);
				pConf->endian = (SbgEComLegacyEndian)sbgStreamBufferReadUint8LE(&inputStream);
				pConf->mode = (SbgEComOutputMode)sbgStreamBufferReadUint16LE(&inputStream);

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
	
	return errorCode;
}

/*!
 *	Set the configuration of one the message on one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *  \param[in]  pConf						Pointer to a SbgEComLegacyConf structure containing new legacy configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputSetLegacyConf(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, const SbgEComLegacyConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[9];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Init stream buffer for output
		//
		sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

		//
		// Build payload
		//
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)outputPort);
		sbgStreamBufferWriteUint32LE(&outputStream, pConf->mask);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->format);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->endian);
		sbgStreamBufferWriteUint16LE(&outputStream, (uint16_t)pConf->mode);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_LEGACY_CONT_OUTPUT_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_LEGACY_CONT_OUTPUT_CONF, pHandle->cmdDefaultTimeOut);

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
	
	return errorCode;
}

/*!
 *	Retrieve the NMEA talker id of one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[out]	nmeaTalkerId				A 2-char array to contain the nmea talker id.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputGetNmeaTalkerId(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, char nmeaTalkerId[2])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	uint8_t				outputBuffer[1];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(nmeaTalkerId);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command with the output port as a 1-byte payload
		//
		sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
		sbgStreamBufferWriteUint8(&outputStream, outputPort);
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_NMEA_TALKER_ID, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_NMEA_TALKER_ID, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_NMEA_TALKER_ID command
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to read parameters
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				//
				// Read parameters
				//
				outputPort = (SbgEComOutputPort)sbgStreamBufferReadUint8LE(&inputStream);
				nmeaTalkerId[0] = (char)sbgStreamBufferReadUint8LE(&inputStream);
				nmeaTalkerId[1] = (char)sbgStreamBufferReadUint8LE(&inputStream);

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
	
	return errorCode;
}

/*!
 *	Set the NMEA talker id of one of the output interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	outputPort					The output port of the device for the log concerned.
 *	\param[out]	nmeaTalkerId				A 2-char array containint the new nmea talker id.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOutputSetNmeaTalkerId(SbgEComHandle *pHandle, SbgEComOutputPort outputPort, const char nmeaTalkerId[2])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[3];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(nmeaTalkerId);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Init stream buffer for output
		//
		sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

		//
		// Build payload
		//
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)outputPort);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)(nmeaTalkerId[0]));
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)(nmeaTalkerId[1]));

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_NMEA_TALKER_ID, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_NMEA_TALKER_ID, pHandle->cmdDefaultTimeOut);

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
	
	return errorCode;
}
