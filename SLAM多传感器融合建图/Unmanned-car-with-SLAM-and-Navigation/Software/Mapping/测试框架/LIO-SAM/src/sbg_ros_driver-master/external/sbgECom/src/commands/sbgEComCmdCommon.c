#include "sbgEComCmdCommon.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Common command reception operations                                -//
//----------------------------------------------------------------------//

/*!
 *	Wait until any command that is not a output log is recevied during a specific time out.
 *	All binary logs received during this time are handled trough the standard callback system.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[out]	pMsgClass				Pointer used to hold the received command class.
 *	\param[out]	pMsg					Pointer used to hold the received command ID.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 *										SBG_TIME_OUT if the command hasn't been received withint the specified time out.
 */
SbgErrorCode sbgEComReceiveAnyCmd(SbgEComHandle *pHandle, uint8_t *pMsgClass, uint8_t *pMsg, void *pData, size_t *pSize, size_t maxSize, uint32_t timeOut)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgBinaryLogData	logData;
	uint8_t				receivedMsg;
	uint8_t				receivedMsgClass;
	size_t				payloadSize;
	uint8_t				payloadData[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint32_t			lastValidTime;
	
	assert(pHandle);

	//
	// Compute the last valid time according to the time out
	//
	lastValidTime = sbgGetTime() + timeOut;

	//
	// Try to receive the desired frame within the specified time out
	//
	do
	{
		//
		// Read a received frame
		//
		errorCode = sbgEComProtocolReceive(&pHandle->protocolHandle, &receivedMsgClass, &receivedMsg, payloadData, &payloadSize, sizeof(payloadData));

		//
		// Test if we have received a valid frame
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Test if the received frame is a binary log
			//
			if (sbgEComMsgClassIsALog((SbgEComClass)receivedMsgClass))
			{
				//
				// The received frame is a binary log one
				//
				errorCode = sbgEComBinaryLogParse((SbgEComClass)receivedMsgClass, receivedMsg, payloadData, payloadSize, &logData);

				//
				// Test if the incoming log has been parsed successfully
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Test if we have a valid callback to handle received logs
					//
					if (pHandle->pReceiveLogCallback)
					{
						//
						// Call the binary log callback using the new method
						//
						errorCode = pHandle->pReceiveLogCallback(pHandle, (SbgEComClass)receivedMsgClass, receivedMsg, &logData, pHandle->pUserArg);
					}
				}
				else
				{
					//
					// Call the on error callback
					//
				}
			}
			else
			{
				//
				// Return the received command
				//
				if (pMsg)
				{
					*pMsg = receivedMsg;
				}
				if (pMsgClass)
				{
					*pMsgClass = receivedMsgClass;
				}

				//
				// We have received a command so return the payload size
				//
				if (pSize)
				{
					*pSize = payloadSize;
				}

				//
				// Test if we have a payload to return
				//
				if (payloadSize > 0)
				{
					//
					// Make sure that the payload can be stored and fit in the destination buffer
					//
					if ( (pData) && (payloadSize <= maxSize) )
					{
						//
						// Copy the payload
						//
						memcpy(pData, payloadData, payloadSize);
					}
					else
					{
						//
						// We have a buffer overflow
						//
						return SBG_BUFFER_OVERFLOW;
					}
				}

				//
				// We have received the frame we are looking for so return
				//
				return SBG_NO_ERROR;
			}
		}
		else if (errorCode == SBG_NOT_READY)
		{
			//
			// No more data are present in the reception buffer so release some CPU before the next try
			//
			sbgSleep(1);
		}
	} while (lastValidTime >= sbgGetTime());

	//
	// The time out has expired so return time out error
	//
	errorCode = SBG_TIME_OUT;
	
	return errorCode;
}

/*!
 *	Wait for a specific command to be received given a time out.								<br>
 *	All binary logs received during this time are handled trough the standard callback system.	<br>
 *	NACK for the requested command is treated as a negative answer and the corresponding error code is returned to user. In case an ACK is returned, we condider it's an error as well as expected command is not an ACK but regular command.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[out]	msgClass				Command class we expect to receive
 *	\param[out]	msg						Message id we expect to receive
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										Other error codes may come from the function itself, or from the device returning an NACK.
 *										
 */
SbgErrorCode sbgEComReceiveCmd(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, void *pData, size_t *pSize, size_t maxSize, uint32_t timeOut)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgErrorCode		ackErrorCode;
	uint8_t				receivedMsg;
	uint8_t				receivedMsgClass;
	uint8_t				ackMsg;
	uint8_t				ackMsgClass;
	uint32_t			lastValidTime;
	SbgStreamBuffer		inputStream;
	
	assert(pHandle);

	//
	// Compute the last valid time according to the time out
	//
	lastValidTime = sbgGetTime() + timeOut;

	//
	// Try to receive the desired frame within the specified time out
	//
	do
	{
		//
		// Read a received frame - no timeout as this is handled in this function
		// Content is directly copied to user parameters in order to avoid multiple data copy
		//
		errorCode = sbgEComReceiveAnyCmd(pHandle, &receivedMsgClass, &receivedMsg, pData, pSize, maxSize, 0);

		//
		// Test if we have received a valid frame
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Test if the received frame the requested one
			//
			if ((receivedMsgClass == msgClass) && (receivedMsg == msg))
			{
				//
				// We have received the frame we are looking for so return
				// Output parameters are already filled by sbgEComReceiveAnyCmd
				//
				return SBG_NO_ERROR;
			}
			else if ((receivedMsgClass == SBG_ECOM_CLASS_LOG_CMD_0) && (receivedMsg == SBG_ECOM_CMD_ACK))
			{
				//
				// We have received an ACK frame. 
				// We can now parse the message and check if this gives an negative answer to our expected command
				//
				sbgStreamBufferInitForRead(&inputStream, pData, *pSize);
				ackMsg			= sbgStreamBufferReadUint8LE(&inputStream);
				ackMsgClass		= sbgStreamBufferReadUint8LE(&inputStream);
				ackErrorCode	= (SbgErrorCode)sbgStreamBufferReadUint16LE(&inputStream);

				//
				// Check if the ACK corresponds to the frame we expected
				//
				if ((ackMsg == msg) && (ackMsgClass == msgClass))
				{
					//
					// return the error code if this is a negative one. in case of NO_ERROR, return SBG_ERROR instead
					//
					if (ackErrorCode != SBG_NO_ERROR)
					{
						return ackErrorCode;
					}
					else
					{
						return SBG_ERROR;
					}
				}
			}
		}
		else if (errorCode == SBG_NOT_READY)
		{
			//
			// No more data are present in the reception buffer so release some CPU before the next try
			//
			sbgSleep(1);
		}
	} while (lastValidTime >= sbgGetTime());

	//
	// The time out has expired so return time out error
	//
	errorCode = SBG_TIME_OUT;

	return errorCode;
}

//----------------------------------------------------------------------//
//- ACK related commands  operations                                   -//
//----------------------------------------------------------------------//

/*!
 * Wait for an ACK for a specified amount of time.
 *\param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	msgClass					The message class that we want to check
 * \param[in]	msg							The message ID that we want to check
 * \param[in]	timeOut						Time out in ms during which we can receive the ACK.
 * \return									SBG_NO_ERROR if the ACK has been received.
 */
SbgErrorCode sbgEComWaitForAck(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, uint32_t timeOut)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint8_t				payload[2*sizeof(uint16_t)];
	SbgStreamBuffer		inputStream;
	size_t				receivedSize;
	uint8_t				ackClass;
	uint8_t				ackMsg;

	assert(pHandle);

	//
	// Try to receive the ACK
	//
	errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ACK, payload, &receivedSize, sizeof(payload), timeOut);

	//
	// Test if an ACK frame has been received
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Validate the received ACK frame
		//
		if (receivedSize == 2*sizeof(uint16_t))
		{
			//
			// Initialize a stream buffer to parse the received payload
			//
			sbgStreamBufferInitForRead(&inputStream, payload, sizeof(payload));

			//
			// The ACK frame contains the ack message ID and class, and a uint16_t for the return error code
			// We make sure that the ACK is for the correct command
			//
			ackMsg		= sbgStreamBufferReadUint8LE(&inputStream);
			ackClass	= sbgStreamBufferReadUint8LE(&inputStream);

			if ((ackMsg == msg) && (ackClass == msgClass))
			{
				//
				// Parse the error code and return it
				//
				errorCode = (SbgErrorCode)sbgStreamBufferReadUint16LE(&inputStream);
			}
			else
			{
				//
				// We have received an ACK but not for this frame!
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
		else
		{
			//
			// The ACK is invalid
			//
			errorCode = SBG_INVALID_FRAME;
		}
	}	

	return errorCode;
}

/*!
 * Send an ACK for a specific command with an associated error code.
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	msgClass					The message class that we want to send
 * \param[in]	msg							The message ID that we want to send.
 * \param[in]	cmdError					The associated error code.
 * \return									SBG_NO_ERROR if the ACK has been sent.
 */
SbgErrorCode sbgEComSendAck(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, SbgErrorCode cmdError)
{
	SbgStreamBuffer 		outputStream;
	uint8_t 		 		payload[2*sizeof(uint8_t)+sizeof(uint16_t)];
	
	assert(pHandle);

	//
	// Initialize a stream buffer to write the command payload
	//
	sbgStreamBufferInitForWrite(&outputStream, payload, sizeof(payload));

	//
	// Write the message ID and class and then the error code
	//
	sbgStreamBufferWriteUint8LE(&outputStream, msg);
	sbgStreamBufferWriteUint8LE(&outputStream, msgClass);
	sbgStreamBufferWriteUint16LE(&outputStream, (uint16_t)cmdError);

	//
	// Send the ACK command
	//
	return sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ACK, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));
}

//----------------------------------------------------------------------//
//- Generic command definitions                                        -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComCmdGenericSetModelId(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, uint32_t modelId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[sizeof(uint32_t)];
	SbgStreamBuffer		outputStream;
	
	assert(pHandle);
	
	//
	// Init stream buffer for output and Build payload
	//
	sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
	sbgStreamBufferWriteUint32LE(&outputStream, modelId);

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
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, msgClass, msg, pHandle->cmdDefaultTimeOut);

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

SbgErrorCode sbgEComCmdGenericGetModelId(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, uint32_t *pModelId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[sizeof(uint32_t)];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pModelId);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, msgClass, msg, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a the specified command
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
				*pModelId = sbgStreamBufferReadUint32LE(&inputStream);

				//
				// The command has been executed successfully so return
				// We return the stream buffer error code to catch any overflow error on the payload
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

SbgErrorCode sbgEComCmdGenericGetModelInfo(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, SbgEComModelInfo *pModelInfo)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[2*sizeof(uint32_t)];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pModelInfo);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, msgClass, msg, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a the specified command
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
				pModelInfo->id			= sbgStreamBufferReadUint32LE(&inputStream);
				pModelInfo->revision	= sbgStreamBufferReadUint32LE(&inputStream);

				//
				// The command has been executed successfully so return
				// We return the stream buffer error code to catch any overflow error on the payload
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
