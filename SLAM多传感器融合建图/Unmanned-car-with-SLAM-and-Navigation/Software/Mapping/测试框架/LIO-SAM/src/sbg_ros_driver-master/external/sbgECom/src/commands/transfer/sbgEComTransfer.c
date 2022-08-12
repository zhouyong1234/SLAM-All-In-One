#include "sbgEComTransfer.h"
#include "../sbgEComCmdCommon.h"
#include <streamBuffer/sbgStreamBuffer.h>
#include <splitBuffer/sbgSplitBuffer.h>

//----------------------------------------------------------------------//
//- Internal transfer method definitions			                   -//
//----------------------------------------------------------------------//

/*!
 * Initiates an upload transfer sequence with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] msgClass					Original protocol class asking for transfer.
 * \param[in] msg						Original protocol message id asking for transfer.
 * \param[in] size						Total size of the upload.
 * \return								SBG_NO_ERROR when the transfer was initiated successfully.
 */
static SbgErrorCode sbgEComTransferSendInit(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, size_t size)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		streamBuffer;
	uint8_t				outputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint32_t			i;

	assert(pHandle);

	//
	// Initialize stream buffer that will contain payload
	//
	sbgStreamBufferInitForWrite(&streamBuffer, outputBuffer, sizeof(outputBuffer));

	//
	// Build transfer payload (a SBG_ECOM_TRANSFER_START command and the total size of the upload)
	//
	sbgStreamBufferWriteUint16LE(&streamBuffer, SBG_ECOM_TRANSFER_START);
	sbgStreamBufferWriteSizeT32LE(&streamBuffer, size);

	//
	// Send command (multiple times in case of failures)
	//
	for (i = 0; i < 3; i++)
	{
		//
		// Send transfer payload encapsulated in ECom protocol
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, sbgStreamBufferGetLinkedBuffer(&streamBuffer), sbgStreamBufferGetLength(&streamBuffer));
		
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// If the device accepts the transfer, it returns an ack, wait for the answer.
			//
			errorCode = sbgEComWaitForAck(pHandle, msgClass, msg, pHandle->cmdDefaultTimeOut);

			//
			// Test if the response is positive from device
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Ack received, no need for other trial.
				//
				break;
			}
		}
	}
	
	return errorCode;
}

/*!
 * Send one packet of data on a initiated upload transfer.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] msgClass					Original protocol class asking for transfer.
 * \param[in] msg						Original protocol message id asking for transfer.
 * \param[in] pBuffer					Pointer to the buffer containing the data to send.
 * \param[in] offset					The offset from the start of the transfer.
 * \param[in] packetSize				The size of this packet.
 * \return								SBG_NO_ERROR if the packet was sent and acknowledged by the device.
 */
static SbgErrorCode sbgEComTransferSendData(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, const void *pBuffer, size_t offset, size_t packetSize)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		streamBuffer;
	uint8_t				outputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint32_t			i;
	
	assert(pHandle);
	assert(pBuffer);
	assert(packetSize > 0);

	//
	// Initialize stream buffer for output
	//
	sbgStreamBufferInitForWrite(&streamBuffer, outputBuffer, sizeof(outputBuffer));

	//
	// Build payload: a SBG_ECOM_TRANSFER_DATA command, the offset from the start of the transfer, and the data
	//
	sbgStreamBufferWriteUint16LE(&streamBuffer, SBG_ECOM_TRANSFER_DATA);
	sbgStreamBufferWriteSizeT32LE(&streamBuffer, offset);
	sbgStreamBufferWriteBuffer(&streamBuffer, pBuffer, packetSize);

	//
	// Send command (multiple times in case of failures)
	//
	for (i = 0; i < 3; i++)
	{
		//
		// Send transfer payload encapsulated in a ECom protocol frame
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, sbgStreamBufferGetLinkedBuffer(&streamBuffer), sbgStreamBufferGetLength(&streamBuffer));
		
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// If the device receives the frame successfully received, it responds with an ACK, wait for the answer
			//
			errorCode = sbgEComWaitForAck(pHandle, msgClass, msg, pHandle->cmdDefaultTimeOut);

			//
			// Test if the response is positive from device
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Ack received, no need for other trial
				//
				break;
			}
		}
	}

	return errorCode;
}

/*!
 * Ends ongoing upload transfer sequence with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] msgClass					Original protocol class asking for transfer.
 * \param[in] msg						Original protocol message id asking for transfer.
 * \return								SBG_NO_ERROR when the transfer ended successfully.
 */
static SbgErrorCode sbgEComTransferSendEnd(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	uint8_t				outputBuffer[sizeof(uint16_t)];
	uint32_t			i;

	assert(pHandle);
	
	//
	// Initialize stream buffer for output
	//
	sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outStreamBuffer));

	//
	// Build payload, only a SBG_ECOM_TRANSFER_END cmd
	//
	sbgStreamBufferWriteUint16LE(&outStreamBuffer, SBG_ECOM_TRANSFER_END);
		
	//
	// Send command (multiple times in case of failures)
	//
	for (i = 0; i < 3; i++)
	{
		//
		// Send upload end payload encapsulated in a ECom protocol frame
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));

		if (errorCode == SBG_NO_ERROR)
		{
			//
			// If the device finishes the sequence successfully, it responds with an ACK, wait for answer
			//
			errorCode = sbgEComWaitForAck(pHandle, msgClass, msg, pHandle->cmdDefaultTimeOut);

			//
			// Test if the response is positive from device
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// ACK received, no need for other trial
				//
				break;
			}
		}
	}
	
	return errorCode;
}

/*!
 * Initiates a download sequences with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] msgClass					Original protocol class asking for transfer.
 * \param[in] msg						Original protocol message id asking for transfer.
 * \param[out] pSize					Size of the transfer initiated, returned from the device.
 * \return								SBG_NO_ERROR when the transfer initiated successfully.
 */
static SbgErrorCode sbgEComTransferReceiveInit(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, size_t *pSize)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	SbgStreamBuffer		inStreamBuffer;
	uint8_t				inputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint8_t				outputBuffer[sizeof(uint16_t)];
	uint8_t				receivedMsgClass;
	uint8_t				receivedMsg;
	uint16_t			transferCmd;
	size_t				inputSize;
	size_t				transferSize;
	uint32_t			i;

	assert(pHandle);

	//
	// Initialize stream buffer for output
	//
	sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outStreamBuffer));

	//
	// Build payload, only a SBG_ECOM_TRANSFER_START cmd
	//
	sbgStreamBufferWriteUint16LE(&outStreamBuffer, SBG_ECOM_TRANSFER_START);

	//
	// Send command (multiple times in case of failures)
	//
	for (i = 0; i < 3; i++)
	{
		//
		// Send transfer payload encapsulated in an ECom protocol frame
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));
		
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for reponse, the device should respond with a ECOM_TRANSFER_START command and the transfer size
			// If it can not initiate the transfer, it will respond with a NACK
			//
			errorCode = sbgEComReceiveAnyCmd(pHandle, &receivedMsgClass, &receivedMsg, inputBuffer, &inputSize, SBG_ECOM_MAX_PAYLOAD_SIZE, pHandle->cmdDefaultTimeOut);
			
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Test if the command received is the one expected
				// 
				if ((receivedMsgClass == msgClass) && (receivedMsg == msg))
				{
					//
					// Init stream buffer on received payload to process it
					//
					sbgStreamBufferInitForRead(&inStreamBuffer, inputBuffer, inputSize);

					//
					// Retrieve parameters, the first one is the transfer command
					// The second one is the total transfer size
					//
					transferCmd = sbgStreamBufferReadUint16LE(&inStreamBuffer);
					transferSize = sbgStreamBufferReadSizeT32LE(&inStreamBuffer);

					//
					// The device should have answered with SBG_ECOM_TRANSFER_START transfer command
					//
					if (transferCmd == SBG_ECOM_TRANSFER_START)
					{
						//
						// Update output variable with the transfer size
						//
						*pSize = transferSize;

						//
						// No need for other trials, exit loop/
						//
						break;
					}
					else
					{
						//
						// Invalid transfer command response
						//
						errorCode = SBG_ERROR;
					}
				}
				else
				{
					//
					// This is not the command expected
					// 
					errorCode = SBG_ERROR;
				}
			}
		}
	}
	
	return errorCode;
}

/*!
 * Receive one packet of data on a initiated download transfer.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] msgClass					Original protocol class asking for transfer.
 * \param[in] msg						Original protocol message id asking for transfer.
 * \param[in] pBuffer					Pointer to the buffer where to write the packet.
 * \param[in] offset					The offset from the start of the buffer.
 * \param[in] packetSize				The size of the data asked to the device.
 * \return								SBG_NO_ERROR if the packet was successfully received.
 */
static SbgErrorCode sbgEComTransferReceiveData(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, void *pBuffer, size_t offset, size_t packetSize)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	SbgStreamBuffer		inStreamBuffer;
	uint8_t				outputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint8_t				inputBuffer[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint16_t				transferCmd;
	uint8_t				receivedMsgClass;
	uint8_t				receivedMsg;
	size_t				rcvdOffset;
	size_t				inputSize;
	uint32_t				i;

	assert(pHandle);
	assert(pBuffer);
	assert(packetSize > 0);

	//
	// Initialize stream buffer for output
	//
	sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outputBuffer));

	//
	// Build payload: an SBG_ECOM_TRANSFER_DATA transfer command, the offset from the start of the transfer, the size of the packet the device must send
	//
	sbgStreamBufferWriteUint16LE(&outStreamBuffer, SBG_ECOM_TRANSFER_DATA);
	sbgStreamBufferWriteSizeT32LE(&outStreamBuffer, offset);
	sbgStreamBufferWriteSizeT32LE(&outStreamBuffer, packetSize);

	//
	// Send command (multiple times in case of failures)
	//
	for (i = 0; i < 3; i++)
	{
		//
		// Send transfer payload encapsulated in an ECom protocol frame
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));

		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for reponse, the device should respond with a ECOM_TRANSFER_DATA, the offset from the start of the transfer and the data payload
			// If it can not provide the data, it will respond with a NACK
			//
			errorCode = sbgEComReceiveAnyCmd(pHandle, &receivedMsgClass, &receivedMsg, inputBuffer, &inputSize, SBG_ECOM_MAX_PAYLOAD_SIZE, pHandle->cmdDefaultTimeOut);

			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Test if this is the protocol command expected
				//
				if ((receivedMsgClass == msgClass) && (receivedMsg == msg))
				{
					//
					// Initialize stream buffer for read on input buffer
					//
					sbgStreamBufferInitForRead(&inStreamBuffer, inputBuffer, inputSize);

					//
					// Read response fields, first is the transfer command, second is the offset
					//
					transferCmd = sbgStreamBufferReadUint16LE(&inStreamBuffer);
					rcvdOffset = sbgStreamBufferReadSizeT32LE(&inStreamBuffer);

					//
					// Test that it's a SBG_ECOM_TRANSFER_DATA command
					// The data is at the offset asked
					// And the size corresponds
					//
					if ( (transferCmd == SBG_ECOM_TRANSFER_DATA) && (offset == rcvdOffset) && (packetSize == (inputSize - (sizeof(uint16_t) + sizeof(uint32_t)))) )
					{
						//
						// Read then all the buffer
						//
						sbgStreamBufferReadBuffer(&inStreamBuffer, pBuffer, inputSize - (sizeof(uint16_t) + sizeof(uint32_t)));

						//
						// No need for other trials, exit loop
						//
						break;
					}
				}
				else
				{
					//
					// Not the command expected
					//
					errorCode = SBG_ERROR;
				}
			}
		}
	}
	
	return errorCode;
}

/*!
 * Function that ends a download sequence with a device.
 * \param[in] pHandle					Pointer to a valid SbgEComHandle.
 * \param[in] msgClass					Original protocol class asking for transfer.
 * \param[in] msg						Original protocol message id asking for transfer.
 * \return								SBG_NO_ERROR when the transfer ended successfully.
 */
static SbgErrorCode sbgEComTransferReceiveEnd(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outStreamBuffer;
	uint8_t				outputBuffer[sizeof(uint16_t)];
	uint32_t			i;

	assert(pHandle);
	
	//
	// Initialize stream buffer for output
	//
	sbgStreamBufferInitForWrite(&outStreamBuffer, outputBuffer, sizeof(outStreamBuffer));

	//
	// Build payload, only a SBG_ECOM_TRANSFER_END cmd
	//
	sbgStreamBufferWriteUint16LE(&outStreamBuffer, SBG_ECOM_TRANSFER_END);
		
	//
	// Send command (multiple times in case of failures)
	//
	for (i = 0; i < 3; i++)
	{
		//
		// Send upload end payload encapsulated in a ECom protocol frame
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, msgClass, msg, sbgStreamBufferGetLinkedBuffer(&outStreamBuffer), sbgStreamBufferGetLength(&outStreamBuffer));

		if (errorCode == SBG_NO_ERROR)
		{
			//
			// If the device is able to finish transfer sequence, it responds with an ACK
			//
			errorCode = sbgEComWaitForAck(pHandle, msgClass, msg, pHandle->cmdDefaultTimeOut);

			//
			// Test if the response is positive from device
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// No need for other trial, exit loop
				//
				break;
			}
		}
	}
	
	return errorCode;
}

//----------------------------------------------------------------------//
//- Public transfer method definitions			                       -//
//----------------------------------------------------------------------//

/*!
 * Specific method to handle a large send into multiple frames.
 * \param[in]	pHandle					Pointer to a valid SbgEComHandle.
 * \param[in]	msgClass				Original protocol class asking for transfer.
 * \param[in]	msg						Original protocol message id asking for transfer.
 * \param[in]	pBuffer					Pointer to the buffer containing the data to send.
 * \param[in]	size					The size of the buffer.
 * \return								SBG_NO_ERROR in case of a successful upload.
 */
SbgErrorCode sbgEComTransferSend(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, const void *pBuffer, size_t size)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	SbgSplitBuffer	splitBuffer;
	size_t			i;

	assert(pHandle);
	assert(pBuffer);
	assert(size > 0);

	//
	// Make sure we are not trying to send a buffer that is too large
	//
	if (size <= SBG_ECOM_TRANSFER_MAX_SIZE)
	{
		//
		// Initiate data transfer
		//
		errorCode = sbgEComTransferSendInit(pHandle, msgClass, msg, size);

		//
		// Check that the transfer was correctly initialized
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Initialize split buffer that will help with splitting up provided buffer
			//
			sbgSplitBufferInitForRead(&splitBuffer, pBuffer, size, SBG_ECOM_TRANSFER_PACKET_SIZE);

			//
			// Transfer sub buffer one by one
			//
			for (i = 0; i < sbgSplitBufferGetSubBufferNbr(&splitBuffer); i++)
			{
				//
				// Send a sub buffer
				//
				errorCode = sbgEComTransferSendData(pHandle, msgClass, msg, sbgSplitBufferGetSubBuffer(&splitBuffer, i), sbgSplitBufferGetSubBufferOffset(&splitBuffer, i), sbgSplitBufferGetSubBufferSize(&splitBuffer, i));

				//
				// Test if the sub buffer has been sent
				//
				if (errorCode != SBG_NO_ERROR)
				{
					//
					// Unable to send a sub buffer, abort send operation.
					//
					break;
				}
			}

			//
			// Test if any error occurred during data transfer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// End data transfer
				//
				errorCode = sbgEComTransferSendEnd(pHandle, msgClass, msg);
			}
		}
	}
	else
	{
		//
		// Trying to send a buffer that is too large
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 * Specific method to handle a large receive from the device.
 * \param[in]	pHandle					Pointer to a valid SbgEComHandle.
 * \param[in]	msgClass				Original protocol class asking for transfer.
 * \param[in]	msg						Original protocol message id asking for transfer.
 * \param[in]	pBuffer					Pointer to the buffer where to write data.
 * \param[out]	pActualSize				The final size written into the buffer.
 * \param[in]	size					The size of the buffer.
 * \return								SBG_NO_ERROR in case of a successful download.
 */
SbgErrorCode sbgEComTransferReceive(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, void *pBuffer, size_t *pActualSize, size_t bufferSize)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	SbgSplitBuffer	splitBuffer;
	size_t			transferSize;
	size_t			i;

	assert(pHandle);
	assert(pBuffer);
	assert(pActualSize);
	assert(bufferSize > 0);

	//
	// initiate data transfer
	//
	errorCode = sbgEComTransferReceiveInit(pHandle, msgClass, msg, &transferSize);

	//
	// Make sure the receive transfer has been correctly initialized
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Test that the provided buffer is large enough to receive all data
		//
		if (transferSize <= bufferSize)
		{
			//
			// Initialize Split buffer to help with sub buffer receive
			//
			sbgSplitBufferInitForWrite(&splitBuffer, pBuffer, transferSize, SBG_ECOM_TRANSFER_PACKET_SIZE);

			//
			// Receive buffers one by one 
			//
			for (i = 0; i < sbgSplitBufferGetSubBufferNbr(&splitBuffer); i++)
			{
				//
				// Receive a sub buffer
				//																									  
				errorCode = sbgEComTransferReceiveData(pHandle, msgClass, msg, sbgSplitBufferGetSubBuffer(&splitBuffer, i), sbgSplitBufferGetSubBufferOffset(&splitBuffer, i), sbgSplitBufferGetSubBufferSize(&splitBuffer, i));

				//
				// Make sure that the sub buffer has been correctly received
				//
				if (errorCode != SBG_NO_ERROR)
				{
					//
					// An error occurred, abort data transfer
					//
					break;
				}
			}

			//
			// Test if any error occurred during transfer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// End data transfer
				//
				errorCode = sbgEComTransferReceiveEnd(pHandle, msgClass, msg);

				//
				// Make sure that the transfer has been correctly ended
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Since the transfer was successful update output variable pActualSize
					//
					*pActualSize = transferSize;
				}
			}
		}
		else
		{
			//
			// Provided buffer is too small
			//
			errorCode = SBG_INVALID_PARAMETER;
		}
	}

	return errorCode;
}
