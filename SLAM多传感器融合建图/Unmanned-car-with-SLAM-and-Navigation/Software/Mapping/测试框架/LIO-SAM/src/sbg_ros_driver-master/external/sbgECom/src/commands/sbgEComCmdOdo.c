/*!
 *	\file		sbgEComCmdOdo.c
 *  \author		SBG Systems
 *
 *	\brief		This file implements SbgECom commands related to Odometer module.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2019, SBG Systems SAS. All rights reserved.
 *	
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */
#include "sbgEComCmdOdo.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Odometer commands                                                  -//
//----------------------------------------------------------------------//

SbgErrorCode sbgEComCmdOdoGetConf(SbgEComHandle *pHandle, SbgEComOdoConf *pOdometerConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pOdometerConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_ODO_CONF command
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
				pOdometerConf->gain 		= sbgStreamBufferReadFloatLE(&inputStream);
				pOdometerConf->gainError 	= sbgStreamBufferReadUint8LE(&inputStream);
				pOdometerConf->reverseMode 	= sbgStreamBufferReadBooleanLE(&inputStream);

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

SbgErrorCode sbgEComCmdOdoSetConf(SbgEComHandle *pHandle, const SbgEComOdoConf *pOdometerConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pOdometerConf);

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
		sbgStreamBufferWriteFloatLE(&outputStream, pOdometerConf->gain);
		sbgStreamBufferWriteUint8LE(&outputStream, pOdometerConf->gainError);
		sbgStreamBufferWriteBooleanLE(&outputStream, pOdometerConf->reverseMode);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, pHandle->cmdDefaultTimeOut);

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

SbgErrorCode sbgEComCmdOdoGetLeverArm(SbgEComHandle *pHandle, float leverArm[3])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(leverArm);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

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
				//
				leverArm[0] = sbgStreamBufferReadFloatLE(&inputStream);
				leverArm[1] = sbgStreamBufferReadFloatLE(&inputStream);
				leverArm[2] = sbgStreamBufferReadFloatLE(&inputStream);

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

SbgErrorCode sbgEComCmdOdoSetLeverArm(SbgEComHandle *pHandle, const float leverArm[3])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(leverArm);

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
		sbgStreamBufferWriteFloatLE(&outputStream, leverArm[0]);
		sbgStreamBufferWriteFloatLE(&outputStream, leverArm[1]);
		sbgStreamBufferWriteFloatLE(&outputStream, leverArm[2]);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, pHandle->cmdDefaultTimeOut);

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

SbgErrorCode sbgEComCmdOdoGetRejection(SbgEComHandle *pHandle, SbgEComOdoRejectionConf *pRejectConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
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
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

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
				//
				pRejectConf->velocity = (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);

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

SbgErrorCode sbgEComCmdOdoSetRejection(SbgEComHandle *pHandle, const SbgEComOdoRejectionConf *pRejectConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pRejectConf);

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
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pRejectConf->velocity);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, pHandle->cmdDefaultTimeOut);

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

SbgErrorCode sbgEComCmdOdoCanGetConf(SbgEComHandle *pHandle, SbgEComCmdOdoCanChannel canChannel, SbgEComCmdOdoCanConf *pOdoCanConf)
{
	SbgErrorCode			errorCode = SBG_NO_ERROR;
	uint32_t				trial;
	uint8_t					cmdPayload[16];
	size_t					receivedSize;	
	uint8_t					receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer			outputStream;
	SbgStreamBuffer			inputStream;

	assert(pHandle);
	assert(pOdoCanConf);
	assert(canChannel <= UCHAR_MAX);

	//
	// Build the command payload used to ask the CAN odometer configuration for a specific channel
	//
	sbgStreamBufferInitForWrite(&outputStream, cmdPayload, sizeof(cmdPayload));
	sbgStreamBufferWriteUint8LE(&outputStream, canChannel);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CAN_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));
		
		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CAN_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_ODO_CAN_SPEED_CONF command
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to read parameters
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				//
				// Read fields from payload
				//
				canChannel					= sbgStreamBufferReadUint8LE(&inputStream);

				pOdoCanConf->options 		= sbgStreamBufferReadUint16LE(&inputStream);
				pOdoCanConf->canId 			= sbgStreamBufferReadUint32LE(&inputStream);
				
				pOdoCanConf->startBit	 	= sbgStreamBufferReadUint8LE(&inputStream);
				pOdoCanConf->dataSize		= sbgStreamBufferReadUint8LE(&inputStream);

				pOdoCanConf->scale 			= sbgStreamBufferReadFloatLE(&inputStream);
				pOdoCanConf->offset 		= sbgStreamBufferReadFloatLE(&inputStream);
				pOdoCanConf->minValue 		= sbgStreamBufferReadFloatLE(&inputStream);
				pOdoCanConf->maxValue 		= sbgStreamBufferReadFloatLE(&inputStream);

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

SbgErrorCode sbgEComCmdOdoCanSetConf(SbgEComHandle *pHandle, SbgEComCmdOdoCanChannel canChannel, const SbgEComCmdOdoCanConf *pOdoCanConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pOdoCanConf);
	assert(canChannel <= UCHAR_MAX);

	//
	// A CAN message has a payload of up to 64 bits so the offset can range from 0 to 63 and size from 1 to 64
	//
	assert(pOdoCanConf->startBit < 64);
	assert(pOdoCanConf->dataSize > 0);
	assert(pOdoCanConf->dataSize <= 64);

	//
	// Build the command payload
	//
	sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
	
	sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)canChannel);

	sbgStreamBufferWriteUint16LE(&outputStream, pOdoCanConf->options);
	sbgStreamBufferWriteUint32LE(&outputStream, pOdoCanConf->canId);

	sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pOdoCanConf->startBit);
	sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pOdoCanConf->dataSize);
	
	sbgStreamBufferWriteFloatLE(&outputStream, pOdoCanConf->scale);
	sbgStreamBufferWriteFloatLE(&outputStream, pOdoCanConf->offset);
	sbgStreamBufferWriteFloatLE(&outputStream, pOdoCanConf->minValue);
	sbgStreamBufferWriteFloatLE(&outputStream, pOdoCanConf->maxValue);
	
	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CAN_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CAN_CONF, pHandle->cmdDefaultTimeOut);

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
