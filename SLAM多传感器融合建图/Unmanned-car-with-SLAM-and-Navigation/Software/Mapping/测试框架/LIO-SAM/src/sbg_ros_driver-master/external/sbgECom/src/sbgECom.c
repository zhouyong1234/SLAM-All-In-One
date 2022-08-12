#include "sbgECom.h"
#include <streamBuffer/sbgStreamBuffer.h>
#include "commands/sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Private methods declarations                                       -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Public methods declarations                                        -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the protocol system used to communicate with the product and return the created handle.
 *	\param[out]	pHandle							Pointer used to store the allocated and initialized sbgECom handle.
 *	\param[in]	pInterface						Interface to use for read/write operations.
 *	\return										SBG_NO_ERROR if we have initialised the protocol system.
 */
SbgErrorCode sbgEComInit(SbgEComHandle *pHandle, SbgInterface *pInterface)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	
	//
	// Check input parameters
	//
	if ( (pInterface) && (pHandle) )
	{
		//
		// Initialize the sbgECom handle
		//
		pHandle->pReceiveLogCallback	= NULL;
		pHandle->pUserArg				= NULL;

		//
		// Initialize the default number of trials and time out
		//
		pHandle->numTrials			= 3;
		pHandle->cmdDefaultTimeOut	= SBG_ECOM_DEFAULT_CMD_TIME_OUT;

		//
		// Initialize the protocol 
		//
		errorCode = sbgEComProtocolInit(&pHandle->protocolHandle, pInterface);
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	pHandle							A valid sbgECom handle to close.
 *	\return										SBG_NO_ERROR if we have closed and released the sbgECom system.
 */
SbgErrorCode sbgEComClose(SbgEComHandle *pHandle)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		//
		// Close the protocol
		//
		errorCode = sbgEComProtocolClose(&pHandle->protocolHandle);
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Try to parse one log from the input interface and then return.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\return										SBG_NO_ERROR if no error occurs during incoming log parsing.
 */
SbgErrorCode sbgEComHandleOneLog(SbgEComHandle *pHandle)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgBinaryLogData	logData;
	uint8_t				receivedMsg;
	uint8_t				receivedMsgClass;
	size_t				payloadSize;
	uint8_t				payloadData[SBG_ECOM_MAX_PAYLOAD_SIZE];

	//
	// Check input arguments
	//
	assert(pHandle);

	//
	// Try to read a received frame
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
			errorCode = sbgEComBinaryLogParse((SbgEComClass)receivedMsgClass, (SbgEComMsgId)receivedMsg, payloadData, payloadSize, &logData);

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
			// We have received a command, it shouldn't happen
			//
		}
	}
	else if (errorCode != SBG_NOT_READY)
	{
		//
		// We have received an invalid frame
		//
		SBG_LOG_WARNING(errorCode, "Invalid frame received");
	}
	
	return errorCode;
}

/*!
 *	Handle all incoming logs until no more log are available in the input interface.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\return										SBG_NO_ERROR if no error occurs during incoming logs parsing.
 */
SbgErrorCode sbgEComHandle(SbgEComHandle *pHandle)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	
	//
	// Check input arguments
	//
	assert(pHandle);

	//
	// Try to read all received frames, we thus loop until we get an SBG_NOT_READY error
	//
	do
	{
		//
		// Try to read and parse one frame
		//
		errorCode = sbgEComHandleOneLog(pHandle);
	} while (errorCode != SBG_NOT_READY);
	
	return errorCode;
}

/*!
 *	Define the callback that should be called each time a new binary log is received.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\param[in]	pReceiveLogCallback				Pointer on the callback to call when a new log is received.
 *	\param[in]	pUserArg						Optional user argument that will be passed to the callback method.
 *	\return										SBG_NO_ERROR if the callback and user argument have been defined successfully.
 */
SbgErrorCode sbgEComSetReceiveLogCallback(SbgEComHandle *pHandle, SbgEComReceiveLogFunc pReceiveLogCallback, void *pUserArg)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test that we have a valid protocol handle
	//
	if (pHandle)
	{
		//
		// Define the callback and the user argument
		//
		pHandle->pReceiveLogCallback = pReceiveLogCallback;
		pHandle->pUserArg = pUserArg;
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 * Define the default number of trials that should be done when a command is send to the device as well as the time out.
 * \param[in]	pHandle							A valid sbgECom handle.
 * \parma[in]	numTrials						Number of trials when a command is sent (starting at 1).
 * \param[in]	cmdDefaultTimeOut				Default time out in milliseconds to wait to receive an answer from the device.
 */
void sbgEComSetCmdTrialsAndTimeOut(SbgEComHandle *pHandle, uint32_t numTrials, uint32_t cmdDefaultTimeOut)
{
	//
	// Check input arguments
	//
	assert(pHandle);
	assert(numTrials > 0);
	assert(cmdDefaultTimeOut > 0);

	//
	// Define the new settings
	//
	pHandle->numTrials			= numTrials;
	pHandle->cmdDefaultTimeOut	= cmdDefaultTimeOut;
}

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						The errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold the error string.
 */
void sbgEComErrorToString(SbgErrorCode errorCode, char errorMsg[256])
{
	if (errorMsg)
	{
		//
		// For each error code, copy the error msg
		//
		switch (errorCode)
		{
		case SBG_NO_ERROR:
			strcpy(errorMsg, "SBG_NO_ERROR: No error."); 
			break;
		case SBG_ERROR:
			strcpy(errorMsg, "SBG_ERROR: Generic error."); 
			break;
		case SBG_NULL_POINTER:
			strcpy(errorMsg, "SBG_NULL_POINTER: A pointer is null."); 
			break;
		case SBG_INVALID_CRC:
			strcpy(errorMsg, "SBG_INVALID_CRC: The received frame has an invalid CRC.");
			break;
		case SBG_INVALID_FRAME:
			strcpy(errorMsg, "SBG_INVALID_FRAME: The received frame is invalid.");
			break;
		case SBG_TIME_OUT:
			strcpy(errorMsg, "SBG_TIME_OUT: We have a time out during frame reception.");
			break;
		case SBG_WRITE_ERROR:
			strcpy(errorMsg, "SBG_WRITE_ERROR: All bytes hasn't been written.");
			break;
		case SBG_READ_ERROR:
			strcpy(errorMsg, "SBG_READ_ERROR: All bytes hasn't been read.");
			break;
		case SBG_BUFFER_OVERFLOW:
			strcpy(errorMsg, "SBG_BUFFER_OVERFLOW: A buffer is too small to contain so much data.");
			break;
		case SBG_INVALID_PARAMETER:
			strcpy(errorMsg, "SBG_INVALID_PARAMETER: An invalid parameter has been founded.");
			break;
		case SBG_NOT_READY:
			strcpy(errorMsg, "SBG_NOT_READY: A device isn't ready (Rx isn't ready for example).");
			break;
		case SBG_MALLOC_FAILED:
			strcpy(errorMsg, "SBG_MALLOC_FAILED: Failed to allocate a buffer.");
			break;
		case SGB_CALIB_MAG_NOT_ENOUGH_POINTS:
			strcpy(errorMsg, "SGB_CALIB_MAG_NOT_ENOUGH_POINTS: Not enough points were available to perform magnetometers calibration.");
			break;
		case SBG_CALIB_MAG_INVALID_TAKE:
			strcpy(errorMsg, "SBG_CALIB_MAG_INVALID_TAKE: The calibration procedure could not be properly executed due to insufficient precision.");
			break;
		case SBG_CALIB_MAG_SATURATION:
			strcpy(errorMsg, "SBG_CALIB_MAG_SATURATION: Saturation were detected when attempt to calibrate magnetos.");
			break;
		case SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE:
			strcpy(errorMsg, "SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE: 2D calibration procedure could not be performed.");
			break;
		case SBG_DEVICE_NOT_FOUND:
			strcpy(errorMsg, "SBG_DEVICE_NOT_FOUND: A device couldn't be founded or opened.");
			break;
		case SBG_OPERATION_CANCELLED:
			strcpy(errorMsg, "SBG_OPERATION_CANCELLED: An operation has been cancelled by a user.");
			break;
		case SBG_NOT_CONTINUOUS_FRAME:
			strcpy(errorMsg, "SBG_NOT_CONTINUOUS_FRAME: We have received a frame that isn't a continuous one.");
			break;
		case SBG_INCOMPATIBLE_HARDWARE:
			strcpy(errorMsg, "SBG_INCOMPATIBLE_HARDWARE: Hence valid, the configuration cannot be executed because of incompatible hardware.");
			break;
		default:
			sprintf(errorMsg, "Undefined error code: %u", errorCode);
			break;
		}
	}
}
