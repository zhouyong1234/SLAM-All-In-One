#include "sbgEComCmdMag.h"
#include <streamBuffer/sbgStreamBuffer.h>
#include "transfer/sbgEComTransfer.h"

//----------------------------------------------------------------------//
//- Magnetometer commands                                              -//
//----------------------------------------------------------------------//

/*!
 *	Set magnetometer error model ID.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	id							Magnetometer model ID to set
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetModelId(SbgEComHandle *pHandle, uint32_t id)
{
	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericSetModelId(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_MODEL_ID, id);
}

/*!
 *	Retrieve magnetometer error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current magnetometer error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagGetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo)
{
	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericGetModelInfo(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_MODEL_ID, pModelInfo);
}

/*!
 *	Send a command that set the magnetometers calibration parameters.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	offset						Magnetometers calibration offset vector.
 *	\param[in]	matix						Magnetometers calibration 3x3 matrix.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetCalibData(SbgEComHandle *pHandle, const float offset[3], const float matrix[9])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outputStream;
	uint8_t				payload[12*sizeof(float)];
	uint32_t			trial;
	uint32_t			i;

	assert(pHandle);
	assert(offset);
	assert(matrix);

	//
	// Initialize a stream buffer to write the command payload
	//
	errorCode = sbgStreamBufferInitForWrite(&outputStream, payload, sizeof(payload));

	//
	// Write the offset vector
	//
	sbgStreamBufferWriteFloatLE(&outputStream, offset[0]);
	sbgStreamBufferWriteFloatLE(&outputStream, offset[1]);
	sbgStreamBufferWriteFloatLE(&outputStream, offset[2]);

	//
	// Write the matrix
	//
	for (i = 0; i < 9; i++)
	{
		sbgStreamBufferWriteFloatLE(&outputStream, matrix[i]);
	}

	//
	// Make sure that the stream buffer has been initialized
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SET_MAG_CALIB, payload, sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SET_MAG_CALIB, pHandle->cmdDefaultTimeOut);

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

/*!
 *	Retrieve the rejection configuration of the magnetometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pRejectConf					Pointer to a SbgEComMagRejectionConf struct to hold rejection configuration of the magnetometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagGetRejection(SbgEComHandle *pHandle, SbgEComMagRejectionConf *pRejectConf)
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
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received frame was OK
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
				pRejectConf->magneticField = (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);

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
 *	Set the rejection configuration of the magnetometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pRejectConf					Pointer to a SbgEComMagRejectionConf struct holding rejection configuration for the magnetometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetRejection(SbgEComHandle *pHandle, const SbgEComMagRejectionConf *pRejectConf)
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
		// Build payload
		//
		sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pRejectConf->magneticField);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE, pHandle->cmdDefaultTimeOut);

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

//----------------------------------------------------------------------//
//- Magnetometer onboard calibration commands	                       -//
//----------------------------------------------------------------------//

/*!
 *	Start the magnetic calibration process.
 *	As soon as this command is sent, the device will start logging magnetic field data internally.
 *	This set of data will be used later by the magnetic calibration algorithms to map the surrounding magnetic field.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	mode						Define which magnetic calibration type to perform. It could be 3D or 2D.
 *	\param[in]	bandwidth					Tell the device that we should have low, medium or high dynamics during the magnetic calibration process.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagStartCalib(SbgEComHandle *pHandle, SbgEComMagCalibMode mode, SbgEComMagCalibBandwidth bandwidth)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		outputStream;
	uint8_t				payload[2];
	uint32_t			trial;

	assert(pHandle);
	
	//
	// Initialize a stream buffer to write the command payload
	//
	errorCode = sbgStreamBufferInitForWrite(&outputStream, payload, sizeof(payload));

	//
	// Write the calibration mode and bandwith
	//
	sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)mode);
	sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)bandwidth);
		
	//
	// Make sure that the stream buffer has been initialized
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_START_MAG_CALIB, payload, sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_START_MAG_CALIB, pHandle->cmdDefaultTimeOut);

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

/*!
 *	This command computes a magnetic calibration solution based on the magnetic field logged since the last call to the command SBG_ECOM_CMD_START_MAG_CALIB (15).
 *	As soon as the computations are done, the device will answer with quality indicators, status flags and if possible a valid magnetic calibration matrix and offset.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pCalibResults				Pointer on a SbgEComMagCalibResults structure that can hold onboard magnetic calibration results and status.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagComputeCalib(SbgEComHandle *pHandle, SbgEComMagCalibResults *pCalibResults)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	uint32_t			i;

	assert(pHandle);
	assert(pCalibResults);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_COMPUTE_MAG_CALIB, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 5 s because the onboard magnetic computation can take some time
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_COMPUTE_MAG_CALIB, receivedBuffer, &receivedSize, sizeof(receivedBuffer), 5000);

			//
			// Test if we have received the correct command
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to read parameters
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				//
				// Read quality and status parameters
				//
				pCalibResults->quality			= (SbgEComMagCalibQuality)sbgStreamBufferReadUint8LE(&inputStream);
				pCalibResults->confidence		= (SbgEComMagCalibConfidence)sbgStreamBufferReadUint8LE(&inputStream);
				pCalibResults->advancedStatus	= sbgStreamBufferReadUint16LE(&inputStream);

				pCalibResults->beforeMeanError	= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->beforeStdError	= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->beforeMaxError	= sbgStreamBufferReadFloatLE(&inputStream);

				pCalibResults->afterMeanError	= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->afterStdError	= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->afterMaxError	= sbgStreamBufferReadFloatLE(&inputStream);

				pCalibResults->meanAccuracy		= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->stdAccuracy		= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->maxAccuracy		= sbgStreamBufferReadFloatLE(&inputStream);

				pCalibResults->numPoints		= sbgStreamBufferReadUint16LE(&inputStream);
				pCalibResults->maxNumPoints		= sbgStreamBufferReadUint16LE(&inputStream);

				//
				// Read the computed hard iron offset vector
				//
				pCalibResults->offset[0]		= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->offset[1]		= sbgStreamBufferReadFloatLE(&inputStream);
				pCalibResults->offset[2]		= sbgStreamBufferReadFloatLE(&inputStream);

				//
				// Read the computed soft iron matrix
				//
				for (i = 0; i < 9; i++)
				{
					pCalibResults->matrix[i]	= sbgStreamBufferReadFloatLE(&inputStream);
				}

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
