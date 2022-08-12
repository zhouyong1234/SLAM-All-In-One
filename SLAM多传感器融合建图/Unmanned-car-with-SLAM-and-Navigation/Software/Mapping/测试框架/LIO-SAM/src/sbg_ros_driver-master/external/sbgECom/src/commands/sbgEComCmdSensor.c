#include "sbgEComCmdSensor.h"
#include <streamBuffer/sbgStreamBuffer.h>
#include "transfer/sbgEComTransfer.h"

//----------------------------------------------------------------------//
//- Sensor commands		                                               -//
//----------------------------------------------------------------------//

/*!
 *	Set a Motion profile ID
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[id]	id							Motion profile ID to set
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorSetMotionProfileId(SbgEComHandle *pHandle, uint32_t id)
{
	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericSetModelId(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MOTION_PROFILE_ID, id);
}


/*!
 *	Retrieve Motion profile information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current motion profile info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorGetMotionProfileInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo)
{
	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericGetModelInfo(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_MOTION_PROFILE_ID, pModelInfo);
}

/*!
 *	Retrieve the initial conditions settings.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pConf						Pointer to a SbgEComInitConditionConf to contain the current initial conditions settings.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorGetInitCondition(SbgEComHandle *pHandle, SbgEComInitConditionConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INIT_PARAMETERS, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INIT_PARAMETERS, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_INIT_PARAMETERS command
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
				pConf->latitude = sbgStreamBufferReadDoubleLE(&inputStream);
				pConf->longitude = sbgStreamBufferReadDoubleLE(&inputStream);
				pConf->altitude = sbgStreamBufferReadDoubleLE(&inputStream);
				pConf->year = sbgStreamBufferReadUint16LE(&inputStream);
				pConf->month = sbgStreamBufferReadUint8LE(&inputStream);
				pConf->day = sbgStreamBufferReadUint8LE(&inputStream);

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
 *	Set the initial condition configuration.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pConf						Pointer to a SbgEComInitConditionConf containing the new initial condition configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorSetInitCondition(SbgEComHandle *pHandle, const SbgEComInitConditionConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
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
		sbgStreamBufferWriteDoubleLE(&outputStream, pConf->latitude);
		sbgStreamBufferWriteDoubleLE(&outputStream, pConf->longitude);
		sbgStreamBufferWriteDoubleLE(&outputStream, pConf->altitude);
		sbgStreamBufferWriteUint16LE(&outputStream, (uint16_t)pConf->year);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->month);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->day);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INIT_PARAMETERS, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INIT_PARAMETERS, pHandle->cmdDefaultTimeOut);

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
 *	Retrieve the assignment of the aiding sensors.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pConf						Pointer to a SbgEComAidingAssignConf to contain the current assignment of the aiding sensors.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorGetAidingAssignment(SbgEComHandle *pHandle, SbgEComAidingAssignConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[16];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIDING_ASSIGNMENT, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIDING_ASSIGNMENT, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_AIDING_ASSIGNMENT command
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
				pConf->gps1Port			= (SbgEComModulePortAssignment)sbgStreamBufferReadUint8LE(&inputStream);
				pConf->gps1Sync			= (SbgEComModuleSyncAssignment)sbgStreamBufferReadUint8LE(&inputStream);
					
				sbgStreamBufferSeek(&inputStream, 4*sizeof(uint8_t), SB_SEEK_CUR_INC);						/*!< Reserved fields to ignore */

				pConf->dvlPort			= (SbgEComModulePortAssignment)sbgStreamBufferReadUint8LE(&inputStream);
				pConf->dvlSync			= (SbgEComModuleSyncAssignment)sbgStreamBufferReadUint8LE(&inputStream);

				pConf->rtcmPort			= (SbgEComModulePortAssignment)sbgStreamBufferReadUint8LE(&inputStream);
				pConf->airDataPort		= (SbgEComModulePortAssignment)sbgStreamBufferReadUint8LE(&inputStream);
				pConf->odometerPinsConf	= (SbgEComOdometerPinAssignment)sbgStreamBufferReadUint8LE(&inputStream);

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
 *	Set the assignment of the aiding sensors.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pConf						Pointer to a SbgEComAidingAssignConf containing the new assignment of the aiding sensors.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorSetAidingAssignment(SbgEComHandle *pHandle, const SbgEComAidingAssignConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[16];
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
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->gps1Port);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->gps1Sync);

		//
		// Skip the 4 reserved bytes
		//
		sbgStreamBufferWriteUint8LE(&outputStream, 0);
		sbgStreamBufferWriteUint8LE(&outputStream, 0);
		sbgStreamBufferWriteUint8LE(&outputStream, 0);
		sbgStreamBufferWriteUint8LE(&outputStream, 0);

		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->dvlPort);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->dvlSync);			
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->rtcmPort);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->airDataPort);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pConf->odometerPinsConf);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIDING_ASSIGNMENT, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_AIDING_ASSIGNMENT, pHandle->cmdDefaultTimeOut);

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
 *	Retrieve the alignment and lever arm configuration of the sensor.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComSensorAlignmentInfo struct to hold alignment configuration of the sensor.
 *	\param[out] leverArm					Pointer to a table to contain lever arm X, Y, Z components in meters.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorGetAlignmentAndLeverArm(SbgEComHandle *pHandle, SbgEComSensorAlignmentInfo *pAlignConf, float leverArm[3])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[32];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pAlignConf);
	assert(leverArm);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command only since this is a no-payload command
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received a SBG_ECOM_CMD_IMU_ALIGNMENT command
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
				pAlignConf->axisDirectionX = (SbgEComAxisDirection)sbgStreamBufferReadUint8LE(&inputStream);
				pAlignConf->axisDirectionY = (SbgEComAxisDirection)sbgStreamBufferReadUint8LE(&inputStream);
				pAlignConf->misRoll = sbgStreamBufferReadFloatLE(&inputStream);
				pAlignConf->misPitch = sbgStreamBufferReadFloatLE(&inputStream);
				pAlignConf->misYaw = sbgStreamBufferReadFloatLE(&inputStream);
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

/*!
 *	Set the alignment and lever arm configuration of the sensor.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pAlignConf					Pointer to a SbgEComSensorAlignmentInfo struct holding alignment configuration for the sensor.
 *	\param[in]  leverArm					Pointer to a table containing lever arm X, Y, Z components in meters.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSensorSetAlignmentAndLeverArm(SbgEComHandle *pHandle, const SbgEComSensorAlignmentInfo *pAlignConf, const float leverArm[3])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[32];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pAlignConf);
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
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pAlignConf->axisDirectionX);
		sbgStreamBufferWriteUint8LE(&outputStream, (uint8_t)pAlignConf->axisDirectionY);
		sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->misRoll);
		sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->misPitch);
		sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->misYaw);
		sbgStreamBufferWriteFloatLE(&outputStream, leverArm[0]);
		sbgStreamBufferWriteFloatLE(&outputStream, leverArm[1]);
		sbgStreamBufferWriteFloatLE(&outputStream, leverArm[2]);

		//
		// Send the payload over ECom
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM, pHandle->cmdDefaultTimeOut);

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
