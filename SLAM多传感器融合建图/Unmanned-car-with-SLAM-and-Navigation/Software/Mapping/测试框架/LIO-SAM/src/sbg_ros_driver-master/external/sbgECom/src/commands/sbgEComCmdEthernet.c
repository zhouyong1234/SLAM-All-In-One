#include "sbgEComCmdEthernet.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Private methods declarations                                       -//
//----------------------------------------------------------------------//

/*!
 * Write in the output stream buffer, the provided Ethernet configuration.
 * \param[out]	pOutputStream					Pointer on the output stream buffer to write to.
 * \param[in]	pEthernetConf					Structure used to hold the parameters to write to the payload buffer.
 * \return										SBG_NO_ERROR if the structure has been written correctly.
 */
static SbgErrorCode sbgEComEthernetConfWrite(SbgStreamBuffer *pOutputStream, const SbgEComEthernetConf *pEthernetConf)
{
	assert(pOutputStream);
	assert(pEthernetConf);

	//
	// Build payload
	//
	sbgStreamBufferWriteUint8LE(pOutputStream, (uint8_t)pEthernetConf->mode);
	sbgStreamBufferWriteUint32LE(pOutputStream, pEthernetConf->ipAddress);
	sbgStreamBufferWriteUint32LE(pOutputStream, pEthernetConf->netmask);
	sbgStreamBufferWriteUint32LE(pOutputStream, pEthernetConf->gateway);
	sbgStreamBufferWriteUint32LE(pOutputStream, pEthernetConf->dns1);
	sbgStreamBufferWriteUint32LE(pOutputStream, pEthernetConf->dns2);

	//
	// Return if an error has occurred during the parse
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}

/*!
 * Parse the input stream buffer to extract all parameters and fill the corresponding structure.
 * \param[in]	pInputStream					Pointer on the input stream buffer to read from.
 * \param[out]	pEthernetConf					Structure used to store the parsed parameters.
 * \return										SBG_NO_ERROR if the structure has been parsed correctly.
 */
static SbgErrorCode sbgEComEthernetConfParse(SbgStreamBuffer *pInputStream, SbgEComEthernetConf *pEthernetConf)
{
	assert(pInputStream);
	assert(pEthernetConf);

	//
	// Read all parameters from the payload
	//
	pEthernetConf->mode = (SbgEComEthernetMode)sbgStreamBufferReadUint8LE(pInputStream);
	pEthernetConf->ipAddress = (sbgIpAddress)sbgStreamBufferReadUint32LE(pInputStream);
	pEthernetConf->netmask = (sbgIpAddress)sbgStreamBufferReadUint32LE(pInputStream);
	pEthernetConf->gateway = (sbgIpAddress)sbgStreamBufferReadUint32LE(pInputStream);
	pEthernetConf->dns1 = (sbgIpAddress)sbgStreamBufferReadUint32LE(pInputStream);
	pEthernetConf->dns2 = (sbgIpAddress)sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Return if an error has occurred during the parse
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
* Get the configuration for the Ethernet interface.
* Warning: this method only returns the Ethernet configuration and NOT the ip address currently used by the device.
* You should rather use sbgEComEthernetInfo to retreive the current assigned IP.
* \param[in]	pHandle						A valid sbgECom handle.
* \param[out]	pEthernetConf				Poiner to a SbgEComEthernetConf struct that holds the read configuration from the device.
* \return									SBG_NO_ERROR if the command has been executed successfully.
*/
SbgErrorCode sbgEComEthernetGetConf(SbgEComHandle *pHandle, SbgEComEthernetConf *pEthernetConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pEthernetConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command with no payload to retreive the network configuration
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ETHERNET_CONF, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ETHERNET_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received correctly the answer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to read parameters
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

				//
				// Read all parameters from the payload and return any error during the parse
				//
				errorCode = sbgEComEthernetConfParse(&inputStream, pEthernetConf);

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
* Set the configuration for the Ethernet interface.
* \param[in]	pHandle						A valid sbgECom handle.
* \param[in]	pEthernetConf				Poiner to a SbgEComEthernetConf struct that holds the new configuration to apply.
* \return									SBG_NO_ERROR if the command has been executed successfully.
*/
SbgErrorCode sbgEComEthernetSetConf(SbgEComHandle *pHandle, const SbgEComEthernetConf *pEthernetConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	uint8_t				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	assert(pHandle);
	assert(pEthernetConf);
		
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
		errorCode = sbgEComEthernetConfWrite(&outputStream, pEthernetConf);
		
		//
		// Send the payload if no error has occurred
		//
		if (errorCode == SBG_NO_ERROR)
		{
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ETHERNET_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));
		}

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ETHERNET_CONF, pHandle->cmdDefaultTimeOut);

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
* Get the current assigned and used IP address as well as network inforamtion.
* In opposition to sbgEComEthernetGetConf, this method will not return the Ethernet configuration.
* It will rather return the IP address currently used by the device.
* \param[in]	pHandle						A valid sbgECom handle.
* \param[out]	pEthernetConf				Poiner to a SbgEComEthernetConf struct that holds the read IP settings from the device.
* \return									SBG_NO_ERROR if the command has been executed successfully.
*/
SbgErrorCode sbgEComEthernetInfo(SbgEComHandle *pHandle, SbgEComEthernetConf *pEthernetConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32_t			trial;
	size_t				receivedSize;
	uint8_t				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	assert(pHandle);
	assert(pEthernetConf);

	//
	// Send the command three times
	//
	for (trial = 0; trial < pHandle->numTrials; trial++)
	{
		//
		// Send the command with no payload to retreive the network configuration
		//
		errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ETHERNET_INFO, NULL, 0);

		//
		// Make sure that the command has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Try to read the device answer for 500 ms
			//
			errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ETHERNET_INFO, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

			//
			// Test if we have received correctly the answer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Initialize stream buffer to read parameters
				//
				sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);
				
				//
				// Read all parameters from the payload and return any error during the parse
				//
				errorCode = sbgEComEthernetConfParse(&inputStream, pEthernetConf);

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
