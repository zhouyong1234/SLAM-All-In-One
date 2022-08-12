#include "sbgInterfaceSerial.h"
#include <windows.h>

//----------------------------------------------------------------------//
//- Internal methods declarations                                      -//
//----------------------------------------------------------------------//

/*!
 *	Returns the last error message for windows api calls.
 *	\param[out]	outErrorMsg					Pointer on an allocated string that can stores the windows error message.
 *	\return									The last error number.
 */
uint32 sbgGetWindowsErrorMsg(char outErrorMsg[256])
{
	DWORD	dw = GetLastError(); 
	DWORD	numCharWritten;
	LPVOID	lpMsgBuf = NULL;

	//
	// Get the error message
	//
	numCharWritten = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER|FORMAT_MESSAGE_FROM_SYSTEM|FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPTSTR)&lpMsgBuf, 0, NULL);

	//
	// Test if a message has been correctly written
	//
	if (numCharWritten > 0)
	{
		//
		// Copy the error message
		//
		strcpy_s(outErrorMsg, 256, lpMsgBuf);	
	}
	else
	{
		outErrorMsg[0] = '\0';
	}

	//
	// Release the buffer
	//
	LocalFree(lpMsgBuf);

	return dw;
}

//----------------------------------------------------------------------//
//- Operations methods declarations                                    -//
//----------------------------------------------------------------------//

/*!
 *	Initialize a serial interface for read and write operations.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	deviceName						Serial interface location (COM21 , /dev/ttys0, depending on platform).
 *	\param[in]	baudRate						Serial interface baud rate in bps.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SbgErrorCode sbgInterfaceSerialCreate(SbgInterface *pHandle, const char *deviceName, uint32 baudRate)
{
	char errorMsg[256];
	char comPortPath[32];
	COMMTIMEOUTS comTimeOut;
	DCB comState;
	uint32 deviceNum;
	HANDLE hSerialDevice;

	assert(pHandle);

	//
	// Extract device number
	//
	if (sscanf_s(deviceName, "COM%i", &deviceNum) == 1)
	{
		//
		// Build the com port path
		//
		sprintf_s(comPortPath, 32, "\\\\.\\COM%i", deviceNum);

		//
		// Init the com port
		//
		hSerialDevice = CreateFile(comPortPath, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

		//
		// Test that the port has been initialized
		//
		if (hSerialDevice != INVALID_HANDLE_VALUE)
		{
			//
			// Purge the com port
			//
			if (PurgeComm(hSerialDevice, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
			{
				//
				// Retreives current com state and com timeout
				//
				if ( (GetCommState(hSerialDevice, &comState)) && (GetCommTimeouts(hSerialDevice, &comTimeOut)) )
				{
					//
					// Define common attributes
					//
					comState.BaudRate= baudRate;
					comState.Parity= NOPARITY;
					comState.ByteSize= 8;
					comState.StopBits= ONESTOPBIT;

					//
					// Disable flow control
					//
					comState.fDsrSensitivity = false;
					comState.fOutxCtsFlow = false;
					comState.fOutxDsrFlow = false;
					comState.fOutX = false;
					comState.fInX = false;

					//
					// Define timeout attributes (0 ms read timeout)
					//
					comTimeOut.ReadIntervalTimeout = MAXDWORD;
					comTimeOut.ReadTotalTimeoutMultiplier = 0;
					comTimeOut.ReadTotalTimeoutConstant = 0;

					comTimeOut.WriteTotalTimeoutConstant = 0;
					comTimeOut.WriteTotalTimeoutMultiplier = 0;
						
					//
					// Configure the com port
					//
					if ( (SetCommState(hSerialDevice, &comState)) && (SetCommTimeouts(hSerialDevice, &comTimeOut)) )
					{
						//
						// Wait until the com port has been configured by windows
						//
						sbgSleep(60);

						//
						// Define the COM port buffer size
						//
						if (SetupComm(hSerialDevice, SBG_IF_SERIAL_RX_BUFFER_SIZE, SBG_IF_SERIAL_TX_BUFFER_SIZE))
						{
							//
							// The serial port is ready so create a new serial interface
							//
							pHandle->handle = hSerialDevice;
							pHandle->type = SBG_IF_TYPE_SERIAL;
							pHandle->pReadFunc = sbgInterfaceSerialRead;
							pHandle->pWriteFunc = sbgInterfaceSerialWrite;
								
							//
							// Purge the communication
							//
							return sbgInterfaceSerialFlush(pHandle);
						}
						else
						{
							sbgGetWindowsErrorMsg(errorMsg);
							SBG_LOG_ERROR(SBG_ERROR, "Unable to define buffer size: %s", errorMsg);
						}
					}
					else
					{
						sbgGetWindowsErrorMsg(errorMsg);
						SBG_LOG_ERROR(SBG_ERROR, "Unable to set com state and/or timeout: %s", errorMsg);
					}
				}
				else
				{
					sbgGetWindowsErrorMsg(errorMsg);
					SBG_LOG_ERROR(SBG_ERROR, "Unable to retreive com state and/or timeout: %s", errorMsg);
				}
			}
			else
			{
				sbgGetWindowsErrorMsg(errorMsg);
				SBG_LOG_ERROR(SBG_ERROR, "Unable to purge com port %i: %s", deviceNum, errorMsg);
			}

			//
			// Close the port com
			// An error occurred while setting up the serial port, close it.
			//
			CloseHandle(hSerialDevice);
		}
			
		return SBG_ERROR;
	}
	else
	{
		//
		// Invalid device name
		//
		return SBG_INVALID_PARAMETER;
	}
}

/*!
 *	Destroy an interface initialized using sbgInterfaceSerialCreate.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
SbgErrorCode sbgInterfaceSerialDestroy(SbgInterface *pHandle)
{
	HANDLE pSerialDevice;

	assert(pHandle);

	//
	// Get the internal serial handle
	//
	pSerialDevice = (HANDLE)(pHandle->handle);

	//
	// Close the port com
	//
	CloseHandle(pSerialDevice);
	pHandle->handle = NULL;

	return SBG_NO_ERROR;
}

/*!
 * Flush the RX and TX buffers (remove all old data)
 * \param[in]	handle				Valid handle on an initialized interface.
 * \return							SBG_NO_ERROR if everything is OK
 */
SbgErrorCode sbgInterfaceSerialFlush(SbgInterface *pHandle)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	HANDLE			pSerialDevice;
	char			errorMsg[256];
	uint8			dummyBuffer[256];
	size_t			numBytesRead;

	assert(pHandle);

	//
	// Get the internal serial handle
	//
	pSerialDevice = (HANDLE)(pHandle->handle);

	//
	// Flush both Rx and Tx buffers
	//
	if (PurgeComm(pSerialDevice, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
	{
		//
		// Wait because some hardware doesn't execute the flush directly
		//
		sbgSleep(5);

		//
		// Try to read as much data as possible to avoid flush issues with some hardware
		//
		do
		{
			errorCode = sbgInterfaceSerialRead(pHandle, dummyBuffer, &numBytesRead, sizeof(dummyBuffer));
		} while ( (errorCode == SBG_NO_ERROR) && (numBytesRead > 0) );

		//
		// Serial port successfully purged
		//
		return SBG_NO_ERROR;
	}
	else
	{
		//
		// Flush has failed
		//
		sbgGetWindowsErrorMsg(errorMsg);
		errorCode = SBG_ERROR;
		SBG_LOG_ERROR(SBG_ERROR, "PurgeComm failed: %s", errorMsg);			
	}

	return errorCode;
}

/*!
 * Change the serial interface baud rate immediatly.
 * \param[in]	handle				Valid handle on an initialized interface.
 * \param[in]	baudRate			The new baudrate to apply in bps.
 * \return							SBG_NO_ERROR if everything is OK
 */
SbgErrorCode sbgInterfaceSerialChangeBaudrate(SbgInterface *pHandle, uint32 baudRate)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	HANDLE			pSerialDevice;
	DCB				comState;
	char			errorMsg[256];

	assert(pHandle);

	//
	// Get the internal serial handle
	//
	pSerialDevice = (HANDLE)(pHandle->handle);

	//
	// Try to retreive current com state
	//
	if (GetCommState(pSerialDevice, &comState))
	{
		//
		// Change the baud rate
		//
		comState.BaudRate = baudRate;

		//
		// Configure the com state
		//
		if (SetCommState(pSerialDevice, &comState))
		{
			//
			// Wait until the com port has been configured by windows
			//
			sbgSleep(60);

			errorCode = SBG_NO_ERROR;
		}
		else
		{
			errorCode = SBG_ERROR;
			sbgGetWindowsErrorMsg(errorMsg);
			SBG_LOG_ERROR(errorCode, "Unable to set com state: %s", errorMsg);
				
		}
	}
	else
	{
		errorCode = SBG_ERROR;
		sbgGetWindowsErrorMsg(errorMsg);
		SBG_LOG_ERROR(errorCode, "Unable to retreive com state: %s", errorMsg);			
	}

	return errorCode;
}

//----------------------------------------------------------------------//
//- Internal interfaces write/read implementations                     -//
//----------------------------------------------------------------------//

/*!
 * Try to write some data to an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SbgErrorCode sbgInterfaceSerialWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite)
{
	DWORD numBytesLeftToWrite = (DWORD)bytesToWrite;
	uint8 *pCurrentBuffer = (uint8*)pBuffer;
	DWORD  numBytesWritten;
	HANDLE pSerialDevice;
	char errorMsg[256];

	assert(pHandle);
	assert(pBuffer);

	//
	// Get the internal serial handle
	//
	pSerialDevice = (HANDLE)(pHandle->handle);

	//
	// Write the whole buffer
	//
	while (numBytesLeftToWrite > 0)
	{
		//
		// Write these bytes to the serial interface
		//
		if (!WriteFile(pSerialDevice, pCurrentBuffer, numBytesLeftToWrite, (LPDWORD)&numBytesWritten, NULL))
		{
			//
			// An error has occured during the write
			//
			sbgGetWindowsErrorMsg(errorMsg);
			SBG_LOG_ERROR(SBG_WRITE_ERROR, "Write failed error: %s", errorMsg);
			return SBG_WRITE_ERROR;
		}

		//
		// Update the buffer pointer and the number of bytes to write
		//
		numBytesLeftToWrite -= (size_t)numBytesWritten;
		pCurrentBuffer += numBytesWritten;
	}

	return SBG_NO_ERROR;
}

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SbgErrorCode sbgInterfaceSerialRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	HANDLE pSerialDevice;
	char errorMsg[256];
	DWORD bytesRead;

	assert(pHandle);
	assert(pBuffer);
	assert(pReadBytes);

	//
	// Get the internal serial handle
	//
	pSerialDevice = (HANDLE)(pHandle->handle);

	//
	// Read some bytes on the serial buffer
	//
	if (ReadFile(pSerialDevice, pBuffer, (DWORD)bytesToRead, (LPDWORD)&bytesRead, NULL))
	{
		//
		//	Update the number of bytes read
		//
		(*pReadBytes) = (size_t)bytesRead;

		return SBG_NO_ERROR;
	}
	else
	{
		*pReadBytes = (size_t)bytesRead;

		//
		// Unable to read some bytes
		//
		sbgGetWindowsErrorMsg(errorMsg);
		SBG_LOG_ERROR(SBG_READ_ERROR, "Read failed: %s", errorMsg);
		return SBG_READ_ERROR;
	}
}
