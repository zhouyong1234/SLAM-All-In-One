#include "sbgInterfaceFile.h"

//----------------------------------------------------------------------//
//- Internal methods declarations                                      -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Operations methods declarations                                    -//
//----------------------------------------------------------------------//

/*!
 *	Open a file as an interface for read only operations.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	filePath						File path to open.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileOpen(SbgInterface *pHandle, const char *filePath)
{
	SbgErrorCode	 errorCode = SBG_NO_ERROR;
	FILE			*pInputFile;
	
	assert(pHandle);
	assert(filePath);

	//
	// Always call the underlying zero init method to make sure we can correctly handle SbgInterface evolutions
	//
	sbgInterfaceZeroInit(pHandle);

	//
	// Try to open the file
	//
	pInputFile = fopen(filePath, "rb");

	//
	// Test if the input file has been opened
	//
	if (pInputFile)
	{
		//
		// Define base interface members
		//
		pHandle->handle		= pInputFile;
		pHandle->type		= SBG_IF_TYPE_FILE;

		//
		// Define the interface name
		//
		sbgInterfaceNameSet(pHandle, filePath);
				
		//
		// Define all overloaded members
		//
		pHandle->pReadFunc	= sbgInterfaceFileRead;
		pHandle->pWriteFunc	= sbgInterfaceFileWriteFake;
		pHandle->pFlushFunc	= sbgInterfaceFileFlush;
	}
	else
	{
		//
		// Unable to open the input file
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 *	Open a file as an interface for write only operations.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	filePath						File path to open.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileWriteOpen(SbgInterface *pHandle, const char *filePath)
{
	SbgErrorCode	 errorCode = SBG_NO_ERROR;
	FILE			*pInputFile;
			
	//
	//	Test input arguments
	//
	assert(pHandle);
	assert(filePath);

	//
	// Try to open the file
	//
	pInputFile = fopen(filePath, "wb");

	//
	// Test if the input file has been opened
	//
	if (pInputFile)
	{
		//
		// Define the interface members
		//
		pHandle->handle = pInputFile;
		pHandle->type = SBG_IF_TYPE_FILE;
		pHandle->pReadFunc = sbgInterfaceFileReadFake;
		pHandle->pWriteFunc = sbgInterfaceFileWrite;
		pHandle->pFlushFunc = sbgInterfaceFileFlush;
	}
	else
	{
		//
		// Unable to open the input file
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 *	Destroy an interface initialized using sbgInterfaceFileOpen.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileClose(SbgInterface *pHandle)
{
	FILE *pInputFile;

	//
	//	Test input arguments
	//
	assert(pHandle);

	//
	// Get the internal FILE handle
	//
	pInputFile = (FILE*)(pHandle->handle);

	//
	// Close the input file only if opened
	//
	if (pInputFile)
	{
		fclose(pInputFile);
		pHandle->handle = NULL;
	}

	return SBG_NO_ERROR;
}

/*!
 *	Returns the file size in bytes.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										The file size in bytes.
 */
SBG_COMMON_LIB_API size_t sbgInterfaceFileGetSize(SbgInterface *pHandle)
{
	FILE	*pInputFile;
	long	 cursorPos;
	long	 fileSize;

	//
	//	Test input arguments
	//
	assert(pHandle);

	//
	// Initialize the file size to 0 in case of error
	//
	fileSize = 0;

	//
	// Get the internal FILE handle
	//
	pInputFile = (FILE*)(pHandle->handle);

	//
	// Test if the file handle is valid
	//
	if (pInputFile)
	{
		//
		// Compute the file size
		//
		cursorPos = ftell(pInputFile);
		fseek(pInputFile, 0, SEEK_END);
		fileSize = ftell(pInputFile);
		fseek(pInputFile, cursorPos, SEEK_SET);
	}
	
	return (size_t)fileSize;
}

/*!
 *	Returns the current cursor position in the file in bytes.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										The current cursor position in bytes.
 */
SBG_COMMON_LIB_API size_t sbgInterfaceFileGetCursor(const SbgInterface *pHandle)
{
	FILE	*pInputFile;

	//
	//	Test input argument
	//
	assert(pHandle);

	//
	// Get the internal FILE handle
	//
	pInputFile = (FILE*)(pHandle->handle);

	//
	// Test if the file handle is valid
	//
	if (pInputFile)
	{
		//
		// Return the current cursor position
		//
		return (size_t)ftell(pInputFile);
	}
	
	return 0;
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
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite)
{
	FILE *pOutputFile;
	size_t bytesWritten;

	//
	//	Test input argument
	//
	assert(pHandle);
	assert(pBuffer);

	//
	// Get the internal FILE handle
	//
	pOutputFile = (FILE*)(pHandle->handle);

	//
	// Write as much bytes as we can
	//
	bytesWritten = fwrite(pBuffer, sizeof(uint8_t), bytesToWrite, pOutputFile);

	//
	// Check if we could successfuly write our bytes
	//
	if (bytesWritten == bytesToWrite)
	{

		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_WRITE_ERROR;
	}
}

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32_t used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	FILE *pInputFile;

	//
	//	Test input argument
	//
	assert(pHandle);
	assert(pBuffer);
	assert(pReadBytes);

	//
	// Get the internal FILE handle
	//
	pInputFile = (FILE*)(pHandle->handle);

	//
	// Read some bytes from the file
	//
	*pReadBytes = fread(pBuffer, sizeof(uint8_t), bytesToRead, pInputFile);

	return SBG_NO_ERROR;
}

/*!
 * Make an interface flush all pending input or output data.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \return												SBG_NO_ERROR if successful.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileFlush(SbgInterface *pHandle)
{
	SbgErrorCode	 errorCode;
	FILE			*pInputFile;
	int				 ret;

	//
	//	Test input argument
	//
	assert(pHandle);

	//
	// Get the internal FILE handle
	//
	pInputFile = (FILE*)(pHandle->handle);

	//
	// Flush all pending data
	//
	ret = fflush(pInputFile);

	if (ret == 0)
	{
		errorCode = SBG_NO_ERROR;
	}
	else
	{
		errorCode = SBG_ERROR;
	}

	return errorCode;
}

/*!
 * Fake write function for read only interfaces
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileWriteFake(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite)
{
	//
	// Simply avoid warnings and return an error
	//
	SBG_UNUSED_PARAMETER(pHandle);
	SBG_UNUSED_PARAMETER(pBuffer);
	SBG_UNUSED_PARAMETER(bytesToWrite);

	return SBG_ERROR;
}

/*!
 * Fake read function for write only interfaces
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32_t used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileReadFake(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	//
	// Simply avoid warnings and return an error
	//
	SBG_UNUSED_PARAMETER(pHandle);
	SBG_UNUSED_PARAMETER(pBuffer);
	SBG_UNUSED_PARAMETER(pReadBytes);
	SBG_UNUSED_PARAMETER(bytesToRead);

	return SBG_ERROR;
}
