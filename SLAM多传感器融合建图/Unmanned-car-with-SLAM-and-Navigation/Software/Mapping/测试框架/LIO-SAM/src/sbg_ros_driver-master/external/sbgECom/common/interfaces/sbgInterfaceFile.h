/*!
 *	\file		sbgInterfaceFile.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		01 April 2013
 *
 *	\brief		This file implements a file interface for read only operations.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2013, SBG Systems SAS. All rights reserved.
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

#ifndef SBG_INTERFACE_FILE_H
#define SBG_INTERFACE_FILE_H

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

#include "sbgInterface.h"

//----------------------------------------------------------------------//
//- Definitions                                                        -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
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
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileOpen(SbgInterface *pHandle, const char *filePath);

/*!
 *	Open a file as an interface for write only operations.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	filePath						File path to open.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileWriteOpen(SbgInterface *pHandle, const char *filePath);

/*!
 *	Destroy an interface initialized using sbgInterfaceFileOpen.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileClose(SbgInterface *pHandle);

/*!
 *	Returns the file size in bytes.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										The file size in bytes.
 */
SBG_COMMON_LIB_API size_t sbgInterfaceFileGetSize(SbgInterface *pHandle);

/*!
 *	Returns the current cursor position in the file in bytes.
 *	\param[in]	pInterface						Valid handle on an initialized interface.
 *	\return										The current cursor position in bytes.
 */
SBG_COMMON_LIB_API size_t sbgInterfaceFileGetCursor(const SbgInterface *pHandle);

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
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite);

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32_t used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead);

/*!
 * Make an interface flush all pending input or output data.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \return												SBG_NO_ERROR if successful.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileFlush(SbgInterface *pHandle);

/*!
 * Fake write function for read only interfaces
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileWriteFake(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite);

/*!
 * Fake read function for write only interfaces
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32_t used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceFileReadFake(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif /* SBG_INTERFACE_FILE_H */
