/*!
 *	\file		sbgInterface.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		10 December 2012
 *
 *	\brief		This file implements the base interface for all Serial and Ethernet ports.
 *
 *	An interface is used to provide a common API for both serial and ethernet ports.
 *	An interface can be opened/closed and some data can be written or read from it.
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

#ifndef SBG_INTERFACE_H
#define SBG_INTERFACE_H

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

/* sbgCommonLib headers */
#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Constant definitions                                               -//
//----------------------------------------------------------------------//

#define SBG_IF_NAME_MAX_SIZE		(48)			/*!< Maximum size in bytes for the interface name string */

/*!
 * Type values reserved for standard interface types.
 */
#define SBG_IF_TYPE_UNKNOW			(0)				/*!< The interface type is not defined. */
#define SBG_IF_TYPE_SERIAL			(1)				/*!< The interface is a serial com port. */
#define SBG_IF_TYPE_ETH_UDP			(2)				/*!< The interface is an UDP one. */
#define SBG_IF_TYPE_ETH_TCP_IP		(3)				/*!< The interface is an TCP/IP one. */
#define SBG_IF_TYPE_FILE			(4)				/*!< The interface is a file. */
#define SBG_IF_TYPE_LAST_RESERVED	(999)			/*!< Last reserved value for standard types. */

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

/*!
 * Interface structure pre-definition.
 */
typedef struct _SbgInterface SbgInterface;

/*!
 * Handle that stores the internal interface handle (ie Serial or Ethernet)
 */
typedef void* SbgInterfaceHandle;

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Method to implement to write a buffer to an interface.
 *
 * This method should return an error only if all bytes were not written successfully.
 * If you try to write zero byte, the method shouldn't return any error.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write (can be zero).
 * \return												SBG_NO_ERROR if exactly bytesToWrite have been written successfully.
 */
typedef SbgErrorCode (*SbgInterfaceWriteFunc)(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite);

/*!
 * Method to implement to read data from an interface.
 *
 * This method returns an error only if there is a 'low level' error on the interface.
 * If no byte is read at all or less bytes than bytesToRead, this method returns SBG_NO_ERROR.
 * You have to check pReadBytes field to know the number of bytes actually read.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Returns the number of bytes actually read (can be zero and up to bytesToRead).
 * \param[in]	bytesToRead								Maximum number of bytes to try to read on the interface.
 * \return												SBG_NO_ERROR if zero or some bytes have been read successfully.
 */
typedef SbgErrorCode (*SbgInterfaceReadFunc)(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead);

/*!
 * Make an interface flush all pending input or output data.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \return												SBG_NO_ERROR if successful.
 */
typedef SbgErrorCode (*SbgInterfaceFlushFunc)(SbgInterface *pHandle);

/*!
 * Compute and return the delay needed by the interface to transmit / receive X number of bytes.
 *
 * This method isn't applicable to all interfaces and as such should return a default delay of zero.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	numBytes								The number of bytes to transmit / receive to evaluate the needed delay.
 * \return												The expected delay in us needed to transmit / receive the specified number of bytes or 0 if not applicable.
 */
typedef uint32_t (*SbgInterfaceGetDelayFunc)(SbgInterface *pHandle, size_t numBytes);

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Interface definition that stores methods used to communicate on the interface.
 *
 * The interface class is designed to allow custom user implementations. The type member stores
 * a type identifier allowing the identification of the underlying type, including custom
 * implementations. Standard interfaces provided by this library use types from 1 up to
 * and including SBG_IF_TYPE_LAST_RESERVED. Greater values are intended to identify custom
 * types that are normally specific to the project using this library. The value 0 identifies
 * an unknown interface type, usually indicating that the interface was not correctly initialized.
 */
struct _SbgInterface
{
	SbgInterfaceHandle			 handle;							/*!< Internal interface handle used to access the media. */
	uint32_t					 type;								/*!< Opaque interface type. */
	char						 name[SBG_IF_NAME_MAX_SIZE];		/*!< The interface name as passed during the creation */

	SbgInterfaceWriteFunc		 pWriteFunc;						/*!< Mandatory method used to write some data to this interface. */
	SbgInterfaceReadFunc		 pReadFunc;							/*!< Mandatory method used to read some data to this interface. */
	SbgInterfaceFlushFunc		 pFlushFunc;						/*!< Optional method used to make this interface flush all pending data. */
	SbgInterfaceGetDelayFunc	 pDelayFunc;						/*!< Optional method used to compute an expected delay to transmit/receive X bytes */
};

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Initialize an interface instance to zero.
 *
 * \param[in]	pHandle									Handle on an allocated interface to initialize to zero.
 */
SBG_COMMON_LIB_API void sbgInterfaceZeroInit(SbgInterface *pHandle);

/*!
 * Write some data to an interface.
 *
 * This method should return an error only if all bytes were not written successfully.
 * If you try to write zero byte, the method shouldn't return any error.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write (can be zero).
 * \return												SBG_NO_ERROR if exactly bytesToWrite have been written successfully.
 */
SBG_INLINE SbgErrorCode sbgInterfaceWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite)
{
	assert(pHandle);
	assert(pBuffer);
	assert(pHandle->pWriteFunc);

	//
	// Call the correct write method according to the interface
	//
	return pHandle->pWriteFunc(pHandle, pBuffer, bytesToWrite);
}

/*!
 * Try to read some data from an interface.
 *
 * This method returns an error only if there is a 'low level' error on the interface.
 * If no byte is read at all or less bytes than bytesToRead, this method returns SBG_NO_ERROR.
 * You have to check pReadBytes field to know the number of bytes actually read.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Returns the number of bytes actually read (can be zero and up to bytesToRead).
 * \param[in]	bytesToRead								Maximum number of bytes to try to read on the interface.
 * \return												SBG_NO_ERROR if zero or some bytes have been read successfully.
 */
SBG_INLINE SbgErrorCode sbgInterfaceRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	assert(pHandle);
	assert(pBuffer);
	assert(pReadBytes);
	assert(pHandle->pReadFunc);

	//
	// Call the correct read method according to the interface
	//
	return pHandle->pReadFunc(pHandle, pBuffer, pReadBytes, bytesToRead);
}

/*!
 * Make an interface flush all pending input or output data.
 *
 * This method isn't applicable to all interfaces and as such if no implementation is provided it does nothing.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \return												SBG_NO_ERROR if successful.
 */
SBG_INLINE SbgErrorCode sbgInterfaceFlush(SbgInterface *pHandle)
{
	SbgErrorCode		 errorCode;

	assert(pHandle);

	//
	// The Flush method is optional so check if it has been defined.
	//
	if (pHandle->pFlushFunc)
	{
		//
		// Call the correct flush method according to the interface
		//
		errorCode = pHandle->pFlushFunc(pHandle);
	}
	else
	{
		errorCode = SBG_NO_ERROR;
	}

	return errorCode;
}

/*!
 * Compute and return the delay needed by the interface to transmit / receive X number of bytes.
 *
 * This method isn't applicable to all interfaces and as such if no implementation is provided a default delay of zero.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	numBytes								The number of bytes to transmit / receive to evaluate the needed delay.
 * \return												The expected delay in us needed to transmit / receive the specified number of bytes or 0 if not applicable.
 */
SBG_INLINE uint32_t sbgInterfaceGetDelay(SbgInterface *pHandle, size_t numBytes)
{
	assert(pHandle);

	//
	// The get delay method is optional so check if it has been defined.
	//
	if (pHandle->pDelayFunc)
	{
		//
		// Call the correct flush method according to the interface
		//
		return pHandle->pDelayFunc(pHandle, numBytes);
	}
	
	return 0;
}

/*!
 * Returns the interface type.
 *
 * \param[in]	pInterface								Interface instance
 * \return												The interface type.
 */
SBG_INLINE uint32_t sbgInterfaceTypeGet(const SbgInterface *pInterface)
{
	assert(pInterface);

	return pInterface->type;
}

/*!
 * Returns the interface as string.
 *
 * \param[in]	pInterface								Interface instance
 * \return												The interface type.
 */
SBG_COMMON_LIB_API const char *sbgInterfaceTypeGetAsString(const SbgInterface *pInterface);

/*!
 * Returns the interface name string.
 *
 * \param[in]	pInterface								Interface instance
 * \return												The interface name as a NULL terminated C string.
 */
SBG_INLINE const char *sbgInterfaceNameGet(const SbgInterface *pInterface)
{
	assert(pInterface);

	return pInterface->name;
}

/*!
 * Define the interface name as a NULL terminated C string.
 *
 * This method make sure that the provided string will always fit within
 * the allocated name buffer.
 *
 * If the interface name you would like to set is too long, only the end
 * of the string will be kept.
 *
 * \param[in]	pInterface								Interface instance
 * \param[in]	pName									The interface name to set as a NULL terminated C string												
 */
SBG_COMMON_LIB_API void sbgInterfaceNameSet(SbgInterface *pInterface, const char *pName);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif /* SBG_INTERFACE_H */
