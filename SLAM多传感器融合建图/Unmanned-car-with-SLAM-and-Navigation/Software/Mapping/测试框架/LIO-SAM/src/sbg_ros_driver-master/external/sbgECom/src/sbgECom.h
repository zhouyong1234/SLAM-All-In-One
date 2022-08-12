/*!
 *	\file		sbgECom.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		05 February 2013
 *
 *	\brief		Contains main sbgECom methods.
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

/*!
 *	\mainpage SBG Systems Enhanced Communication library documentation
 *	Welcome to the sbgECom library documentation.<br>
 *	This documentation describes all functions implemented in the sbgECom library.
 */

#ifndef __SBG_ECOM_H__
#define __SBG_ECOM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <sbgCommon.h>
#include "sbgEComIds.h"
#include "protocol/sbgEComProtocol.h"
#include "binaryLogs/sbgEComBinaryLogs.h"

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

/*!
 * Interface structure pre-definition.
 */
typedef struct _SbgEComHandle SbgEComHandle;

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

/*!
 *	Callback definition called each time a new log is received.
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	msgClass								Class of the message we have received
 *	\param[in]	msg										Message ID of the log received.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
typedef SbgErrorCode (*SbgEComReceiveLogFunc)(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Interface definition that stores methods used to communicate on the interface.
 */
struct _SbgEComHandle
{
	SbgEComProtocol				 protocolHandle;			/*!< Handle on the protocol system. */
	
	SbgEComReceiveLogFunc		 pReceiveLogCallback;		/*!< Pointer on the method called each time a new binary log is received. */
	void						*pUserArg;					/*!< Optional user supplied argument for callbacks. */
	
	uint32_t					 numTrials;					/*!< Number of trials when a command is sent (default is 3). */
	uint32_t					 cmdDefaultTimeOut;			/*!< Default time out in ms to get an answer from the device (default 500 ms). */
};

//----------------------------------------------------------------------//
//- Public methods declarations                                        -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the protocol system used to communicate with the product and return the created handle.
 *	\param[out]	pHandle							Pointer used to store the allocated and initialized sbgECom handle.
 *	\param[in]	pInterface						Interface to use for read/write operations.
 *	\return										SBG_NO_ERROR if we have initialized the protocol system.
 */
SbgErrorCode sbgEComInit(SbgEComHandle *pHandle, SbgInterface *pInterface);

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	pHandle							A valid sbgECom handle to close.
 *	\return										SBG_NO_ERROR if we have closed and released the sbgECom system.
 */
SbgErrorCode sbgEComClose(SbgEComHandle *pHandle);

/*!
 *	Try to parse one log from the input interface and then return.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\return										SBG_NO_ERROR if no error occurs during incoming log parsing.
 */
SbgErrorCode sbgEComHandleOneLog(SbgEComHandle *pHandle);

/*!
 *	Handle all incoming logs until no more log are available in the input interface.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\return										SBG_NO_ERROR if no error occurs during incoming logs parsing.
 */
SbgErrorCode sbgEComHandle(SbgEComHandle *pHandle);

/*!
 *	Define the callback that should be called each time a new binary log is received.
 *	\param[in]	pHandle							A valid sbgECom handle.
 *	\param[in]	pReceiveLogCallback				Pointer on the callback to call when a new log is received.
 *	\param[in]	pUserArg						Optional user argument that will be passed to the callback method.
 *	\return										SBG_NO_ERROR if the callback and user argument have been defined successfully.
 */
SbgErrorCode sbgEComSetReceiveLogCallback(SbgEComHandle *pHandle, SbgEComReceiveLogFunc pReceiveLogCallback, void *pUserArg);

/*!
 * Define the default number of trials that should be done when a command is send to the device as well as the time out.
 * \param[in]	pHandle							A valid sbgECom handle.
 * \parma[in]	numTrials						Number of trials when a command is sent (starting at 1).
 * \param[in]	cmdDefaultTimeOut				Default time out in milliseconds to wait to receive an answer from the device.
 */
void sbgEComSetCmdTrialsAndTimeOut(SbgEComHandle *pHandle, uint32_t numTrials, uint32_t cmdDefaultTimeOut);

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						The errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold the error string.
 */
void sbgEComErrorToString(SbgErrorCode errorCode, char errorMsg[256]);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif	/* __SBG_ECOM_H__ */

