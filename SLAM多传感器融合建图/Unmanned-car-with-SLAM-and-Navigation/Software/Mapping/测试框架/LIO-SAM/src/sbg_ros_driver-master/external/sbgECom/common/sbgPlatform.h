/*!
 *	\file		sbgPlatform.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file that contains all platform specific definitions.
 *
 *	This file should be modified to each targeted platform.
 *	For example, all common headers should be included from this file.
 *	
 *	The platform endianness should be defined here.
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
#ifndef SBG_PLATFORM_H
#define SBG_PLATFORM_H

// System headers
#include <stdint.h>

// Local headers
#include "sbgDefines.h"
#include "sbgDebug.h"
#include "sbgErrorCodes.h"

//----------------------------------------------------------------------//
//- Add here any additional includes you want to share                 -//
//----------------------------------------------------------------------//

#ifdef _WIN32
#include <direct.h>
#elif defined(__unix__)
#include <unistd.h>
#endif

//----------------------------------------------------------------------//
//- Specific timing methods to reimplement for your platform           -//
//----------------------------------------------------------------------//

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
SBG_COMMON_LIB_API uint32_t sbgGetTime(void);

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
SBG_COMMON_LIB_API void sbgSleep(uint32_t ms);

//----------------------------------------------------------------------//
//- Specific logging methods to reimplement for your platform          -//
//----------------------------------------------------------------------//

/*!
 *	The method is called when one of the SBG_LOG_ERROR, SBG_LOG_WARNING, SBG_LOG_INFO or SBG_LOG_VERBOSE is called.
 *	It logs an error message with debug information and support a variable list of arguments
 *	\param[in]	pFileName					File name where the error occurred.
 *	\param[in]	pFunctionName				Function name where the error occurred.
 *	\param[in]	line						Line number where the error occurred.
 *	\param[in]	pCategory					Category for this log or "None" if no category has been specified.
 *	\param[in]	logType						Define if we have an error, a warning, an info or a debug log.
 *	\param[in]	errorCode					The error code associated with the message.
 *	\param[in]	pFormat						The error message that will be used with the variable list of arguments.
 */
SBG_COMMON_LIB_API void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...);

//----------------------------------------------------------------------//
//- Errors and warning managment                                       -//
//----------------------------------------------------------------------//

/*!
 *	Run time assert that is raised if the expression is false.
 *	\param[in]	expression						The boolean expression to test, the execution is interrupted if the expression is evaluated as false.
 */
#define SBG_PLATFORM_ASSERT(expression)		assert((expression))

#endif	/* SBG_PLATFORM_H */
