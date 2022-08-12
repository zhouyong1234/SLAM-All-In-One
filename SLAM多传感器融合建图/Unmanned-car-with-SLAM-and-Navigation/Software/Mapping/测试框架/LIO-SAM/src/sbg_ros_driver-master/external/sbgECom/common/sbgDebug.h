/*!
 *	\file		sbgDebug.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Define and handle error logging for the SBG Systems common C library.
 *
 *	The methods defined here should be implemented in sbgPlatform.h/sbgPlatform.c
 *	according to your platform and needs.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2015, SBG Systems SAS. All rights reserved.
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
#ifndef SBG_DEBUG_H
#define SBG_DEBUG_H

// Local headers
#include "sbgDefines.h"

#ifndef SBG_DEBUG_LOG_CATEGORY
	#define SBG_DEBUG_LOG_CATEGORY		("None")
#endif

//----------------------------------------------------------------------//
//- Errors and warning definitions                                     -//
//----------------------------------------------------------------------//

/*!
 *	Enum that identify the type of error / warning that has been thrown.
 */
typedef enum _SbgDebugLogType
{
	SBG_DEBUG_LOG_TYPE_ERROR,					/*!< The message to log is an error. */
	SBG_DEBUG_LOG_TYPE_WARNING,					/*!< The message to log is a warning. */
	SBG_DEBUG_LOG_TYPE_INFO,					/*!< The message to log is an information. */
	SBG_DEBUG_LOG_TYPE_DEBUG					/*!< The message to log is a debug information. */
} SbgDebugLogType;

//----------------------------------------------------------------------//
//- Errors and warning macros                                          -//
//----------------------------------------------------------------------//

/*!
 *	Log an error with its associated message.
 *	\param[in]	errorCode						The error code that has thrown this error.
 *	\param[in]	format							String litteral for the associated error message (you can use printf like string formating).
 */
#define SBG_LOG_ERROR_CALL(errorCode, format, ...)		sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_ERROR, errorCode, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_ERROR == 1
	#define	SBG_LOG_ERROR								SBG_LOG_ERROR_CALL
#else
	#define SBG_LOG_ERROR(format, ...)					((void)sizeof(SBG_LOG_ERROR_CALL(format, ## __VA_ARGS__)))
#endif

/*!
 *	Log a warning with its associated message.
 *	\param[in]	errorCode						The error code that has thrown this warning.
 *	\param[in]	format							String litteral for the associated warning message (you can use printf like string formating).
 */
#define SBG_LOG_WARNING_CALL(errorCode, format, ...)	sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_WARNING, errorCode, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_WARNING == 1
	#define	SBG_LOG_WARNING								SBG_LOG_WARNING_CALL
#else
	#define SBG_LOG_WARNING(format, ...)				((void)sizeof(SBG_LOG_WARNING_CALL(format, ## __VA_ARGS__)))
#endif

/*!
 *	Log an information message.
 *	\param[in]	format							String litteral for the information message (you can use printf like string formating).
 */
#define SBG_LOG_INFO_CALL(format, ...)					sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_INFO, SBG_NO_ERROR, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_INFO == 1
	#define	SBG_LOG_INFO								SBG_LOG_INFO_CALL
#else
	#define SBG_LOG_INFO(format, ...)					((void)sizeof(SBG_LOG_INFO_CALL(format, ## __VA_ARGS__)))
#endif

/*!
 *	Log an information message only in debug mode
 *	\param[in]	format							String litteral for the information message (you can use printf like string formating).
 */
#define SBG_LOG_DEBUG_CALL(format, ...)					sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_CATEGORY, SBG_DEBUG_LOG_TYPE_DEBUG, SBG_NO_ERROR, format, ##__VA_ARGS__)

#if SBG_CONFIG_ENABLE_LOG_DEBUG == 1
	#define	SBG_LOG_DEBUG								SBG_LOG_DEBUG_CALL
#else
	#define SBG_LOG_DEBUG(format, ...)					((void)sizeof(SBG_LOG_DEBUG_CALL(format, ## __VA_ARGS__)))
#endif

//----------------------------------------------------------------------//
//- Inline functions                                                   -//
//----------------------------------------------------------------------//

/*!
 * Convert a log type into a string representation.
 *
 * \param[in]	logType						Log type.
 * \return									String representation for the given log type.
 */
SBG_INLINE const char *sbgDebugLogTypeToStr(SbgDebugLogType logType)
{
	const char				*pString;

	switch (logType)
	{
	case SBG_DEBUG_LOG_TYPE_ERROR:
		pString = "error";
		break;
	case SBG_DEBUG_LOG_TYPE_WARNING:
		pString = "warning";
		break;
	case SBG_DEBUG_LOG_TYPE_INFO:
		pString = "info";
		break;
	case SBG_DEBUG_LOG_TYPE_DEBUG:
		pString = "debug";
		break;
	default:
		pString = "unknown";
		break;
	}

	return pString;
}

#endif	/* SBG_DEBUG_H */
