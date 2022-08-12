/*!
 *	\file		sbgCommon.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Main header file for SBG Systems common C library.
 *
 *	All files / projects that would like to use the SBG Systems common C library
 *	should include this file.
 *
 *	Be aware that this file doesn't include SBG Systems common C library components
 *	but just defines all types and environments such as error logging.
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
#ifndef SBG_COMMON_H
#define SBG_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Include user defined configuration                                 -//
//----------------------------------------------------------------------//
#include "sbgConfig.h"

//----------------------------------------------------------------------//
//- Platform default configuration                                     -//
//----------------------------------------------------------------------//

/*!
 * If set to 0, the platform support only aligned memory access.
 * If set to 1, the platform support unaligned memory access.
 * Default: Support only Aligned access - Disabled
 */
#ifndef SBG_CONFIG_UNALIGNED_ACCESS_AUTH
	#define SBG_CONFIG_UNALIGNED_ACCESS_AUTH			(0)
#endif

/*!
 * If set to 0, the platform is using little endian.
 * If set to 1, the platform is using big endian.
 * Default: Little Endian - Disabled
 */
#ifndef SBG_CONFIG_BIG_ENDIAN
	#define SBG_CONFIG_BIG_ENDIAN						(0)
#endif

//----------------------------------------------------------------------//
//- Logging default configuration                                      -//
//----------------------------------------------------------------------//

/*!
 * If set to 1, error logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_ERROR
	#define SBG_CONFIG_ENABLE_LOG_ERROR					(1)
#endif

/*!
 * If set to 1, warning logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_WARNING
	#define SBG_CONFIG_ENABLE_LOG_WARNING				(1)
#endif

/*!
 * If set to 1, information logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_INFO
	#define SBG_CONFIG_ENABLE_LOG_INFO					(1)
#endif

/*!
 * If set to 1, debug logs level will be thrown.
 * Default: Enabled
 */
#ifndef SBG_CONFIG_ENABLE_LOG_DEBUG
	#define SBG_CONFIG_ENABLE_LOG_DEBUG					(1)
#endif

/*!
 * Maximum error message size in bytes that can be generated including the NULL Char.
 * Default: 1024
 */
#ifndef SBG_CONFIG_LOG_MAX_SIZE
	#define SBG_CONFIG_LOG_MAX_SIZE					(size_t)(1024)
#endif

//----------------------------------------------------------------------//
//- File system default configuration                                  -//
//----------------------------------------------------------------------//

/*!
 * Maximum number of chars for a file name including the NULL char.
 * Default: 256
 */
#ifndef SBG_CONFIG_PATH_MAX_SIZE
	#define SBG_CONFIG_PATH_MAX_SIZE				(size_t)(256)
#endif

//----------------------------------------------------------------------//
//- Include core headers                                               -//
//----------------------------------------------------------------------//
#include "sbgTypes.h"
#include "sbgErrorCodes.h"
#include "sbgDefines.h"
#include "sbgDebug.h"
#include "sbgPlatform.h"

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif	/* SBG_COMMON_H */
