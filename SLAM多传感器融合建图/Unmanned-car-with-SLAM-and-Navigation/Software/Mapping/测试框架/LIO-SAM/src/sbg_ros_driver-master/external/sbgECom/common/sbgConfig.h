/*!
 *	\file		sbgConfig.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file used to configure the framework.
 *
 *	You can configure for example the logging system.
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
#ifndef SBG_CONFIG_H
#define SBG_CONFIG_H

//----------------------------------------------------------------------//
//- Platform specific configurations                                   -//
//----------------------------------------------------------------------//

/*!
 * Windows x86 & x64 support both aligned and unaligned access
 */
#define SBG_CONFIG_UNALIGNED_ACCESS_AUTH			(0)

/*!
 * Windows is using little endianess
 */
#define SBG_CONFIG_BIG_ENDIAN						(0)

//----------------------------------------------------------------------//
//- Debug / logging Configurations                                     -//
//----------------------------------------------------------------------//

/*!
 *	Define the error log configuration for debug and release modes.
 *	You should setup your C preprocessor to define SBG_NDEBUG in release mode.
 */
#ifdef NDEBUG
	#define SBG_CONFIG_ENABLE_ASSERT				(0)						/*!< Set to 1 to enable all assertion checks. */
	#define SBG_CONFIG_ENABLE_LOG_ERROR				(1)						/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_ERROR. */
	#define SBG_CONFIG_ENABLE_LOG_WARNING			(1)						/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_WARNING. */
	#define SBG_CONFIG_ENABLE_LOG_INFO				(0)						/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_INFO. */
	#define SBG_CONFIG_ENABLE_LOG_DEBUG				(0)						/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_DEBUG */
#else
	#define SBG_CONFIG_ENABLE_ASSERT				(1)					/*!< Set to 1 to enable all assertion checks. */
	#define SBG_CONFIG_ENABLE_LOG_ERROR				(1)					/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_ERROR. */
	#define SBG_CONFIG_ENABLE_LOG_WARNING			(1)					/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_WARNING. */
	#define SBG_CONFIG_ENABLE_LOG_INFO				(1)					/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_INFO. */
	#define SBG_CONFIG_ENABLE_LOG_VERBOSE			(1)					/*!< Set to 1 to enable log of all errors thrown using SBG_LOG_DEBUG. */
#endif

#endif	/* SBG_CONFIG_H */
