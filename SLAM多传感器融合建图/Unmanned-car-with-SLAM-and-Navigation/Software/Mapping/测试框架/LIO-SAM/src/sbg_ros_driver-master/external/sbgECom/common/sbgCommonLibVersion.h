/*!
 *	\file		sbgCommonLibVersion.h
 *	\author		SBG Systems (Rémi Burtin)
 *	\date		28/09/2015
 *
 *	\brief		Header file that contains all versions related information.
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
#ifndef __SBG_COMMON_LIB_VERSION_H__
#define __SBG_COMMON_LIB_VERSION_H__

#include <version\sbgVersion.h>

//----------------------------------------------------------------------//
//- Version definitions                                                -//
//----------------------------------------------------------------------//

#define SBG_COMMON_LIB_VERSION_MAJOR			1
#define SBG_COMMON_LIB_VERSION_MINOR			1
#define SBG_COMMON_LIB_VERSION_BUILD			174
#define SBG_COMMON_LIB_VERSION_QUALIFIER		SBG_VERSION_QUALIFIER_ALPHA

#define SBG_COMMON_LIB_VERSION					SBG_VERSION_SOFTWARE(SBG_COMMON_LIB_VERSION_MAJOR,SBG_COMMON_LIB_VERSION_MINOR,SBG_COMMON_LIB_VERSION_BUILD,SBG_COMMON_LIB_VERSION_QUALIFIER)

/*
 * Backward compatibility macro definitions.
 */
#ifndef SBG_STR
	#define SBG_STR(X)		#X
#endif
#ifndef SBG_ASSTR
	#define SBG_ASSTR(X)	SBG_STR(X)
#endif
#define SBG_COMMON_LIB_VERSION_STR				SBG_ASSTR(SBG_COMMON_LIB_VERSION_MAJOR) "." SBG_ASSTR(SBG_COMMON_LIB_VERSION_MINOR) "." SBG_ASSTR(SBG_COMMON_LIB_VERSION_BUILD) "\0"

#endif
