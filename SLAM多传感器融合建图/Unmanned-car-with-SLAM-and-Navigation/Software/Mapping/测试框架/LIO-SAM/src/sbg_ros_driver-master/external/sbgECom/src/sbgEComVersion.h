/*!
 *	\file		sbgEComVersion.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		05 February 2013
 *
 *	\brief		Header file that contains all versions related information such as change log.
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
#ifndef SBG_E_COM_VERSION_H
#define SBG_E_COM_VERSION_H

#include "sbgCommon.h"

//----------------------------------------------------------------------//
//- Version definitions                                                -//
//----------------------------------------------------------------------//

#define SBG_E_COM_VERSION_MAJOR			1
#define SBG_E_COM_VERSION_MINOR			11
#define SBG_E_COM_VERSION_REV			920
#define SBG_E_COM_VERSION_BUILD			SBG_VERSION_QUALIFIER_STABLE

#define SBG_E_COM_VERSION				SBG_VERSION_SOFTWARE(SBG_E_COM_VERSION_MAJOR,SBG_E_COM_VERSION_MINOR,SBG_E_COM_VERSION_REV,SBG_E_COM_VERSION_BUILD)

/*
 * Backward compatibility macro definitions.
 */
 #ifndef SBG_STR
	#define SBG_STR(X)		#X
#endif
#ifndef SBG_ASSTR
	#define SBG_ASSTR(X)	SBG_STR(X)
#endif
#define SBG_E_COM_VERSION_STR			SBG_ASSTR(SBG_E_COM_VERSION_MAJOR) "." SBG_ASSTR(SBG_E_COM_VERSION_MINOR) "." SBG_ASSTR(SBG_E_COM_VERSION_REV) "-stable\0"

#endif /* SBG_E_COM_VERSION_H */
