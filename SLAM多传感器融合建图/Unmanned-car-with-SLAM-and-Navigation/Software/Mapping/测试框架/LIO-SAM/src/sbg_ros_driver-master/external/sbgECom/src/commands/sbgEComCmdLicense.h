/*!
 *	\file		sbgEComCmdLicense.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		25 February 2015
 *
 *	\brief		This file implements SbgECom commands related to licenses.
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
#ifndef __SBG_ECOM_CMD_LICENSE_H__
#define __SBG_ECOM_CMD_LICENSE_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- License commands                                                   -//
//----------------------------------------------------------------------//

/*!
 *	Upload and apply a new license to a device.
 *	The device will reboot automatically to use the new license.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the license.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdLicenseApply(SbgEComHandle *pHandle, const void *pBuffer, size_t size);

#endif
