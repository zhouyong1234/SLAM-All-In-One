/*!
 * \file		sbgEComGetVersion.h
 * \author		SBG Systems
 *
 * \brief		Version information.
 *
 * \section CodeCopyright Copyright Notice
 * Copyright (C) 2019, SBG Systems SAS. All rights reserved.
 *
 * This source code is intended for use only by SBG Systems SAS and
 * those that have explicit written permission to use it from
 * SBG Systems SAS.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#ifndef SBG_ECOM_GET_VERSION_H
#define SBG_ECOM_GET_VERSION_H

// sbgCommonLib headers
#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

/*!
 * Returns an integer representing the version of the sbgCom library.
 *
 * \return										An integer representing the version of the sbgCom library.<br>
 *												Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32_t sbgEComGetVersion(void);

/*!
 * Retrieve the sbgCom library version as a string (1.0.0.0).
 *
 * \return										Null terminated string that contains the sbgCom library version.
 */
const char *sbgEComGetVersionAsString(void);

#endif //SBG_ECOM_GET_VERSION_H
