/*!
 *	\file		sbgCrc.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		15 January 2013
 *
 *	\brief		This file provides CRC-32 and CRC-16 methods.
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

#ifndef SBG_CRC_H
#define SBG_CRC_H

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Types definitions                                                  -//
//----------------------------------------------------------------------//

/*!< Type used to compute a 32 bit Ethernet CRC. */
typedef uint32_t SbgCrc32;

/*!< Type used to compute a 16 bit CRC. */
typedef uint16_t SbgCrc16;

//----------------------------------------------------------------------//
//- 32 bits Ethernet CRC                                               -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the 32 bit CRC computation system.
 *	\param[in]	pInstance				Pointer on an allocated but non initialized Crc32 object.
 */
SBG_COMMON_LIB_API void sbgCrc32Initialize(SbgCrc32 *pInstance);

/*!
 *	Compute a 32 bit CRC using an Ethernet polynome.
 *	Warning: the buffer size should be at least 4 bytes long.
 *	\param[in]	pInstance				Read only pointer on a valid Crc32 object.
 *	\param[in]	pData					Read only pointer on the data buffer to compute CRC on.
 *	\param[in]	dataSize				Data size in bytes of the buffer, has to be greater or equals to 4.
 */
SBG_COMMON_LIB_API void sbgCrc32Update(SbgCrc32 *pInstance, const void *pData, size_t dataSize);

/*!
 *	Returns the computed 32 bit CRC value.
 *	\param[in]	pInstance				Read only pointer on a valid Crc32 object.
 *	\return								The computed CRC.
 */
SBG_INLINE uint32_t sbgCrc32Get(const SbgCrc32 *pInstance)
{
	return *pInstance;
}

/*!
 *	Compute a 32 Bit CRC using an Ethernet polynome.
 *	Warning: the buffer size should be at least 4 bytes long.
 *	\param[in]	pData					Read only pointer on the data buffer to compute CRC on.
 *	\param[in]	dataSize				Data size in bytes of the buffer, has to be greater or equals to 4.
 *	\return								The computed CRC.
 */
SBG_COMMON_LIB_API uint32_t sbgCrc32Compute(const void *pData, size_t dataSize);

//----------------------------------------------------------------------//
//- CRC-16 operations                                                  -//
//----------------------------------------------------------------------//

/*!
 *	Initialize the 16 bit CRC computation system.
 *	\param[in]	pInstance				Pointer on an allocated but non initialized Crc16 object.
 */
SBG_COMMON_LIB_API void sbgCrc16Initialize(SbgCrc16 *pInstance);

/*!
 *	Compute a 16 bit CRC using an the polynome 0x8408.
 *	\param[in]	pInstance				Read only pointer on a valid Crc16 object.
 *	\param[in]	pData					Read only pointer on the data buffer to compute CRC on.
 *	\param[in]	dataSize				Data size in bytes of the buffer.
 */
SBG_COMMON_LIB_API void sbgCrc16Update(SbgCrc16 *pInstance, const void *pData, size_t dataSize);

/*!
 *	Returns the computed 32 bit CRC value.
 *	\param[in]	pInstance				Read only pointer on a valid Crc16 object.
 *	\return								The computed CRC.
 */
SBG_INLINE uint16_t sbgCrc16Get(const SbgCrc16 *pInstance)
{
	return *pInstance;
}

/*!
 *	Compute a 32 Bit CRC using an the polynome 0x8408.
 *	\param[in]	pData					Read only pointer on the data buffer to compute CRC on.
 *	\param[in]	dataSize				Data size in bytes of the buffer.
 *	\return								The computed CRC.
 */
SBG_COMMON_LIB_API uint16_t sbgCrc16Compute(const void *pData, size_t dataSize);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif /* SBG_CRC_H */
