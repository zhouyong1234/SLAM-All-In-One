/*!
 *	\file		sbgEComBinaryLogDebug.h
 *  \author		SBG Systems (Alexis Guinamard)
 *	\date		03 October 2013
 *
 *	\brief		This file is used to parse received debug frames
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
#ifndef __SBG_ECOM_BINARY_LOG_DEBUG_H__
#define __SBG_ECOM_BINARY_LOG_DEBUG_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

#include "../protocol/sbgEComProtocol.h"

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for debug data.
 *
 * If debug data doesn't fit in a frame, multiple debug messages are sent
 * with different message IDs but the same internal ID.
 */
typedef struct _SbgLogDebugData
{
	uint32_t		 id;										/*!< Debug frame ID */
	uint32_t		 offset;									/*!< Offset of the debug log */
	uint32_t		 size;										/*!< Debug frame size */
	uint32_t		 totalSize;									/*!< Total size of the debug log */
	uint8_t		 data[SBG_ECOM_MAX_PAYLOAD_SIZE - 16];		/*!< Debug data */
} SbgLogDebugData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for SBG_ECOM_LOG_DEBUG_X messages and fill the corresponding structure.
 *	\param[in]	pInputStream				Input stream buffer to read the payload from.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDebugData(SbgStreamBuffer *pInputStream, SbgLogDebugData *pOutputData);

/*!
 * Write data for SBG_ECOM_LOG_DEBUG_X messages to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDebugData(SbgStreamBuffer *pOutputStream, const SbgLogDebugData *pInputData);

#endif
