/*!
 *	\file		sbgEComBinaryLogDiag.h
 *  \author		SBG Systems
 *	\date		12 June 2019
 *
 *	\brief		Diagnostic log handling
 *
 *	\section CodeCopyright Copyright Notice
 *	Copyright (C) 2019, SBG Systems SAS. All rights reserved.
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
#ifndef SBG_ECOM_BINARY_LOG_DIAG_H
#define SBG_ECOM_BINARY_LOG_DIAG_H

// sbgCommonLib headers
#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

// Project headers
#include "../protocol/sbgEComProtocol.h"

//----------------------------------------------------------------------//
//- Constant definitions                                               -//
//----------------------------------------------------------------------//

/*!
 * Maximum size of the log string, in bytes.
 */
#define SBG_ECOM_LOG_DIAG_MAX_STRING_SIZE					(SBG_ECOM_MAX_PAYLOAD_SIZE - 6)

//----------------------------------------------------------------------//
//- Structure definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Diagnostic log structure.
 */
typedef struct _SbgLogDiagData
{
	uint32_t							 timestamp;									/*!< Timestamp, in microseconds. */
	SbgDebugLogType						 type;										/*!< Log type. */
	SbgErrorCode						 errorCode;									/*!< Error code. */
	char								 string[SBG_ECOM_LOG_DIAG_MAX_STRING_SIZE];	/*!< Log string, null-terminated. */
} SbgLogDiagData;

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for SBG_ECOM_LOG_DIAG messages and fill the corresponding structure.
 *
 *	\param[in]	pInputStream				Input stream buffer to read the payload from.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDiagData(SbgStreamBuffer *pInputStream, SbgLogDiagData *pOutputData);


/*!
 * Write data for SBG_ECOM_LOG_DIAG messages to the output stream buffer from the provided structure.
 *
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDiagData(SbgStreamBuffer *pOutputStream, const SbgLogDiagData *pInputData);

#endif
