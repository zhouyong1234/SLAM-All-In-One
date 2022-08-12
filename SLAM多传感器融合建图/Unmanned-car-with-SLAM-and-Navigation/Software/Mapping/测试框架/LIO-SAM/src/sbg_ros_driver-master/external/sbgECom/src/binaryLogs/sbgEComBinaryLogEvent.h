/*!
 *	\file		sbgEComBinaryLogEvent.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		28 October 2013
 *
 *	\brief		This file is used to parse received event markers binary logs.
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
#ifndef __SBG_ECOM_BINARY_LOG_EVENT_H__
#define __SBG_ECOM_BINARY_LOG_EVENT_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Log marker events definitions                                      -//
//----------------------------------------------------------------------//

/*!
 * Log market events status mask definitions
 */
#define	SBG_ECOM_EVENT_OVERFLOW			(0x00000001u << 0)		/*!< Set to 1 if we have received events at a higher rate than 1 kHz. */
#define	SBG_ECOM_EVENT_OFFSET_0_VALID	(0x00000001u << 1)		/*!< Set to 1 if at least two events have been received. */
#define	SBG_ECOM_EVENT_OFFSET_1_VALID	(0x00000001u << 2)		/*!< Set to 1 if at least three events have been received. */
#define	SBG_ECOM_EVENT_OFFSET_2_VALID	(0x00000001u << 3)		/*!< Set to 1 if at least four events have been received. */
#define	SBG_ECOM_EVENT_OFFSET_3_VALID	(0x00000001u << 4)		/*!< Set to 1 if at least five events have been received. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_EVENT_# message.
 */
typedef struct _SbgLogEvent
{
	uint32_t	timeStamp;					/*!< Measurement time since the sensor power up. */
	uint16_t	status;						/*!< Events status bitmask. */
	uint16_t	timeOffset0;				/*!< Time offset for the second received event. */
	uint16_t	timeOffset1;				/*!< Time offset for the third received event. */
	uint16_t	timeOffset2;				/*!< Time offset for the fourth received event. */
	uint16_t	timeOffset3;				/*!< Time offset for the fifth received event. */
} SbgLogEvent;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_EVENT_# message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseEvent(SbgStreamBuffer *pInputStream, SbgLogEvent *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_EVENT_# message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteEvent(SbgStreamBuffer *pOutputStream, const SbgLogEvent *pInputData);

#endif
