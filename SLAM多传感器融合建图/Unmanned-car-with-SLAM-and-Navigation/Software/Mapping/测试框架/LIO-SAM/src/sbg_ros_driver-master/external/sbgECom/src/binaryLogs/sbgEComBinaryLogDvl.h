/*!
 *	\file		sbgEComBinaryLogDvl.h
 *  \author		SBG Systems
 *	\date		05 June 2013
 *
 *	\brief		This file is used to parse received DVL binary logs.
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
#ifndef __SBG_ECOM_BINARY_LOG_DVL_H__
#define __SBG_ECOM_BINARY_LOG_DVL_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Log DVL status definitions                                         -//
//----------------------------------------------------------------------//

/*!
 * DVL status mask definitions
 */
#define	SBG_ECOM_DVL_VELOCITY_VALID		(0x0001u << 0)			/*!< Set to 1 if the DVL equipment was able to measure a valid velocity. */
#define SBG_ECOM_DVL_TIME_SYNC			(0x0001u << 1)			/*!< Set to 1 if the DVL data is correctly synchronized. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Log structure for DVL data.
 */
typedef struct _SbgLogDvlData
{
	uint32_t	timeStamp;				/*!< Time in us since the sensor power up. */
	uint16_t	status;					/*!< DVL status bitmask. */
	float	velocity[3];			/*!< X, Y, Z velocities in m.s^-1 expressed in the DVL instrument frame. */
	float	velocityQuality[3];		/*!< X, Y, Z velocities quality indicators as provided by the DVL sensor and expressed in m.s^-1.
										 WARNING: This is typically just a residual information and not a real standard deviation. */
} SbgLogDvlData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK / SBG_ECOM_LOG_DVL_WATER_TRACK message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDvlData(SbgStreamBuffer *pInputStream, SbgLogDvlData *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_DVL_BOTTOM_TRACK / SBG_ECOM_LOG_DVL_WATER_TRACK message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDvlData(SbgStreamBuffer *pOutputStream, const SbgLogDvlData *pInputData);

#endif
