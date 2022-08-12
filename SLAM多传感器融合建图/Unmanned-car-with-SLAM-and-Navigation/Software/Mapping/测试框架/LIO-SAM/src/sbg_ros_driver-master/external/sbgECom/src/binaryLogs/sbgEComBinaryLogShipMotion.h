/*!
 *	\file		sbgEComBinaryLogShipMotion.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		30 March 2013
 *
 *	\brief		This file is used to parse received ship motion binary logs.
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
#ifndef __SBG_ECOM_BINARY_LOG_SHIP_MOTION_H__
#define __SBG_ECOM_BINARY_LOG_SHIP_MOTION_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Heave status definitions                                           -//
//----------------------------------------------------------------------//

#define	SBG_ECOM_HEAVE_VALID				(0x0001u << 0)			/*!< Set to 1 after heave convergence time. */
#define SBG_ECOM_HEAVE_VEL_AIDED			(0x0001u << 1)			/*!< Set to 1 if heave output is compensated for transient accelerations. */
#define SBG_ECOM_HEAVE_SURGE_SWAY_INCLUDED	(0x0001u << 2)			/*!< Set to 1 if surge and sway channels are provided in this output. */
#define SBG_ECOM_HEAVE_PERIOD_INCLUDED		(0x0001u << 3)			/*!< Set to 1 if the heave period is provided in this output. */
#define SBG_ECOM_HEAVE_PERIOD_VALID			(0x0001u << 4)			/*!< Set to 1 if the returned heave period is assumed to be valid. */
#define SBG_ECOM_HEAVE_SWELL_MODE			(0x0001u << 5)			/*!< Set to 1 if the real time heave filter is using the swell mode computations. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP message. <br>
 * The data are expressed in the standard NED Ekinox coordiante frame. 
 * Surge is positive forward, sway is positive right and heave is positive down. <br>
 * Note that status flag should be read before using the different parameters because it will provide validity information
 * about all included outputs. Some frames may not provide the heave period or surge/sway axes for example
 */
typedef struct _SbgLogShipMotionData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< Ship Motion data status bitmask */
	float	mainHeavePeriod;			/*!< Main heave period in seconds. */
	float	shipMotion[3];				/*!< Surge, sway and heave in meters. */
	float	shipAccel[3];				/*!< Surge, sway and heave ship Acceleration in m.s^-2. */
	float	shipVel[3];					/*!< Surge, sway and heave velocities */
} SbgLogShipMotionData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP  message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseShipMotionData(SbgStreamBuffer *pInputStream, SbgLogShipMotionData *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteShipMotionData(SbgStreamBuffer *pOutputStream, const SbgLogShipMotionData *pInputData);

#endif
