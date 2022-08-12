/*!
 *	\file		sbgEComBinaryLogUtc.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		20 February 2013
 *
 *	\brief		This file is used to parse received UTC binary logs.
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
#ifndef __SBG_ECOM_BINARY_LOG_UTC_H__
#define __SBG_ECOM_BINARY_LOG_UTC_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Clock status definitions                                           -//
//----------------------------------------------------------------------//

/*!
 * Clock status and UTC time status definitions.
 */
#define SBG_ECOM_CLOCK_STATUS_SHIFT			(1u)					/*!< Shift used to extract the clock status part. */
#define SBG_ECOM_CLOCK_STATUS_MASK			(0x000Fu)				/*!< Mask used to keep only the clock status part. */
#define SBG_ECOM_CLOCK_UTC_STATUS_SHIFT		(6u)					/*!< Shift used to extract the clock UTC status part. */
#define SBG_ECOM_CLOCK_UTC_STATUS_MASK		(0x000Fu)				/*!< Mask used to keep only the clock UTC status part. */

/*!
 * Clock status mask definitions.
 */
#define SBG_ECOM_CLOCK_STABLE_INPUT			(0x0001u << 0)			/*!< Set to 1 if a stable input clock could be used to synchronized the internal clock. */
#define SBG_ECOM_CLOCK_UTC_SYNC				(0x0001u << 5)			/*!< The UTC time is synchronized with a PPS. */

/*!
 * Clock status enum.
 */
typedef enum _SbgEComClockStatus
{
	SBG_ECOM_CLOCK_ERROR			= 0,							/*!< An error has occurred on the clock estimation. */
	SBG_ECOM_CLOCK_FREE_RUNNING		= 1,							/*!< The clock is only based on the internal crystal. */
	SBG_ECOM_CLOCK_STEERING			= 2,							/*!< A PPS has been detected and the clock is converging to it. */
	SBG_ECOM_CLOCK_VALID			= 3								/*!< The clock has converged to the PPS and is within 500ns. */
} SbgEComClockStatus;

/*!
 * Status for the UTC time data.
 */
typedef enum _SbgEComClockUtcStatus
{
	SBG_ECOM_UTC_INVALID			= 0,							/*!< The UTC time is not known, we are just propagating the UTC time internally. */
	SBG_ECOM_UTC_NO_LEAP_SEC		= 1,							/*!< We have received valid UTC time information but we don't have the leap seconds information. */
	SBG_ECOM_UTC_VALID				= 2								/*!< We have received valid UTC time data with valid leap seconds. */
} SbgEComClockUtcStatus;

//----------------------------------------------------------------------//
//- Clock status helpers methods                                       -//
//----------------------------------------------------------------------//

/*!
 * Method used to read the clock status from a status field.
 * \param[in]	status				Status field to extract the clock status from it.
 * \return							The extracted clock status.
 */
SBG_INLINE SbgEComClockStatus sbgEComLogUtcGetClockStatus(uint16_t status)
{
	return (SbgEComClockStatus)((status >> SBG_ECOM_CLOCK_STATUS_SHIFT) & SBG_ECOM_CLOCK_STATUS_MASK);
}

/*!
 * Method used to read the UTC time status from a clock status field.
 * \param[in]	status				Status field to extract the UTC time status from it.
 * \return							The extracted UTC time status.
 */
SBG_INLINE SbgEComClockUtcStatus sbgEComLogUtcGetClockUtcStatus(uint16_t status)
{
	return (SbgEComClockUtcStatus)((status >> SBG_ECOM_CLOCK_UTC_STATUS_SHIFT) & SBG_ECOM_CLOCK_UTC_STATUS_MASK);
}

/*!
 * Method used to write the clock status field.
 * \param[in]	clockStatus			The clock status to set.
 * \param[in]	utcStatus			The UTC time status to set.
 * \param[in]	masks				Bit mask to set.
 * \return							The build clock status field.
 */
SBG_INLINE uint16_t sbgEComLogUtcBuildClockStatus(SbgEComClockStatus clockStatus, SbgEComClockUtcStatus utcStatus, uint16_t masks)
{
	//
	// Create the combined status field
	//
	return	((((uint16_t)clockStatus)&SBG_ECOM_CLOCK_STATUS_MASK) << SBG_ECOM_CLOCK_STATUS_SHIFT) |
			((((uint16_t)utcStatus)&SBG_ECOM_CLOCK_UTC_STATUS_MASK) << SBG_ECOM_CLOCK_UTC_STATUS_SHIFT) | masks;
}

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_UTC_TIME message.
 */
typedef struct _SbgLogUtcData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< UTC time and clock status information */
	uint16_t	year;						/*!< Year for example: 2013. */
	int8_t	month;						/*!< Month in year [1 .. 12]. */
	int8_t	day;						/*!< Day in month [1 .. 31]. */
	int8_t	hour;						/*!< Hour in day [0 .. 23]. */
	int8_t	minute;						/*!< Minute in hour [0 .. 59]. */
	int8_t	second;						/*!< Second in minute [0 .. 60]. (60 is used only when a leap second is added) */
	int32_t	nanoSecond;					/*!< Nanosecond of current second in ns. */
	uint32_t	gpsTimeOfWeek;				/*!< GPS time of week in ms. */
} SbgLogUtcData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_UTC_DATA message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseUtcData(SbgStreamBuffer *pInputStream, SbgLogUtcData *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_UTC_DATA message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteUtcData(SbgStreamBuffer *pOutputStream, const SbgLogUtcData *pInputData);

#endif
