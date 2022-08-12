/*!
 *	\file		sbgErrorCodes.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file that defines all error codes for SBG Systems libraries.
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
#ifndef SBG_ERROR_CODES_H
#define SBG_ERROR_CODES_H

// Local headers
#include "sbgDefines.h"

//----------------------------------------------------------------------//
//- Errors code definitions                                            -//
//----------------------------------------------------------------------//

/*!
 *	Generic errors definitions for SBG Systems projects.
 */
typedef enum _SbgErrorCode
{
	SBG_NO_ERROR = 0,						/*!< The operation was successfully executed. */
	SBG_ERROR,								/*!< We have a generic error. */
	SBG_NULL_POINTER,						/*!< A pointer is null. */
	SBG_INVALID_CRC,						/*!< The received frame has an invalid CRC. */
	SBG_INVALID_FRAME,						/*!< The received frame is invalid <br> */
											/*!<	We have received an unexpected frame (not the cmd we are waiting for or with an invalid data size.<br> */
											/*!<	This could be caused by a desync between questions and answers.<br> */
											/*!<	You should flush the serial port to fix this. */
	SBG_TIME_OUT,							/*!< We have started to receive a frame but not the end. */
	SBG_WRITE_ERROR,						/*!< All bytes hasn't been written. */
	SBG_READ_ERROR,							/*!< All bytes hasn't been read. */
	SBG_BUFFER_OVERFLOW,					/*!< A buffer is too small to contain so much data. */
	SBG_INVALID_PARAMETER,					/*!< An invalid parameter has been found. */
	SBG_NOT_READY,							/*!< A device isn't ready (Rx isn't ready for example). */
	SBG_MALLOC_FAILED,						/*!< Failed to allocate a buffer. */
	SGB_CALIB_MAG_NOT_ENOUGH_POINTS,		/*!< Not enough points were available to perform magnetometers calibration. */
	SBG_CALIB_MAG_INVALID_TAKE,				/*!< The calibration procedure could not be properly executed due to insufficient precision. */
	SBG_CALIB_MAG_SATURATION,				/*!< Saturation were detected when attempt to calibrate magnetos. */
	SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE,	/*!< 2D calibration procedure could not be performed. */

	SBG_DEVICE_NOT_FOUND,					/*!< A device couldn't be founded or opened PC only error code */
	SBG_OPERATION_CANCELLED,				/*!< An operation was canceled.  PC only error code*/
	SBG_NOT_CONTINUOUS_FRAME,				/*!< We have received a frame that isn't a continuous one. PC only error code*/

	SBG_INCOMPATIBLE_HARDWARE,				/*!< Hence valid; the command cannot be executed because of hardware incompatibility */
	SBG_INVALID_VERSION						/*!< Incompatible version */
} SbgErrorCode;

//----------------------------------------------------------------------//
//- Error codes to string litteral conversion                          -//
//----------------------------------------------------------------------//

/*!
 * According to an error code, returns a human readable string.
 * \param[in]	errorCode					The errorCode to convert to a string.
 * \return									Read only corresponding string.
 */
static inline const char *sbgErrorCodeToString(SbgErrorCode errorCode)
{
	/*!
	 *	Array of string litterals that should be exactly ordered as the SbgErrorCode enum.
	 */
	static const char *sbgErrorCodeString[] =
	{
		"SBG_NO_ERROR",
		"SBG_ERROR",
		"SBG_NULL_POINTER",
		"SBG_INVALID_CRC",
		"SBG_INVALID_FRAME",
		"SBG_TIME_OUT",
		"SBG_WRITE_ERROR",
		"SBG_READ_ERROR",
		"SBG_BUFFER_OVERFLOW",
		"SBG_INVALID_PARAMETER",
		"SBG_NOT_READY",
		"SBG_MALLOC_FAILED",
		"SGB_CALIB_MAG_NOT_ENOUGH_POINTS",
		"SBG_CALIB_MAG_INVALID_TAKE",
		"SBG_CALIB_MAG_SATURATION",
		"SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE",
		"SBG_DEVICE_NOT_FOUND",
		"SBG_OPERATION_CANCELLED",
		"SBG_NOT_CONTINUOUS_FRAME",
		"SBG_INCOMPATIBLE_HARDWARE",
		"SBG_INVALID_VERSION"
	};

	assert(errorCode < SBG_ARRAY_SIZE(sbgErrorCodeString));
	return sbgErrorCodeString[errorCode];
}

#endif	/* SBG_ERROR_CODES_H */
