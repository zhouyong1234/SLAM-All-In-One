/*!
 *	\file		sbgEComBinaryLogMag.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		12 March 2013
 *
 *	\brief		This file is used to parse received magnetometer binary logs.
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
#ifndef __SBG_ECOM_BINARY_LOG_MAG_H__
#define __SBG_ECOM_BINARY_LOG_MAG_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Log magnetometers status definitions                               -//
//----------------------------------------------------------------------//

/*!
 * Log magnetometer data status mask definitions
 */
#define	SBG_ECOM_MAG_MAG_X_BIT			(0x00000001u << 0)		/*!< Set to 1 if the magnetometer X passes Built In Test. */
#define	SBG_ECOM_MAG_MAG_Y_BIT			(0x00000001u << 1)		/*!< Set to 1 if the magnetometer Y passes Built In Test. */
#define	SBG_ECOM_MAG_MAG_Z_BIT			(0x00000001u << 2)		/*!< Set to 1 if the magnetometer Z passes Built In Test. */

#define	SBG_ECOM_MAG_ACCEL_X_BIT		(0x00000001u << 3)		/*!< Set to 1 if the accelerometer X passes Built In Test. */
#define	SBG_ECOM_MAG_ACCEL_Y_BIT		(0x00000001u << 4)		/*!< Set to 1 if the accelerometer Y passes Built In Test. */
#define	SBG_ECOM_MAG_ACCEL_Z_BIT		(0x00000001u << 5)		/*!< Set to 1 if the accelerometer Z passes Built In Test. */

#define	SBG_ECOM_MAG_MAGS_IN_RANGE		(0x00000001u << 6)		/*!< Set to 1 if all magnetometers are within operating range. */
#define SBG_ECOM_MAG_ACCELS_IN_RANGE	(0x00000001u << 7)		/*!< Set to 1 if all accelerometers are within operating range. */

#define SBG_ECOM_MAG_CALIBRATION_OK		(0x00000001u << 8)		/*!< Set to 1 if the magnetometers seems to be calibrated. */

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_MAG message.
 */
typedef struct _SbgLogMag
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< Magnetometer status bitmask. */
	float	magnetometers[3];			/*!< X, Y, Z magnetometer data in A.U. */
	float	accelerometers[3];			/*!< X, Y, Z accelerometers in m.s^-2. */
} SbgLogMag;

/*!
 * Structure that stores data for the SBG_ECOM_LOG_MAG_CALIB message.
 */
typedef struct _SbgLogMagCalib
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	reserved;					/*!< Reserved for future use. */
	uint8_t	magData[16];				/*!< Magnetometers calibration data. */
} SbgLogMagCalib;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_MAG message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagData(SbgStreamBuffer *pInputStream, SbgLogMag *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_MAG message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteMagData(SbgStreamBuffer *pOutputStream, const SbgLogMag *pInputData);

/*!
 * Parse data for the SBG_ECOM_LOG_MAG_CALIB message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseMagCalibData(SbgStreamBuffer *pInputStream, SbgLogMagCalib *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_MAG_CALIB message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteMagCalibData(SbgStreamBuffer *pOutputStream, const SbgLogMagCalib *pInputData);

#endif
