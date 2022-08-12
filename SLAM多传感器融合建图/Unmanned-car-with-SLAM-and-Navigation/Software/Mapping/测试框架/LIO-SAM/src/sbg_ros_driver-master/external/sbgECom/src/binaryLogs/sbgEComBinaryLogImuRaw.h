/*!
 *	\file		sbgEComBinaryLogImuRaw.h
 *  \author		SBG Systems (Alexis Guinamard)
 *	\date		18 Jully 2017
 *
 *	\brief		This file is used to parse received RAW IMU logs.
 *
 *	Raw IMU logs are used for calibration and tests purposes.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2017, SBG Systems SAS. All rights reserved.
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
#ifndef __SBG_ECOM_BINARY_LOG_IMU_RAW_H__
#define __SBG_ECOM_BINARY_LOG_IMU_RAW_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Log Inertial Data definitions                                      -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_IMU_RAW_DATA message.
 */
typedef struct _SbgLogImuRawData
{
	uint32_t	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16_t	status;						/*!< IMU status bitmask. */
	
	int32_t	rawAccelerometers[3];		/*!< X,Y,Z raw accelerometers signed data. No scale factor defined. */
	int32_t	rawGyroscopes[3];			/*!< X,Y,Z raw gyroscopes signed data. No scale factor defined. */
	int32_t	rawMagnetometers[3];		/*!< X,Y,Z raw magnetometers signed data. No scale factor defined. */

	int32_t	rawTempAccels[3];			/*!< X,Y,Z raw accelerometers temperature signed data. No scale factor defined. */
	int32_t	rawTempGyros[3];			/*!< X,Y,Z raw gyroscopes temperature signed data. No scale factor defined. */
	int32_t	rawTempMags[3];				/*!< X,Y,Z raw magnetometers temperature signed data. No scale factor defined. */
	
	int32_t	rawAuxValues[3];			/*!< Spare raw sensor values for specifics needs. */
	int32_t	rawTempAuxValues[3];		/*!< Spare raw sensor temperature values for specifics needs. */
} SbgLogImuRawData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_IMU_RAW_DATA message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseImuRawData(SbgStreamBuffer *pInputStream, SbgLogImuRawData *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_IMU_RAW_DATA message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteImuRawData(SbgStreamBuffer *pOutputStream, const SbgLogImuRawData *pInputData);

#endif
