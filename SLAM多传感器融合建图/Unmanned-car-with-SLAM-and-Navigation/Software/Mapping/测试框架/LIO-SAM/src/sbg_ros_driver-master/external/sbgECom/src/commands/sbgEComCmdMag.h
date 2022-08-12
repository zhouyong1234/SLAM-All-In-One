/*!
 *	\file		sbgEComCmdMag.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to Magnetometer module.
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
#ifndef __SBG_ECOM_CMD_MAG_H__
#define __SBG_ECOM_CMD_MAG_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Magnetometer definitions										   -//
//----------------------------------------------------------------------//

/*!
 *	Define if the onboard magnetic calibration should acquiere points for a 3D or 2D calibration.
 */
typedef enum _SbgEComMagCalibMode
{
	SBG_ECOM_MAG_CALIB_MODE_2D			= 1,				/*!< Tell the device that the magnetic calibration will be performed with limited motions.
																 This calibration mode is only designed to be used when roll and pitch motions are less than ± 5°.
																 To work correctly, the device should be rotated through at least a full circle. */
	SBG_ECOM_MAG_CALIB_MODE_3D			= 2					/*!< Tell the device to start a full 3D magnetic calibration procedure.
																 The 3D magnetic calibration offers the best accuracy but needs at least motion of ± 30° on the roll and pitch angles. */
} SbgEComMagCalibMode;


/*!
 *	Used to select the expected dynamics during the magnetic calibration.
 */
typedef enum _SbgEComMagCalibBandwidth
{
	SBG_ECOM_MAG_CALIB_LOW_BW		= 0,					/*!< Tell the device that low dynamics will be observed during the magnetic calibration process. */
	SBG_ECOM_MAG_CALIB_MEDIUM_BW	= 1,					/*!< Tell the device that normal dynamics will be observed during the magnetic calibration process. */
	SBG_ECOM_MAG_CALIB_HIGH_BW		= 2						/*!< Tell the device that high dynamics will be observed during the magnetic calibration process. */
} SbgEComMagCalibBandwidth;

/*!
 *	General quality indicator of an onboard magnetic calibration.
 */
typedef enum _SbgEComMagCalibQuality
{
	SBG_ECOM_MAG_CALIB_QUAL_OPTIMAL	= 0,					/*!< All acquired points fit very well on a unit sphere after the calibration. */
	SBG_ECOM_MAG_CALIB_QUAL_GOOD	= 1,					/*!< Small deviations of the magnetic field norm have been detected. The magnetic calibration should although provide accurate heading. */
	SBG_ECOM_MAG_CALIB_QUAL_POOR	= 2,					/*!< Large deviations of the magnetic field norm have been detected. It may come from external magnetic distortions during the calibration. */
	SBG_ECOM_MAG_CALIB_QUAL_INVALID	= 3						/*!< No valid magnetic calibration has been computed. It could comes from too much magnetic disturbances, insufficient or invalid motions. */
} SbgEComMagCalibQuality;

/*!
 *	Confidence indicator on results of an onbard magnetic calibration.
 */
typedef enum _SbgEComMagCalibConfidence
{
	SBG_ECOM_MAG_CALIB_TRUST_HIGH	= 0,					/*!< Reported quality indicator can be trusted as enough remarkable magnetic field points have been acquired. */
	SBG_ECOM_MAG_CALIB_TRUST_MEDIUM	= 1,					/*!< Few remarkable magnetic field points have been used to compute the magnetic calibration leading to a medium confidence in reported quality indicators. */
	SBG_ECOM_MAG_CALIB_TRUST_LOW	= 2						/*!< Even if the quality indicator could report an excellent calibration,
																 The data set used to compute the magnetic calibration was not meaningful enough to compute meaningful quality indicators.
																 This calibration should be used carefully. */
} SbgEComMagCalibConfidence;

/*!
 *	Status bit masks used to report advanced inforamtion on the onboard magnetic calibration.
 */
#define SBG_ECOM_MAG_CALIB_NOT_ENOUGH_POINTS	(0x0001u)	/*!< Not enough valid magnetic points have been acquired. */
#define SBG_ECOM_MAG_CALIB_TOO_MUCH_DISTORTIONS	(0x0002u)	/*!< Unable to compute a magnetic calibration due to magnetic interferences or incorrect data set distribution. */
#define SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE		(0x0004u)	/*!< For a 3D calibration: not enough motion on X axis. For a 2D calibration; too much motion on X axis. */
#define SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE		(0x0008u)	/*!< For a 3D calibration: not enough motion on Y axis. For a 2D calibration; too much motion on Y axis. */
#define SBG_ECOM_MAG_CALIB_Z_MOTION_ISSUE		(0x0010u)	/*!< For a 3D or 2D calibration: not enough motion on Z axis. */
#define SBG_ECOM_MAG_CALIB_ALIGNMENT_ISSUE		(0x0020u)	/*!< For a 3D calibration: the alignment between the magnetometers and the inertial frame seems to be invalid. */ 

/*! 
 * This enum defines the different magnetometer model IDs available in standard
 */
typedef enum _SbgEComMagModelsStdIds
{
	SBG_ECOM_MAG_MODEL_NORMAL					= 201,		/*!< Should be used in most applications */
	SBG_ECOM_MAG_MODEL_NOISY_MAG_TOLERANT		= 202,		/*!< Should be used in disturbed magnetic environment */
} SbgEComMagModelsStdId;


//----------------------------------------------------------------------//
//- Magnetometer configuration										   -//
//----------------------------------------------------------------------//

/*!
 * Holds all necessary information for Magnetometer module data rejection.
 */
typedef struct _SbgEComMagRejectionConf
{
	SbgEComRejectionMode	magneticField;					/*!< Rejection mode for magnetic field. */
} SbgEComMagRejectionConf;

/*!
 * Helper structure to retrieve onboard magnetic calibration results.
 */
typedef struct _SbgEComMagCalibResults
{
	SbgEComMagCalibQuality		quality;					/*!< General magnetic calibration quality indicator. */
	SbgEComMagCalibConfidence	confidence;					/*!< Confidence indicator that should be read to interpret the quality indicator. */
	uint16_t						advancedStatus;				/*!< Set of bit masks used to report advanced information on the magnetic calibration status.*/

	float						beforeMeanError;			/*!< Mean magnetic field norm error observed before calibration. */
	float						beforeStdError;				/*!< Standard deviation of the magnetic field norm error observed before calibration. */
	float						beforeMaxError;				/*!< Maximum magnetic field norm error observed before calibration. */

	float						afterMeanError;				/*!< Mean magnetic field norm error observed after calibration. */
	float						afterStdError;				/*!< Standard deviation of the magnetic field norm error observed after calibration. */
	float						afterMaxError;				/*!< Maximum magnetic field norm error observed after calibration. */

	float						meanAccuracy;				/*!< Mean expected heading accuracy in radians. */
	float						stdAccuracy;				/*!< Standard deviation of the expected heading accuracy in radians. */
	float						maxAccuracy;				/*!< Maximum expected heading accuracy in radians. */

	uint16_t						numPoints;					/*!< Number of magnetic field points stored internally and used to compute the magnetic calibration. */
	uint16_t						maxNumPoints;				/*!< Maximum number of magnetic field points that can be stored internally. */
	float						offset[3];					/*!< Computed Hard Iron correction vector offset. */
	float						matrix[9];					/*!< Computed Hard & Soft Iron correction matrix. */
} SbgEComMagCalibResults;

//----------------------------------------------------------------------//
//- Magnetometer commands		                                       -//
//----------------------------------------------------------------------//

/*!
 *	Set magnetometer error model ID.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	id							Magnetometer model ID to set
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetModelId(SbgEComHandle *pHandle, uint32_t id);

/*!
 *	Retrieve magnetometer error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current magnetometer error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagGetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo);

/*!
 *	Retrieve the rejection configuration of the magnetometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pRejectConf					Pointer to a SbgEComMagRejectionConf struct to hold rejection configuration of the magnetometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagGetRejection(SbgEComHandle *pHandle, SbgEComMagRejectionConf *pRejectConf);

/*!
 *	Set the rejection configuration of the magnetometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pRejectConf					Pointer to a SbgEComMagRejectionConf struct holding rejection configuration for the magnetometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetRejection(SbgEComHandle *pHandle, const SbgEComMagRejectionConf *pRejectConf);

/*!
 *	Send a command that set the magnetometers calibration parameters.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	offset						Magnetometers calibration offset vector.
 *	\param[in]	matix						Magnetometers calibration 3x3 matrix.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagSetCalibData(SbgEComHandle *pHandle, const float offset[3], const float matrix[9]);

//----------------------------------------------------------------------//
//- Magnetometer onboard calibration commands	                       -//
//----------------------------------------------------------------------//

/*!
 *	Start the magnetic calibration process.
 *	As soon as this command is sent, the device will start logging magnetic field data internally.
 *	This set of data will be used later by the magnetic calibration algorithms to map the surrounding magnetic field.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	mode						Define which magnetic calibration type to perform. It could be 3D or 2D.
 *	\param[in]	bandwidth					Tell the device that we should have low, medium or high dynamics during the magnetic calibration process.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagStartCalib(SbgEComHandle *pHandle, SbgEComMagCalibMode mode, SbgEComMagCalibBandwidth bandwidth);

/*!
 *	This command computes a magnetic calibration solution based on the magnetic field logged since the last call to the command SBG_ECOM_CMD_START_MAG_CALIB (15).
 *	As soon as the computations are done, the device will answer with quality indicators, status flags and if possible a valid magnetic calibration matrix and offset.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pCalibResults				Pointer on a SbgEComMagCalibResults structure that can hold onboard magnetic calibration results and status.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdMagComputeCalib(SbgEComHandle *pHandle, SbgEComMagCalibResults *pCalibResults);

#endif
