/*!
 *	\file		sbgEComCmdGnss.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to GNSS module.
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
#ifndef SBG_ECOM_CMD_GNSS_H
#define SBG_ECOM_CMD_GNSS_H

/* sbgCommonLib headers */
#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Public definitions (DEPRECATED)                                    -//
//----------------------------------------------------------------------//

/*!
 * DEPRECATED: Command SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT has been replaced by SBG_ECOM_CMD_GNSS_1_INSTALLATION
 * Holds all necessary information for GNSS module alignment.
 */
typedef struct _SbgEComGnssAlignmentInfo
{
	float	leverArmX;						/*!< GNSS antenna lever arm in IMU X axis in meters */
	float	leverArmY;						/*!< GNSS antenna lever arm in IMU Y axis in meters */
	float	leverArmZ;						/*!< GNSS antenna lever arm in IMU Z axis in meters */
	float	pitchOffset;					/*!< Pitch offset for dual antenna GNSS in rad */
	float	yawOffset;						/*!< Yaw offset for dual antenna GNSS in rad */
	float	antennaDistance;				/*!< Distance between two GNSS antennas in meters */
} SbgEComGnssAlignmentInfo;

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * This enum defines the different GNSS model IDs available in standard
 */
typedef enum _SbgEComGnssModelsStdIds
{
	SBG_ECOM_GNSS_MODEL_UBLOX_GPS_GLONASS			= 101,		/*!< Used on Ellipse-N to setup the internal GNSS in GPS+GLONASS */
	SBG_ECOM_GNSS_MODEL_NMEA						= 102,		/*!< Used on ELLIPSE-E to accept an external GNSS using NMEA protocol */
	SBG_ECOM_GNSS_MODEL_UBLOX_GPS_BEIDOU			= 103,		/*!< Used on ELLIPSE-N to setup the internal GNSS in GPS+BEIDOU */
	SBG_ECOM_GNSS_MODEL_UBLOX_EXTERNAL				= 104,		/*!< Used on ELLIPSE-E to accept an external Ublox GNSS (read only) */
	SBG_ECOM_GNSS_MODEL_UBLOX_HIGH_DYNAMICS			= 105,		/*!< Used on ELLIPSE-N to setup the internal GNSS to high dynamic motion constraints. */
	SBG_ECOM_GNSS_MODEL_NOVATEL_EXTERNAL			= 106,		/*!< Used on ELLIPSE-E to accept an external Novatel GNSS (read only) */
	SBG_ECOM_GNSS_MODEL_ELLIPSE_D_INTERNAL			= 107,		/*!< Used on ELLIPSE-D to setup the internal GNSS receiver. */
	SBG_ECOM_GNSS_MODEL_UBLOX_HIGH_SPEED			= 108,		/*!< Used on ELLIPSE-N to setup the internal GNSS for high speed applications (airborne > 200 km/h) */
	SBG_ECOM_GNSS_MODEL_SEPTENTRIO_EXTERNAL			= 109,		/*!< Used on ELLIPSE-E to accept an external Septentrio GNSS (read only) */
	SBG_ECOM_GNSS_MODEL_UBLOX_LOW_SPEED				= 110		/*!< Used on ELLIPSE-N to setup the internal GNSS receiver for pedestrian applications. */
} SbgEComGnssModelsStdIds;

/*!
 * GNSS mechanical installation modes for the dual antenna mode.
 */
typedef enum _SbgEComGnssInstallationMode
{
	SBG_ECOM_GNSS_INSTALLATION_MODE_SINGLE			= 1,		/*!< The GNSS will be used in single antenna mode only and the secondary lever arm is not used. */
	SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_AUTO		= 2,		/*!< [Reserved] The GNSS dual antenna information will be used but the secondary lever arm is not known. */
	SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_ROUGH		= 3,		/*!< The GNSS dual antenna information will be used and we have a rough guess for the secondary lever arm. */
	SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_PRECISE	= 4			/*!< The GNSS dual antenna information will be used and the secondary lever arm is accurately entered and doesn't need online re-estimation. */
} SbgEComGnssInstallationMode;

/*!
 * GNSS mechanical installation parameters to be used with command SBG_ECOM_CMD_GNSS_#_INSTALLATION
 */
typedef struct _SbgEComGnssInstallation
{
	float						leverArmPrimary[3];				/*!< GNSS primary antenna lever arm in IMU X, Y, Z axis in meters */
	bool						leverArmPrimaryPrecise;			/*!< If set to true, the primary lever arm has been accurately entered and doesn't need online re-estimation. */

	float						leverArmSecondary[3];			/*!< GNSS secondary antenna lever arm in IMU X, Y, Z axis in meters */	
	SbgEComGnssInstallationMode	leverArmSecondaryMode;			/*!< Define the secondary antenna (dual antenna) operating mode. */
} SbgEComGnssInstallation;

/*!
 * Holds all necessary information for GNSS module data rejection.
 */
typedef struct _SbgEComGnssRejectionConf
{
	SbgEComRejectionMode	position;							/*!< Rejection mode for position. */
	SbgEComRejectionMode	velocity;							/*!< Rejection mode for velocity. */
	SbgEComRejectionMode	hdt;								/*!< Rejection mode for true heading. */
} SbgEComGnssRejectionConf;

//----------------------------------------------------------------------//
//- GNSS public commands		                                       -//
//----------------------------------------------------------------------//

/*!
 * Set GNSS error model id.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	id							Model ID to set
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetModelId(SbgEComHandle *pHandle, uint32_t id);

/*!
 * Retrieve GNSS error model information.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current GNSS error model info.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo);

/*!
 * Retrieve the lever arm and alignment configuration of the gnss 1 module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct to hold alignment configuration of the gnss module.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SBG_DEPRECATED(SbgErrorCode sbgEComCmdGnss1GetLeverArmAlignment(SbgEComHandle *pHandle, SbgEComGnssAlignmentInfo *pAlignConf));

/*!
 * Set the lever arm and alignment configuration of the gnss 1 module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct holding alignment configuration for the gnss module.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SBG_DEPRECATED(SbgErrorCode sbgEComCmdGnss1SetLeverArmAlignment(SbgEComHandle *pHandle, const SbgEComGnssAlignmentInfo *pAlignConf));

/*!
 * Retrieve the mechanical installation parameters for the GNSS 1 module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pGnssInstallation			Used to store the retrieved the GNSS installation parameters.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1InstallationGet(SbgEComHandle *pHandle, SbgEComGnssInstallation *pGnssInstallation);

/*!
 * Set the mechanical installation parameters for the GNSS 1 module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	pGnssInstallation				The GNSS installation parameters to set.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1InstallationSet(SbgEComHandle *pHandle, const SbgEComGnssInstallation *pGnssInstallation);

/*!
 * Retrieve the rejection configuration of the gnss module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct to hold rejection configuration of the gnss module.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetRejection(SbgEComHandle *pHandle, SbgEComGnssRejectionConf *pRejectConf);

/*!
 * Set the rejection configuration of the gnss module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct holding rejection configuration for the gnss module.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetRejection(SbgEComHandle *pHandle, const SbgEComGnssRejectionConf *pRejectConf);

#endif /* SBG_ECOM_CMD_GNSS_H */
