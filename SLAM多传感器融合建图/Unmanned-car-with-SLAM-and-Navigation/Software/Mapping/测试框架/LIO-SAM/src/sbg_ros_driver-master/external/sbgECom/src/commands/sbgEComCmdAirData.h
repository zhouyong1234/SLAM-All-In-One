/*!
 *	\file		sbgEComCmdAirData.h
 *  \author		SBG Systems
 *	\date		18 February 2019
 *
 *	\brief		This file implements sbgECom commands related to AirData module.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2019, SBG Systems SAS. All rights reserved.
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
#ifndef SBG_ECOM_CMD_AIR_DATA_H
#define SBG_ECOM_CMD_AIR_DATA_H

/* sbgCommonLib headers */
#include <sbgCommon.h>

/* Local headers */
#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * This enum defines the different AirData model IDs available in standard
 */
typedef enum _SbgEComAirDataModelsIds
{
	SBG_ECOM_AIR_DATA_MODEL_INTERNAL		= 1,		/*!< Use the internal barometer sensor if available. */
	SBG_ECOM_AIR_DATA_MODEL_GENERIC_ECOM	= 2,		/*!< Generic AirData model using sbgECom input protocol format. */
	SBG_ECOM_AIR_DATA_MODEL_AHRS_500		= 3,		/*!< Crossbow AHRS-500 compatible input for barometric altitude and airspeed. */
} SbgEComAirDataModelsIds;

/*!
 * Holds all necessary information for AirData module data rejection.
 */
typedef struct _SbgEComAirDataRejectionConf
{
	SbgEComRejectionMode	airspeed;					/*!< Rejection mode for the true air speed measurement. */
	SbgEComRejectionMode	altitude;					/*!< Rejection mode for the barometric altitude measurement. */
} SbgEComAirDataRejectionConf;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Set the AirData model to use that both defines the protocol as well as the associated error model.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	modelId						AirData model ID to set
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAirDataSetModelId(SbgEComHandle *pHandle, SbgEComAirDataModelsIds modelId);

/*!
 * Retrieve the AirData model id currently in use by the device.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pModelId					Returns the AirData model ID currently in use by the device.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAirDataGetModelId(SbgEComHandle *pHandle, SbgEComAirDataModelsIds *pModelId);

/*!
 * Set the lever arm configuration of the AirData module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	pLeverArm					The X, Y, Z airspeed sensor lever arm in meters from the pitot sensor to the IMU.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAirDataSetLeverArm(SbgEComHandle *pHandle, const float *pLeverArm);

/*!
 * Retrieve the lever arm configuration of the AirData module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pLeverArm					Returns the airspeed sensor X,Y,Z lever arm in meters from the pitot sensor to the IMU.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAirDataGetLeverArm(SbgEComHandle *pHandle, float *pLeverArm);

/*!
 * Set the rejection configuration of the AirData module (this command doesn't need a reboot to be applied)
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pRejectConf					The new rejection configuration to set.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAirDataSetRejection(SbgEComHandle *pHandle, const SbgEComAirDataRejectionConf *pRejectConf);

/*!
 * Retrieve the current rejection configuration of the AirData module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pRejectConf					Return the rejection configuration currently in use.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAirDataGetRejection(SbgEComHandle *pHandle, SbgEComAirDataRejectionConf *pRejectConf);

#endif
