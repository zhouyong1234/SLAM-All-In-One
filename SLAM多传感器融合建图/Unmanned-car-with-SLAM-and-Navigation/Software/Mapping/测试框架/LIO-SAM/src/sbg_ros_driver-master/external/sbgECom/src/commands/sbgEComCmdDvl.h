/*!
 *	\file		sbgEComCmdDvl.h
 *  \author		SBG Systems
 *	\date		13 December 2018
 *
 *	\brief		This file implements sbgECom commands related to DVL module.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2018, SBG Systems SAS. All rights reserved.
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
#ifndef SBG_ECOM_CMD_DVL_H
#define SBG_ECOM_CMD_DVL_H

/* sbgCommonLib headers */
#include <sbgCommon.h>

/* Local headers */
#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * This enum defines the different DVL model IDs available in standard
 */
typedef enum _SbgEComDvlModelsIds
{
	SBG_ECOM_DVL_MODEL_GENERIC_PD6	= 202,		/*!< Generic DVL using PD6 protocol format. */
} SbgEComDvlModelsIds;

/*!
 * DVL mechanical installation parameters such as lever arm and alignment
 */
typedef struct _SbgEComDvlInstallation
{
	float	leverArm[3];						/*!< X, Y, Z DVL lever arm in meters expressed from the DVL to the IMU. */
	float	alignment[3];						/*!< Roll, pitch, yaw DVL alignment expressed in radians. */
	bool	preciseInstallation;				/*!< Set to true if both the DVL lever arm and DVL alignment are precise and don't require in-run estimation. */
} SbgEComDvlInstallation;

/*!
 * Holds all necessary information for DVL module data rejection.
 */
typedef struct _SbgEComDvlRejectionConf
{
	SbgEComRejectionMode	bottomLayer;		/*!< Rejection mode for the bottom tracking (ie when the velocity  measurement is in respect to the seabed). */
	SbgEComRejectionMode	waterLayer;			/*!< Rejection mode for the water tracking (ie when the velocity measurement is relative to a water layer). */
} SbgEComDvlRejectionConf;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Set the DVL model to use that both defines the protocol as well as the associated error model.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	modelId						DVL model ID to set
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdDvlSetModelId(SbgEComHandle *pHandle, SbgEComDvlModelsIds modelId);

/*!
 * Retrieve the DVL model id currently in use by the device.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pModelId					Returns the DVL model ID currently in use by the device.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdDvlGetModelId(SbgEComHandle *pHandle, SbgEComDvlModelsIds *pModelId);

/*!
 * Set the lever arm and alignment configuration of the DVL module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	pDvlInstallation			The DVL lever arm and alignment configuration to apply.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdDvlInstallationSet(SbgEComHandle *pHandle, const SbgEComDvlInstallation *pDvlInstallation);

/*!
 * Retrieve the lever arm and alignment configuration of the DVL module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pDvlInstallation			Returns the DVL lever arm and alignment configuration currently in use.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdDvlInstallationGet(SbgEComHandle *pHandle, SbgEComDvlInstallation *pDvlInstallation);

/*!
 * Set the rejection configuration of the DVL module (this command doesn't need a reboot to be applied)
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pRejectConf					The new DVL rejection configuration to set.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdDvlSetRejection(SbgEComHandle *pHandle, const SbgEComDvlRejectionConf *pRejectConf);

/*!
 * Retrieve the current rejection configuration of the DVL module.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pRejectConf					Return the DVL rejection configuration currently in use.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdDvlGetRejection(SbgEComHandle *pHandle, SbgEComDvlRejectionConf *pRejectConf);

#endif
