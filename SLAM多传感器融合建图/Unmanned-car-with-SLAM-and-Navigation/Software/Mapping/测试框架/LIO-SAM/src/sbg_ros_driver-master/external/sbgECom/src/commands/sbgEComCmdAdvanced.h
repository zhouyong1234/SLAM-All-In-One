/*!
 *	\file		sbgEComCmdAdvanced.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to advanced settings.
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

#ifndef __SBG_ECOM_CMD_ADVANCED_H__
#define __SBG_ECOM_CMD_ADVANCED_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Advanced definitions											   -//
//----------------------------------------------------------------------//

/*!
 * List of available time reference source.
 */
typedef enum _SbgEComTimeReferenceSrc
{
	SBG_ECOM_TIME_REF_DISABLED = 0,				/*!< The device is running it's internal clock without any time reference. */
	SBG_ECOM_TIME_REF_SYNC_IN_A,				/*!< The main port sync in A is used as a time reference. */
	SBG_ECOM_TIME_REF_UTC_GPS_1					/*!< The GPS 1 module is used to provide both time reference and UTC data. */
} SbgEComTimeReferenceSrc;

//----------------------------------------------------------------------//
//- Advanced configurations											   -//
//----------------------------------------------------------------------//

/*!
 * Structure containing all the info for advanced configuration.
 */
typedef struct _SbgEComAdvancedConf
{
	SbgEComTimeReferenceSrc	timeReference;		/*!< Time reference source for clock alignment. */
} SbgEComAdvancedConf;

/*!
 * Structure containing all validity thresholds (status outputs)
 * Setting these thresholds to 0.0 will keep default configuration
 */
typedef struct _SbgEComValidityThresholds
{
	float	positionThreshold;					/*!< Norm of the position standard deviation threshold to raise position valid flag (m)*/
	float	velocityThreshold;					/*!< Norm of the velocity standard deviation threshold to raise velocity valid flag  (m/s)*/
	float	attitudeThreshold;					/*!< Max of the roll/pitch standard deviations threshold to raise attitude valid flag (rad) */					
	float	headingThreshold;					/*!< Heading standard deviations threshold to raise heading valid flag (rad) */
} SbgEComValidityThresholds;

//----------------------------------------------------------------------//
//- Advanced commands			                                       -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the advanced configurations.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pConf						Pointer to a SbgEComAdvancedConf to contain the current configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAdvancedGetConf(SbgEComHandle *pHandle, SbgEComAdvancedConf *pConf);

/*!
 *	Set the advanced configurations.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pConf						Pointer to a SbgEComAdvancedConf that contains the new configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAdvancedSetConf(SbgEComHandle *pHandle, const SbgEComAdvancedConf *pConf);

/*!
 *	Retrieve the current validity thresholds
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pConf						Pointer to a SbgEComValidityThresholds to contain the current configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */

SbgErrorCode sbgEComCmdAdvancedGetThresholds(SbgEComHandle *pHandle, SbgEComValidityThresholds *pConf);
/*!
 *	Set the validity thresholds
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pConf						Pointer to a SbgEComValidityThresholds that contains the new configuration.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdAdvancedSetThresholds(SbgEComHandle *pHandle, const SbgEComValidityThresholds *pConf);

#endif
