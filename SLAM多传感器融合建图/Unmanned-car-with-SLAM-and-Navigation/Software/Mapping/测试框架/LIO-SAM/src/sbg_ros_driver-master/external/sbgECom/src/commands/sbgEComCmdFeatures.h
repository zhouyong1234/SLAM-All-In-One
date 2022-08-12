/*!
 *	\file		sbgEComCmdFeatures.h
 *  \author		SBG Systems (Alexis Guinamard)
 *	\date		19 March 2015
 *
 *	\brief		This file implements SbgECom commands related to device features.
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
#ifndef __SBG_ECOM_CMD_FEATURES_H__
#define __SBG_ECOM_CMD_FEATURES_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Feature commands definitions                                       -//
//----------------------------------------------------------------------//

//
// Main sensor features
//
#define SBG_ECOM_SENSOR_FEATURE_IMU				(0x00000001 << 0)			/*!< This unit can provide IMU data */
#define SBG_ECOM_SENSOR_FEATURE_AHRS			(0x00000001 << 1)			/*!< This unit can provide orientation data	*/
#define SBG_ECOM_SENSOR_FEATURE_NAVIGATION		(0x00000001 << 2)			/*!< This unit can provide position and velocity data */
#define SBG_ECOM_SENSOR_FEATURE_SHIP_MOTION		(0x00000001 << 3)			/*!< This unit can provide ship motion data output (heave) */

//
// GPS Signals bitmask defining every signal
//
#define SBG_ECOM_GNSS_SIGNAL_GPS_L1				(0x00000001 << 0)			/*!< This GNSS receiver tracks GPS L1 band. */
#define SBG_ECOM_GNSS_SIGNAL_GPS_L2				(0x00000001 << 1)			/*!< This GNSS receiver tracks GPS L2 band. */
#define SBG_ECOM_GNSS_SIGNAL_GPS_L5				(0x00000001 << 2)			/*!< This GNSS receiver tracks GPS L5 band. */
#define SBG_ECOM_GNSS_SIGNAL_GLONASS_L1			(0x00000001 << 3)			/*!< This GNSS receiver tracks GLONASS L1 band. */
#define SBG_ECOM_GNSS_SIGNAL_GLONASS_L2			(0x00000001 << 4)			/*!< This GNSS receiver tracks GLONASS L2 band. */
#define SBG_ECOM_GNSS_SIGNAL_BEIDOU_B1			(0x00000001 << 5)			/*!< This GNSS receiver tracks BEIDOU B1 band. */
#define SBG_ECOM_GNSS_SIGNAL_BEIDOU_B2			(0x00000001 << 6)			/*!< This GNSS receiver tracks BEIDOU B2 band. */
#define SBG_ECOM_GNSS_SIGNAL_BEIDOU_B3			(0x00000001 << 7)			/*!< This GNSS receiver tracks BEIDOU B3 band. */
#define SBG_ECOM_GNSS_SIGNAL_GALILEO_E1			(0x00000001 << 8)			/*!< This GNSS receiver tracks GALILEO E1 band. */
#define SBG_ECOM_GNSS_SIGNAL_GALILEO_E5			(0x00000001 << 9)			/*!< This GNSS receiver tracks GALILEO E5 band. */
#define SBG_ECOM_GNSS_SIGNAL_GALILEO_E6			(0x00000001 << 10)			/*!< This GNSS receiver tracks GALILEO E6 band. */
#define SBG_ECOM_GNSS_SIGNAL_QZSS				(0x00000001 << 11)			/*!< This GNSS receiver tracks QZSS signals */
#define SBG_ECOM_GNSS_SIGNAL_SBAS				(0x00000001 << 12)			/*!< This GNSS receiver tracks SBAS signals */
#define SBG_ECOM_GNSS_SIGNAL_L_BAND				(0x00000001 << 13)			/*!< This GNSS receiver tracks L-Band (for PPP services) */

//
// GPS capabilities
//
#define SBG_ECOM_GNSS_FEATURE_DUAL_ANT			(0x00000001 << 0)			/*!< This GNSS receiver provides a dual antenna heading */
#define SBG_ECOM_GNSS_FEATURE_RTK_LIMITED		(0x00000001 << 1)			/*!< This GNSS receiver has limited RTK accuracy (eg. Trimble RTK 30/30)  */
#define SBG_ECOM_GNSS_FEATURE_RTK				(0x00000001 << 2)			/*!< This GNSS receiver provides full RTK accuracy */
#define SBG_ECOM_GNSS_FEATURE_PPP				(0x00000001 << 3)			/*!< This GNSS receiver provides PPP computations */
#define SBG_ECOM_GNSS_FEATURE_RAW_DATA			(0x00000001 << 4)			/*!< This GNSS receiver provides RAW data output */
#define SBG_ECOM_GNSS_FEATURE_RAIM				(0x00000001 << 5)			/*!< This GNSS receiver provides Receiver Autonomous Integrity Monitoring */
#define SBG_ECOM_GNSS_FEATURE_HIGH_SPEED		(0x00000001 << 6)			/*!< This GNSS receiver has no high speed limitation (> 515m/s) */

//----------------------------------------------------------------------//
//- Feature commands types definition                                  -//
//----------------------------------------------------------------------//

/*!
 * This enum defines the different types of internal GNSS receiver that can provide specific features.
 * Note External type is considered as not handled by the feature system
 */
typedef enum _SbgEComGnssType
{
	SBG_ECOM_GNSS_TYPE_DISABLED			= 0,	/*!< GNSS module disabled */
	SBG_ECOM_GNSS_TYPE_EXTERNAL			= 1,	/*!< External GNSS module (all features are unknown) */
	SBG_ECOM_GNSS_TYPE_UBX_MAX_M8		= 2,	/*!< Ublox MAX-M8 module */
	SBG_ECOM_GNSS_TYPE_NOV_OEM615		= 3,	/*!< Novatel OEM615 device */
	SBG_ECOM_GNSS_TYPE_NOV_OEM615_DUAL	= 4,	/*!< Two Novatel OEM615 devices for dual antenna */
	SBG_ECOM_GNSS_TYPE_NOV_OEM617D		= 5,	/*!< Novatel OEM617D device */
	SBG_ECOM_GNSS_TYPE_SEP_AX4			= 6,	/*!< Septentrio Asterx m4 */
	SBG_ECOM_GNSS_TYPE_SEP_AXM2A		= 7		/*!< Septentrio Asterx m2a */
} SbgEComGnssType;

/*!
 * This structure contains all the information provided by the SBG_ECOM_CMD_GET_FEATURES command
 */
typedef struct _SbgEComFeatures
{
	uint32_t			sensorFeaturesMask;			/*!< The different measurement capabilities of this unit */
	SbgEComGnssType gnssType;					/*!< The type of GNSS receiver used (brand and model) */
	uint8_t			gnssUpdateRate;				/*!< The actual GNSS update rate */
	uint32_t			gnssSignalsMask;			/*!< GNSS receiver signals tracking */
	uint32_t			gnssFeaturesMask;			/*!< GNSS receiver computation and output features */
	char			gnssProductCode[32];		/*!< String containing the GNSS receiver product code (“\0” if unknown) */
	char			gnssSerialNumber[32];		/*!< String containing the GNSS receiver serial number (“\0” if unknown) */
} SbgEComFeatures;

//----------------------------------------------------------------------//
//- Info commands				                                       -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the device and embedded GPS receiver features.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pFeatures					A pointer to a structure to hold features.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGetFeatures(SbgEComHandle *pHandle, SbgEComFeatures *pFeatures);

#endif
