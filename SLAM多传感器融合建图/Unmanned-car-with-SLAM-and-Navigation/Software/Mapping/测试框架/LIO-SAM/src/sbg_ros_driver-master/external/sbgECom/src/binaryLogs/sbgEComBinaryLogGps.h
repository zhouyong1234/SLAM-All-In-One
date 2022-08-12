/*!
 *	\file		sbgEComBinaryLogGps.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		20 February 2013
 *
 *	\brief		This file is used to parse received GPS binary logs.
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
#ifndef __SBG_ECOM_BINARY_LOG_GPS_H__
#define __SBG_ECOM_BINARY_LOG_GPS_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Log GPS velocity const definitions                                 -//
//----------------------------------------------------------------------//

/*!
 * Log GPS velocity status and type definitions.
 */
#define SBG_ECOM_GPS_VEL_STATUS_SHIFT		(0u)					/*!< Shift used to extract the GPS velocity status part. */
#define SBG_ECOM_GPS_VEL_STATUS_MASK		(0x0000003Fu)			/*!< Mask used to keep only the GPS velocity status part. */
#define SBG_ECOM_GPS_VEL_TYPE_SHIFT			(6u)					/*!< Shift used to extract the GPS velocity type part. */
#define SBG_ECOM_GPS_VEL_TYPE_MASK			(0x0000003Fu)			/*!< Mask used to keep only the GPS velocity type part. */

//----------------------------------------------------------------------//
//- Log GPS position const definitions                                 -//
//----------------------------------------------------------------------//

/*!
 * GPS position status and type definitions.
 */
#define SBG_ECOM_GPS_POS_STATUS_SHIFT		(0u)					/*!< Shift used to extract the GPS position status part. */
#define SBG_ECOM_GPS_POS_STATUS_MASK		(0x0000003Fu)			/*!< Mask used to keep only the GPS position status part. */
#define SBG_ECOM_GPS_POS_TYPE_SHIFT			(6u)					/*!< Shift used to extract the GPS position type part. */
#define SBG_ECOM_GPS_POS_TYPE_MASK			(0x0000003Fu)			/*!< Mask used to keep only the GPS position type part. */

/*!
 * GPS position mask definitions
 */
#define	SBG_ECOM_GPS_POS_GPS_L1_USED		(0x00000001u << 12)		/*!< Set to 1 if GPS L1 is used in solution. */
#define	SBG_ECOM_GPS_POS_GPS_L2_USED		(0x00000001u << 13)		/*!< Set to 1 if GPS L2 is used in solution. */
#define	SBG_ECOM_GPS_POS_GPS_L5_USED		(0x00000001u << 14)		/*!< Set to 1 if GPS L5 is used in solution. */
#define	SBG_ECOM_GPS_POS_GLO_L1_USED		(0x00000001u << 15)		/*!< Set to 1 if GLONASS L1 is used in solution. */
#define	SBG_ECOM_GPS_POS_GLO_L2_USED		(0x00000001u << 16)		/*!< Set to 1 if GLONASS L2 is used in solution. */

//----------------------------------------------------------------------//
//- Log GPS HDT const definitions                                      -//
//----------------------------------------------------------------------//

/*!
 * GPS HDT status definitions.
 */
#define SBG_ECOM_GPS_HDT_STATUS_SHIFT		(0u)					/*!< Shift used to extract the GPS HDT status part. */
#define SBG_ECOM_GPS_HDT_STATUS_MASK		(0x0000003Fu)			/*!< Mask used to keep only the GPS HDT status part. */

/*!
 * GPS HDT status bitmasks
 */
#define SBG_ECOM_GPS_HDT_BASELINE_VALID		(0x0001 << 6)			/*!< Set to 1 if the baseline length field is filled and valid. */

//----------------------------------------------------------------------//
//- Log GPS RAW const definitions                                      -//
//----------------------------------------------------------------------//

#define SBG_ECOM_GPS_RAW_MAX_BUFFER_SIZE	(4086u)					/*!< Maximum buffer size in bytes that can be stored in the GPS raw log. */

//----------------------------------------------------------------------//
//- Log GPS velocity enums definitions                                 -//
//----------------------------------------------------------------------//

/*!
 * GPS velocity status definitions.
 */
typedef enum _SbgEComGpsVelStatus
{
	SBG_ECOM_VEL_SOL_COMPUTED		= 0,							/*!< A valid solution has been computed. */
	SBG_ECOM_VEL_INSUFFICIENT_OBS	= 1,							/*!< Not enough valid SV to compute a solution. */
	SBG_ECOM_VEL_INTERNAL_ERROR		= 2,							/*!< An internal error has occurred. */
	SBG_ECOM_VEL_LIMIT				= 3								/*!< Velocity limit exceeded. */
} SbgEComGpsVelStatus;

/*!
 * GPS velocity types definitions.
 */
typedef enum _SbgEComGpsVelType
{
	SBG_ECOM_VEL_NO_SOLUTION		= 0,							/*!< No valid velocity solution available. */
	SBG_ECOM_VEL_UNKNOWN_TYPE		= 1,							/*!< An unknown solution type has been computed. */
	SBG_ECOM_VEL_DOPPLER			= 2,							/*!< A Doppler velocity has been computed. */
	SBG_ECOM_VEL_DIFFERENTIAL		= 3								/*!< A differential velocity has been computed between two positions. */
} SbgEComGpsVelType;

//----------------------------------------------------------------------//
//- Log GPS position enums definitions                                 -//
//----------------------------------------------------------------------//

/*!
 * GPS position status definitions.
 */
typedef enum _SbgEComGpsPosStatus
{
	SBG_ECOM_POS_SOL_COMPUTED		= 0,							/*!< A valid solution has been computed. */
	SBG_ECOM_POS_INSUFFICIENT_OBS	= 1,							/*!< Not enough valid SV to compute a solution. */
	SBG_ECOM_POS_INTERNAL_ERROR		= 2,							/*!< An internal error has occurred. */
	SBG_ECOM_POS_HEIGHT_LIMIT		= 3								/*!< The height limit has been exceeded. */
} SbgEComGpsPosStatus;

/*!
 * GPS position types definitions.
 */
typedef enum _SbgEComGpsPosType
{
	SBG_ECOM_POS_NO_SOLUTION		= 0,							/*!< No valid solution available. */
	SBG_ECOM_POS_UNKNOWN_TYPE		= 1,							/*!< An unknown solution type has been computed. */
	SBG_ECOM_POS_SINGLE				= 2,							/*!< Single point solution position. */
	SBG_ECOM_POS_PSRDIFF			= 3,							/*!< Standard Pseudorange Differential Solution (DGPS). */
	SBG_ECOM_POS_SBAS				= 4,							/*!< SBAS satellite used for differential corrections. */
	SBG_ECOM_POS_OMNISTAR			= 5,							/*!< Omnistar VBS Position (L1 sub-meter). */
	SBG_ECOM_POS_RTK_FLOAT			= 6,							/*!< Floating RTK ambiguity solution (20 cms RTK). */
	SBG_ECOM_POS_RTK_INT			= 7,							/*!< Integer RTK ambiguity solution (2 cms RTK). */
	SBG_ECOM_POS_PPP_FLOAT			= 8,							/*!< Precise Point Positioning with float ambiguities. */
	SBG_ECOM_POS_PPP_INT			= 9,							/*!< Precise Point Positioning with fixed ambiguities. */
	SBG_ECOM_POS_FIXED				= 10							/*!< Fixed location solution position. */
} SbgEComGpsPosType;

//----------------------------------------------------------------------//
//- Log GPS HDT enums definitions                                      -//
//----------------------------------------------------------------------//

/*!
 * GPS HDT status definitions.
 */
typedef enum _SbgEComGpsHdtStatus
{
	SBG_ECOM_HDT_SOL_COMPUTED		= 0,							/*!< A valid solution has been computed. */
	SBG_ECOM_HDT_INSUFFICIENT_OBS	= 1,							/*!< Not enough valid SV to compute a solution. */
	SBG_ECOM_HDT_INTERNAL_ERROR		= 2,							/*!< An internal error has occurred. */
	SBG_ECOM_HDT_HEIGHT_LIMIT		= 3								/*!< The height limit has been exceeded. */
} SbgEComGpsHdtStatus;

//----------------------------------------------------------------------//
//- Helpers methods for velocity status access                         -//
//----------------------------------------------------------------------//

/*!
 * Method used to read GPS velocity status from a status field.
 * \param[in]	status				Status uint32_t value to extract the velocity status from it.
 * \return							The extracted velocity status.
 */
SBG_INLINE SbgEComGpsVelStatus sbgEComLogGpsVelGetStatus(uint32_t status)
{
	return (SbgEComGpsVelStatus)((status >> SBG_ECOM_GPS_VEL_STATUS_SHIFT) & SBG_ECOM_GPS_VEL_STATUS_MASK);
}

/*!
 * Method used to read GPS velocity type from a status field.
 * \param[in]	status				Status uint32_t value to extract the velocity type from it.
 * \return							The extracted velocity type.
 */
SBG_INLINE SbgEComGpsVelType sbgEComLogGpsVelGetType(uint32_t status)
{
	return (SbgEComGpsVelType)((status >> SBG_ECOM_GPS_VEL_TYPE_SHIFT) & SBG_ECOM_GPS_VEL_TYPE_MASK);
}

/*!
 * Method used to write the GPS velocity status to a status field.
 * \param[in]	status				The velocity status to set.
 * \param[in]	type				The velocity type to set.
 * \return							The build GpsVelData status field.
 */
SBG_INLINE uint32_t sbgEComLogGpsVelBuildStatus(SbgEComGpsVelStatus status, SbgEComGpsVelType type)
{
	//
	// Create the combined status field
	//
	return	((((uint32_t)status)&SBG_ECOM_GPS_VEL_STATUS_MASK) << SBG_ECOM_GPS_VEL_STATUS_SHIFT) |
			((((uint32_t)type)&SBG_ECOM_GPS_VEL_TYPE_MASK) << SBG_ECOM_GPS_VEL_TYPE_SHIFT);
}

//----------------------------------------------------------------------//
//- Helpers methods for position status access                         -//
//----------------------------------------------------------------------//

/*!
 * Method used to read GPS position status from a status field.
 * \param[in]	status				Status uint32_t value to extract the position status from it.
 * \return							The extracted position status.
 */
SBG_INLINE SbgEComGpsPosStatus sbgEComLogGpsPosGetStatus(uint32_t status)
{
	return (SbgEComGpsPosStatus)((status >> SBG_ECOM_GPS_POS_STATUS_SHIFT) & SBG_ECOM_GPS_POS_STATUS_MASK);
}

/*!
 * Method used to read GPS position type from a status field.
 * \param[in]	status				Status uint32_t value to extract the position type from it.
 * \return							The extracted position type.
 */
SBG_INLINE SbgEComGpsPosType sbgEComLogGpsPosGetType(uint32_t status)
{
	return (SbgEComGpsPosType)((status >> SBG_ECOM_GPS_POS_TYPE_SHIFT) & SBG_ECOM_GPS_POS_TYPE_MASK);
}

/*!
 * Method used to write the GPS position status to a status field.
 * \param[in]	status				The position status to set.
 * \param[in]	type				The position type to set.
 * \param[in]	masks				Bit mask to set.
 * \return							The build GpsPosData status field.
 */
SBG_INLINE uint32_t sbgEComLogGpsPosBuildStatus(SbgEComGpsPosStatus status, SbgEComGpsPosType type, uint32_t masks)
{
	//
	// Create the combined status field
	//
	return	((((uint32_t)status)&SBG_ECOM_GPS_POS_STATUS_MASK) << SBG_ECOM_GPS_POS_STATUS_SHIFT) |
			((((uint32_t)type)&SBG_ECOM_GPS_POS_TYPE_MASK) << SBG_ECOM_GPS_POS_TYPE_SHIFT) | masks;
}

//----------------------------------------------------------------------//
//- Helpers methods for HDT status access                              -//
//----------------------------------------------------------------------//

/*!
 * Method used to read GPS HDT status from a status field.
 * \param[in]	status				Status uint32_t value to extract the HDT status from it.
 * \return							The extracted position status.
 */
SBG_INLINE SbgEComGpsHdtStatus sbgEComLogGpsHdtGetStatus(uint32_t status)
{
	return (SbgEComGpsHdtStatus)((status >> SBG_ECOM_GPS_HDT_STATUS_SHIFT) & SBG_ECOM_GPS_HDT_STATUS_MASK);
}

/*!
 * Method used to write the GPS HDT status to a status field.
 * \param[in]	status				The HDT status to set.
 * \param[in]	masks				Bit mask to set.
 * \return							The build GpsPosData status field.
 */
SBG_INLINE uint32_t sbgEComLogGpsHdtBuildStatus(SbgEComGpsHdtStatus status, uint32_t masks)
{
	//
	// Create the combined status field
	//
	return	((((uint32_t)status)&SBG_ECOM_GPS_HDT_STATUS_MASK) << SBG_ECOM_GPS_HDT_STATUS_SHIFT) | masks;
}

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_VEL message.
 */
typedef struct _SbgLogGpsVel
{
	uint32_t			timeStamp;				/*!< Time in us since the sensor power up. */
	uint32_t			status;					/*!< GPS velocity status, type and bitmask. */
	uint32_t			timeOfWeek;				/*!< GPS time of week in ms. */
	float			velocity[3];			/*!< GPS North, East, Down velocity in m.s^-1. */
	float			velocityAcc[3];			/*!< GPS North, East, Down velocity 1 sigma accuracy in m.s^-1. */
	float			course;					/*!< Track ground course in degrees. */
	float			courseAcc;				/*!< Course accuracy in degrees. */
} SbgLogGpsVel;

/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_POS message.
 */
typedef struct _SbgLogGpsPos
{
	uint32_t			timeStamp;				/*!< Time in us since the sensor power up. */
	uint32_t			status;					/*!< GPS position status, type and bitmask. */
	uint32_t			timeOfWeek;				/*!< GPS time of week in ms. */
	double			latitude;				/*!< Latitude in degrees, positive north. */
	double			longitude;				/*!< Longitude in degrees, positive east. */
	double			altitude;				/*!< Altitude above Mean Sea Level in meters. */
	float			undulation;				/*!< Altitude difference between the geoid and the Ellipsoid in meters (Height above Ellipsoid = altitude + undulation). */
	float			latitudeAccuracy;		/*!< 1 sigma latitude accuracy in meters. */
	float			longitudeAccuracy;		/*!< 1 sigma longitude accuracy in meters. */
	float			altitudeAccuracy;		/*!< 1 sigma altitude accuracy in meters. */
	uint8_t			numSvUsed;				/*!< Number of space vehicles used to compute the solution (since version 1.4). */
	uint16_t			baseStationId;			/*!< Base station id for differential corrections (0-4095). Set to 0xFFFF if differential corrections are not used (since version 1.4). */
	uint16_t			differentialAge;		/*!< Differential correction age in 0.01 seconds. Set to 0XFFFF if differential corrections are not used (since version 1.4). */
} SbgLogGpsPos;

/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_HDT message.
 */
typedef struct _SbgLogGpsHdt
{
	uint32_t			timeStamp;				/*!< Time in us since the sensor power up. */
	uint16_t			status;					/*!< GPS HDT status, type and bitmask. */
	uint32_t			timeOfWeek;				/*!< GPS time of week in ms. */
	float			heading;				/*!< GPS true heading in degrees. */
	float			headingAccuracy;		/*!< 1 sigma GPS true heading accuracy in degrees. */
	float			pitch;					/*!< GPS pitch angle measured from the master to the rover in degrees. */
	float			pitchAccuracy;			/*!< 1 signa GPS pitch angle accuarcy in degrees. */
	float			baseline;				/*!< The distance between the main and aux antenna in meters. */
} SbgLogGpsHdt;

/*!
 * Structure that stores data for the SBG_ECOM_LOG_GPS#_RAW message.
 */
typedef struct _SbgLogGpsRaw
{
	uint8_t			rawBuffer[SBG_ECOM_GPS_RAW_MAX_BUFFER_SIZE];	/*!< Buffer that contains raw GPS data. */
	size_t			bufferSize;										/*!< Raw buffer size in bytes. */
} SbgLogGpsRaw;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_GPS#_VEL message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsVelData(SbgStreamBuffer *pInputStream, SbgLogGpsVel *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_GPS#_VEL message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteGpsVelData(SbgStreamBuffer *pOutputStream, const SbgLogGpsVel *pInputData);

/*!
 * Parse data for the SBG_ECOM_LOG_GPS#_POS message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsPosData(SbgStreamBuffer *pInputStream, SbgLogGpsPos *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_GPS#_POS message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteGpsPosData(SbgStreamBuffer *pOutputStream, const SbgLogGpsPos *pInputData);

/*!
 * Parse data for the SBG_ECOM_LOG_GPS#_HDT message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsHdtData(SbgStreamBuffer *pInputStream, SbgLogGpsHdt *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_GPS#_HDT message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteGpsHdtData(SbgStreamBuffer *pOutputStream, const SbgLogGpsHdt *pInputData);

/*!
 * Parse data for the SBG_ECOM_LOG_GPS#_RAW message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseGpsRawData(SbgStreamBuffer *pInputStream, SbgLogGpsRaw *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_GPS#_RAW message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteGpsRawData(SbgStreamBuffer *pOutputStream, const SbgLogGpsRaw *pInputData);

#endif
