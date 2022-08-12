/*!
 *	\file		sbgEComBinaryLogStatus.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		03 April 2013
 *
 *	\brief		This file is used to parse received device status binary logs.
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
#ifndef __SBG_ECOM_BINARY_LOG_STATUS_H__
#define __SBG_ECOM_BINARY_LOG_STATUS_H__

#include <sbgCommon.h>
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- General status definitions                                         -//
//----------------------------------------------------------------------//
#define SBG_ECOM_GENERAL_MAIN_POWER_OK		(0x0001u << 0)			/*!< Set to 1 when main power supply is OK. */
#define SBG_ECOM_GENERAL_IMU_POWER_OK		(0x0001u << 1)			/*!< Set to 1 when IMU power supply is OK. */
#define SBG_ECOM_GENERAL_GPS_POWER_OK		(0x0001u << 2)			/*!< Set to 1 when GPS power supply is OK. */
#define SBG_ECOM_GENERAL_SETTINGS_OK		(0x0001u << 3)			/*!< Set to 1 if settings where correctly loaded. */
#define SBG_ECOM_GENERAL_TEMPERATURE_OK		(0x0001u << 4)			/*!< Set to 1 when temperature is within specified limits. */
#define SBG_ECOM_GENERAL_DATALOGGER_OK		(0x0001u << 5)			/*!< Set to 1 when the datalogger is working correctly. */
#define SBG_ECOM_GENERAL_CPU_OK				(0x0001u << 6)			/*!< Set to 1 if the CPU headroom is correct.*/

//----------------------------------------------------------------------//
//- Communication status definitions                                   -//
//----------------------------------------------------------------------//

/*!
 * Communication CAN status definitions.
 */
#define SBG_ECOM_CAN_STATUS_SHIFT			(28u)					/*!< Shift used to access the CAN status part. */
#define SBG_ECOM_CAN_STATUS_MASK			(0x00000007u)			/*!< Mask used to keep only the CAN status part. */

/*!
 * Communication status bit mask definitions.
 */
#define SBG_ECOM_PORTA_VALID				(0x00000001u << 0)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTB_VALID				(0x00000001u << 1)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTC_VALID				(0x00000001u << 2)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTD_VALID				(0x00000001u << 3)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTE_VALID				(0x00000001u << 4)		/*!< Set to 0 in case of low level communication error. */

#define SBG_ECOM_PORTA_RX_OK				(0x00000001u << 5)		/*!< Set to 0 in case of error on PORT A input. */
#define SBG_ECOM_PORTA_TX_OK				(0x00000001u << 6)		/*!< Set to 0 in case of error on PORT A output. */
#define SBG_ECOM_PORTB_RX_OK				(0x00000001u << 7)		/*!< Set to 0 in case of error on PORT B input. */
#define SBG_ECOM_PORTB_TX_OK				(0x00000001u << 8)		/*!< Set to 0 in case of error on PORT B output. */
#define SBG_ECOM_PORTC_RX_OK				(0x00000001u << 9)		/*!< Set to 0 in case of error on PORT C input. */
#define SBG_ECOM_PORTC_TX_OK				(0x00000001u << 10)		/*!< Set to 0 in case of error on PORT C output. */
#define SBG_ECOM_PORTD_RX_OK				(0x00000001u << 11)		/*!< Set to 0 in case of error on PORT D input. */
#define SBG_ECOM_PORTD_TX_OK				(0x00000001u << 12)		/*!< Set to 0 in case of error on PORT D input. */
#define SBG_ECOM_PORTE_RX_OK				(0x00000001u << 13)		/*!< Set to 0 in case of error on PORT E input. */
#define SBG_ECOM_PORTE_TX_OK				(0x00000001u << 14)		/*!< Set to 0 in case of error on PORT D input. */

#define SBG_ECOM_ETH0_VALID					(0x00000001u << 15)		/*!< Set to 0 in case of error on ETH0. */
#define SBG_ECOM_ETH1_VALID					(0x00000001u << 16)		/*!< Set to 0 in case of error on ETH1. */
#define SBG_ECOM_ETH2_VALID					(0x00000001u << 17)		/*!< Set to 0 in case of error on ETH2. */
#define SBG_ECOM_ETH3_VALID					(0x00000001u << 18)		/*!< Set to 0 in case of error on ETH3. */
#define SBG_ECOM_ETH4_VALID					(0x00000001u << 19)		/*!< Set to 0 in case of error on ETH4. */

#define SBG_ECOM_CAN_VALID					(0x00000001u << 25)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_CAN_RX_OK					(0x00000001u << 26)		/*!< Set to 0 in case of error on CAN Bus input buffer. */
#define SBG_ECOM_CAN_TX_OK					(0x00000001u << 27)		/*!< Set to 0 in case of error on CAN Bus output buffer. */

/*!
 * Communication status for the CAN Bus.
 */
typedef enum _SbgEComCanBusStatus
{
	SBG_ECOM_CAN_BUS_OFF		= 0,								/*!< Bus OFF operation due to too much errors. */
	SBG_ECOM_CAN_BUS_TX_RX_ERR	= 1,								/*!< Errors on Tx or Rx. */
	SBG_ECOM_CAN_BUS_OK			= 2,								/*!< Bus OK. */
	SBG_ECOM_CAN_BUS_ERROR		= 3									/*!< Bus error. */
} SbgEComCanBusStatus;

//----------------------------------------------------------------------//
//- Aiding status definitions                                          -//
//----------------------------------------------------------------------//
#define SBG_ECOM_AIDING_GPS1_POS_RECV		(0x00000001u << 0)		/*!< Set to 1 when valid GPS 1 position data is received. */
#define SBG_ECOM_AIDING_GPS1_VEL_RECV		(0x00000001u << 1)		/*!< Set to 1 when valid GPS 1 velocity data is received. */
#define SBG_ECOM_AIDING_GPS1_HDT_RECV		(0x00000001u << 2)		/*!< Set to 1 when valid GPS 1 true heading data is received. */
#define SBG_ECOM_AIDING_GPS1_UTC_RECV		(0x00000001u << 3)		/*!< Set to 1 when valid GPS 1 UTC time data is received. */
#define SBG_ECOM_AIDING_GPS2_POS_RECV		(0x00000001u << 4)		/*!< Set to 1 when valid GPS 2 position data is received. */
#define SBG_ECOM_AIDING_GPS2_VEL_RECV		(0x00000001u << 5)		/*!< Set to 1 when valid GPS 2 velocity data is received. */
#define SBG_ECOM_AIDING_GPS2_HDT_RECV		(0x00000001u << 6)		/*!< Set to 1 when valid GPS 2 true heading data is received. */
#define SBG_ECOM_AIDING_GPS2_UTC_RECV		(0x00000001u << 7)		/*!< Set to 1 when valid GPS 2 UTC time data is received. */
#define SBG_ECOM_AIDING_MAG_RECV			(0x00000001u << 8)		/*!< Set to 1 when valid Magnetometer data is received. */
#define SBG_ECOM_AIDING_ODO_RECV			(0x00000001u << 9)		/*!< Set to 1 when Odometer pulse is received. */
#define SBG_ECOM_AIDING_DVL_RECV			(0x00000001u << 10)		/*!< Set to 1 when valid DVL data is received. */
#define SBG_ECOM_AIDING_USBL_RECV			(0x00000001u << 11)		/*!< Set to 1 when valid USBL data is received. */
#define SBG_ECOM_AIDING_DEPTH_RECV			(0x00000001u << 12)		/*!< Set to 1 when valid Depth Log data is received. */
#define SBG_ECOM_AIDING_AIR_DATA_RECV		(0x00000001u << 13)		/*!< Set to 1 when valid Air Data (altitude and/or true airspeed) is received. */
#define SBG_ECOM_AIDING_USER_POS_RECV		(0x00000001u << 14)		/*!< Set to 1 when valid user position data is received. */
#define SBG_ECOM_AIDING_USER_VEL_RECV		(0x00000001u << 15)		/*!< Set to 1 when valid user velocity data is received. */
#define SBG_ECOM_AIDING_USER_HEADING_RECV	(0x00000001u << 16)		/*!< Set to 1 when valid user heading data is received. */

//----------------------------------------------------------------------//
//- Status definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * Stores global status data.
 */
typedef struct _SbgLogStatusData
{
	uint32_t	timeStamp;												/*!< Time in us since the sensor power up. */
	uint16_t	generalStatus;											/*!< General status bitmask and enums. */
	uint16_t	reserved1;												/*!< Reserved status field for future use */
	uint32_t	comStatus;												/*!< Communication status bitmask and enums. */
	uint32_t	aidingStatus;											/*!< Aiding equipments status bitmask and enums. */
	uint32_t	reserved2;												/*!< Reserved status field for future use. */
	uint16_t	reserved3;												/*!< Reserved status field for future use. */
	uint32_t	uptime;													/*!< System uptime in seconds. */
} SbgLogStatusData;

//----------------------------------------------------------------------//
//- Communication status helpers methods                               -//
//----------------------------------------------------------------------//

/*!
 * Method used to read the CAN bus status from a communication status field.
 * \param[in]	status				Status field to extract the CAN bus status from it.
 * \return							The extracted CAN bus status.
 */
SBG_INLINE SbgEComCanBusStatus sbgEComLogStatusGetCanStatus(uint32_t status)
{
	return (SbgEComCanBusStatus)((status >> SBG_ECOM_CAN_STATUS_SHIFT) & SBG_ECOM_CAN_STATUS_MASK);
}

/*!
 * Method used to write the CAN bus status field.
 * \param[in]	canStatus			The CAN bus status to set.
 * \param[in]	masks				Bit mask to set.
 * \return							The build communication status field.
 */
SBG_INLINE uint32_t sbgEComLogStatusBuildCommunicationStatus(SbgEComCanBusStatus canStatus, uint32_t masks)
{
	//
	// Create the combined status field
	//
	return	((((uint32_t)canStatus)&SBG_ECOM_CAN_STATUS_MASK) << SBG_ECOM_CAN_STATUS_SHIFT) | masks;
}

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_STATUS message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseStatusData(SbgStreamBuffer *pInputStream, SbgLogStatusData *pOutputData);

/*!
 * Write data for the SBG_ECOM_LOG_STATUS message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteStatusData(SbgStreamBuffer *pOutputStream, const SbgLogStatusData *pInputData);

#endif
