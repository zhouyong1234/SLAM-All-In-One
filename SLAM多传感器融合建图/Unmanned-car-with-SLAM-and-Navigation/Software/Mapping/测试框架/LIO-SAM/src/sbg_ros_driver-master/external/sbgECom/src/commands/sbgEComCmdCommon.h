/*!
 *	\file		sbgEComCmdCommon.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file groups all common definitions required by all commands.
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
#ifndef __SBG_ECOM_CMD_COMMON_H__
#define __SBG_ECOM_CMD_COMMON_H__

#include "../sbgECom.h"

//----------------------------------------------------------------------//
//- Defintions                                                         -//
//----------------------------------------------------------------------//

#define SBG_ECOM_DEFAULT_CMD_TIME_OUT	(500)	/*!< Default time out in ms for commands reception. */

/*!
 * List of all rejection modes for aiding inputs.
 */
typedef enum _SbgEComRejectionMode
{
	SBG_ECOM_NEVER_ACCEPT_MODE		= 0,		/*!< Measurement is not taken into account. */
	SBG_ECOM_AUTOMATIC_MODE			= 1,		/*!< Measurement is accepted and rejected automatically depending on consistency checks */
	SBG_ECOM_ALWAYS_ACCEPT_MODE		= 2			/*!< Measurement is always accepted. Should be used with caution */
} SbgEComRejectionMode;

/*!
 * List of all axis directions for modules/sensor alignment.
 */
typedef enum _SbgEComAxisDirection
{
	SBG_ECOM_ALIGNMENT_FORWARD		= 0,		/*!< IMU/module Axis is turned in vehicle's forward direction. */
	SBG_ECOM_ALIGNMENT_BACKWARD		= 1,		/*!< IMU/module Axis is turned in vehicle's backward direction. */
	SBG_ECOM_ALIGNMENT_LEFT			= 2,		/*!< IMU/module Axis is turned in vehicle's left direction. */
	SBG_ECOM_ALIGNMENT_RIGHT		= 3,		/*!< IMU/module Axis is turned in vehicle's right direction. */
	SBG_ECOM_ALIGNMENT_UP			= 4,		/*!< IMU/module Axis is turned in vehicle's up direction. */
	SBG_ECOM_ALIGNMENT_DOWN			= 5			/*!< IMU/module Axis is turned in vehicle's down direction. */
} SbgEComAxisDirection;

/*!
 * Common model information structure.
 * This is used for motion profile or Magnetometer,Gps, or other aiding sensor error model.
 */
typedef struct _SbgEComModelInfo
{
	uint32_t	id;									/*!< Identifier of the model */
	uint32_t	revision;							/*!< Revision of the model */
} SbgEComModelInfo;

//----------------------------------------------------------------------//
//- Common command reception operations                                -//
//----------------------------------------------------------------------//

/*!
 *	Wait until any command that is not a output log is recevied during a specific time out.
 *	All binary logs received during this time are handled trough the standard callback system.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[out]	pMsgClass				Pointer used to hold the received command class.
 *	\param[out]	pMsg					Pointer used to hold the received command ID.
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 *										SBG_TIME_OUT if the command hasn't been received withint the specified time out.
 */
SbgErrorCode sbgEComReceiveAnyCmd(SbgEComHandle *pHandle, uint8_t *pMsgClass, uint8_t *pMsg, void *pData, size_t *pSize, size_t maxSize, uint32_t timeOut);

/*!
 *	Wait for a specific command to be received given a time out.
 *	All binary logs received during this time are handled trough the standard callback system.
 *	\param[in]	pHandle					A valid sbgECom handle.
 *	\param[out]	msgClass				Command class we expect to receive
 *	\param[out]	msg						Message id we expect to receive
 *	\param[out]	pData					Allocated buffer used to hold received data field.
 *	\param[out]	pSize					Pointer used to hold the received data field size.
 *	\param[in]	maxSize					Max number of bytes that can be stored in the pData buffer.
 *	\param[in]	timeOut					Time out in ms during which we can receive the command.
 *	\return								SBG_NO_ERROR if we have received a valid frame.<br>
 *										SBG_NOT_READY if we haven't received a valid frame or if the serial buffer is empty.<br>
 *										SBG_INVALID_CRC if the received frame has an invalid CRC.<br>
 *										SBG_NULL_POINTER if an input parameter is NULL.<br>
 *										SBG_BUFFER_OVERFLOW if the received frame payload couldn't fit into the pData buffer.
 *										SBG_TIME_OUT if the command hasn't been received withint the specified time out.
 */
SbgErrorCode sbgEComReceiveCmd(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, void *pData, size_t *pSize, size_t maxSize, uint32_t timeOut);

//----------------------------------------------------------------------//
//- ACK related commands  operations                                   -//
//----------------------------------------------------------------------//

/*!
 * Wait for an ACK for a specified amount of time.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	msgClass					The message class that we want to check
 * \param[in]	msg							The message ID that we want to check
 * \param[in]	timeOut						Time out in ms during which we can receive the ACK.
 * \return									SBG_NO_ERROR if the ACK has been received.
 */
SbgErrorCode sbgEComWaitForAck(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, uint32_t timeOut);

/*!
 * Send an ACK for a specific command with an associated error code.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	msgClass					The message class that we want to send
 * \param[in]	msg							The message ID that we want to send.
 * \param[in]	cmdError					The associated error code.
 * \return									SBG_NO_ERROR if the ACK has been sent.
 */
SbgErrorCode sbgEComSendAck(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, SbgErrorCode cmdError);

//----------------------------------------------------------------------//
//- Generic command definitions                                        -//
//----------------------------------------------------------------------//

/*!
 * Generic function to set an error model ID
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	msgClass					Original message class
 * \param[in]	msg							Original message ID
 * \param[in]	modelId						Model ID to set
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGenericSetModelId(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, uint32_t modelId);

/*!
 * Generic function to get an error model ID
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	msgClass					Original message class
 * \param[in]	msg							Original message ID
 * \param[out]	pModelId					Returns the currently used model ID.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGenericGetModelId(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, uint32_t *pModelId);

/*!
 * Generic function to retrieve error model information.
 *
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	msgClass					Original message class
 * \param[in]	msg							Original message ID
 * \param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain model info.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGenericGetModelInfo(SbgEComHandle *pHandle, uint8_t msgClass, uint8_t msg, SbgEComModelInfo *pModelInfo);

#endif
