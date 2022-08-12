/*!
 *	\file		sbgEComCmdEthernet.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		14 November 2016
 *
 *	\brief		This file implements SbgECom commands related to Ethernet configuration.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2016, SBG Systems SAS. All rights reserved.
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
#ifndef __SBG_ECOM_CMD_ETHERNET_H__
#define __SBG_ECOM_CMD_ETHERNET_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Ethernet interface configuration								   -//
//----------------------------------------------------------------------//

/*!
 * Enum that defines the different type of IP acquisition method.
 */
typedef enum _SbgEComEthernetMode
{
	SBG_ECOM_ETHERNET_DHCP			= 0,			/*!< The TCP/IP configuration should be acquired from a DHCP server. */
	SBG_ECOM_ETHERNET_STATIC		= 1				/*!< The TCP/IP configuration is manually defined. */
} SbgEComEthernetMode;

/*!
 * Structure that contains all Ethernet configuration or settings.
 */
typedef struct _SbgEComEthernetConf
{
	SbgEComEthernetMode	mode;						/*!< Define how the device will acquiere its IP address, either DHCP or Static. */
	sbgIpAddress		ipAddress;					/*!< For static mode, defines the device IP address. */
	sbgIpAddress		netmask;					/*!< For static mode, defines the device net mask. */
	sbgIpAddress		gateway;					/*!< For static mode, defines the gateway to use. */
	sbgIpAddress		dns1;						/*!< For static mode, defines the primary DNS to use. */
	sbgIpAddress		dns2;						/*!< For static mode, defines the secondary DNS to use. */
} SbgEComEthernetConf;

//----------------------------------------------------------------------//
//- Public methods                                                     -//
//----------------------------------------------------------------------//

/*!
 * Get the configuration for the Ethernet interface.
 * Warning: this method only returns the Ethernet configuration and NOT the ip address currently used by the device.
 * You should rather use sbgEComEthernetInfo to retreive the current assigned IP.
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pEthernetConf				Poiner to a SbgEComEthernetConf struct that holds the read configuration from the device.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComEthernetGetConf(SbgEComHandle *pHandle, SbgEComEthernetConf *pEthernetConf);

/*!
 * Set the configuration for the Ethernet interface.
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[in]	pEthernetConf				Poiner to a SbgEComEthernetConf struct that holds the new configuration to apply.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComEthernetSetConf(SbgEComHandle *pHandle, const SbgEComEthernetConf *pEthernetConf);

/*!
 * Get the current assigned and used IP address as well as network inforamtion.
 * In opposition to sbgEComEthernetGetConf, this method will not return the Ethernet configuration.
 * It will rather return the IP address currently used by the device.
 * \param[in]	pHandle						A valid sbgECom handle.
 * \param[out]	pEthernetConf				Poiner to a SbgEComEthernetConf struct that holds the read IP settings from the device.
 * \return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComEthernetInfo(SbgEComHandle *pHandle, SbgEComEthernetConf *pEthernetConf);

#endif
