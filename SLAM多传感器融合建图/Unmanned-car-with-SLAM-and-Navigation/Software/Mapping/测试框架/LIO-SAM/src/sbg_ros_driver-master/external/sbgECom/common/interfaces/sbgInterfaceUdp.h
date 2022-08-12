/*!
 * \file		sbgInterfaceUdp.h
 * \author		SBG Systems
 * \date		05 February 2013
 *
 * \brief		This file implements an UDP interface.
 *
 * \section CodeCopyright Copyright Notice
 * Copyright (C) 2013-2019, SBG Systems SAS. All rights reserved.
 *
 * This source code is intended for use only by SBG Systems SAS and
 * those that have explicit written permission to use it from
 * SBG Systems SAS.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 */

#ifndef SBG_INTERFACE_UDP_H
#define SBG_INTERFACE_UDP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sbgInterface.h"

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

#define SBG_INTERFACE_UDP_PACKET_MAX_SIZE		(1400)

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores all internal data used by the UDP interface.
 */
typedef struct _SbgInterfaceUdp
{
	void			*pUdpSocket;					/*!< The socket used to send and / or receive some UDP data. */
	sbgIpAddress	 remoteAddr;					/*!< IP address to send data to. */
	uint32_t		 remotePort;					/*!< Ethernet port to send data to. */
	uint32_t		 localPort;						/*!< Ethernet port on which the interface is listening. */
} SbgInterfaceUdp;

//----------------------------------------------------------------------//
//- Public functions                                                   -//
//----------------------------------------------------------------------//

/*!
 * Initialize an unconnected UDP interface for read and write operations.
 *
 * If the remote address and port are zero, the interface waits for the first packet received on the
 * local port and uses the source of that packet as its remote host. This provides a convenient way
 * to create "server" UDP interfaces.
 *
 * \param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 * \param[in]	remoteAddr						IP address to send data to.
 * \param[in]	remotePort						Ethernet port to send data to.
 * \param[in]	localPort						Ehternet port on which the interface is listening.
 * \return										SBG_NO_ERROR if the interface has been created.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceUdpCreate(SbgInterface *pHandle, sbgIpAddress remoteAddr, uint32_t remotePort, uint32_t localPort);

/*!
 * Destroy an interface initialized using sbgInterfaceUdpCreate.
 *
 * \param[in]	pInterface						Pointer on a valid UDP interface created using sbgInterfaceUdpCreate.
 * \return										SBG_NO_ERROR if the interface has been closed and released.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceUdpDestroy(SbgInterface *pHandle);

/*!
 * Define if a socket can send broadcasted packets.
 *
 * \param[in]	pInterface						Pointer on a valid UDP interface created using sbgInterfaceUdpCreate.
 * \param[in]	allowBroadcast					Set to true to allow this socket to send broadcasted UDP packets.
 * \return										SBG_NO_ERROR if the allow broadcast status has been changed.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceUdpAllowBroadcast(SbgInterface *pHandle, bool allowBroadcast);

/*!
 * Try to write some data to an interface.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceUdpWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite);

/*!
 * Try to read some data from an interface.
 *
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32_t used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgInterfaceUdpRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead);

#ifdef __cplusplus
}
#endif

#endif /* SBG_INTERFACE_UDP_H */
