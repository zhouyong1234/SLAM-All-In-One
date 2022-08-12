/*!
 *	\file		sbgNetwork.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		15 September 2015
 *
 *	\brief		Useful methods for Network handling such as ip addresses.
 *
 *	IP v4 address is stored in memory with a uint32_t.
 *	Each address component A.B.C.D is stored in 8 bits using the network
 *	endianess ie Big Endian.
 *
 *	We thus have the following memory organisation:
 *	
 *	In Little Endian:
 *		|LSB|   |   |MSB|
 *		| A | B | C | D |
 *
 *	In Big Endian:
 *		|MSB|   |   |LSB|
 *		| A | B | C | D |
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

#ifndef SBG_NETWORK_H
#define SBG_NETWORK_H

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Common IPv4 definitions                                            -//
//----------------------------------------------------------------------//
#define SBG_IPV4_UNSPECIFIED_ADDR	sbgIpAddr(0, 0, 0, 0)				/*!< This represents an undefined IP address. */
#define SBG_IPV4_BROADCAST_ADDR		sbgIpAddr(255, 255, 255, 255)		/*!< Broadcast IP address used to address all devices within the same network. */

//----------------------------------------------------------------------//
//- IP setters / getters                                               -//
//----------------------------------------------------------------------//

/*!
 * Build an IP V4 address in the form a.b.c.d
 * \param[in]	a				First 8 bits IP address.
 * \param[in]	b				Second 8 bits IP address.
 * \param[in]	c				Third 8 bits IP address.
 * \param[in]	d				Last 8 bits IP address.
 * \return						An initialized IP address object.
 */
SBG_INLINE sbgIpAddress sbgIpAddr(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
#if SBG_CONFIG_BIG_ENDIAN == 1
	return (a << 24) | (b << 16) | (c << 8) | d;
#else
	return a | (b << 8) | (c << 16) | (d << 24);
#endif
}

/*!
 * Return the first A field of of an IP v4 address of the form A.B.C.D
 * \param[in]	ipAddr				An sbgIpAddress to convert.
 * \return							The A field of the IP address.
 */
SBG_INLINE uint8_t sbgIpAddrGetA(sbgIpAddress ipAddr)
{
#if SBG_CONFIG_BIG_ENDIAN == 1
	return (uint8_t)((ipAddr & 0xFF000000) >> 24);
#else
	return (uint8_t)((ipAddr & 0x000000FF));
#endif
}

/*!
* Return the first B field of of an IP v4 address of the form A.B.C.D
* \param[in]	ipAddr				An sbgIpAddress to convert.
* \return							The B field of the IP address.
*/
SBG_INLINE uint8_t sbgIpAddrGetB(sbgIpAddress ipAddr)
{
#if SBG_CONFIG_BIG_ENDIAN == 1
	return (uint8_t)((ipAddr & 0x00FF0000) >> 16);
#else
	return (uint8_t)((ipAddr & 0x0000FF00) >> 8);
#endif
}

/*!
* Return the first C field of of an IP v4 address of the form A.B.C.D
* \param[in]	ipAddr				An sbgIpAddress to convert.
* \return							The C field of the IP address.
*/
SBG_INLINE uint8_t sbgIpAddrGetC(sbgIpAddress ipAddr)
{
#if SBG_CONFIG_BIG_ENDIAN == 1
	return (uint8_t)((ipAddr & 0x0000FF00) >> 8);
#else
	return (uint8_t)((ipAddr & 0x00FF0000) >> 16);
#endif
}

/*!
* Return the first D field of of an IP v4 address of the form A.B.C.D
* \param[in]	ipAddr				An sbgIpAddress to convert.
* \return							The D field of the IP address.
*/
SBG_INLINE uint8_t sbgIpAddrGetD(sbgIpAddress ipAddr)
{
#if SBG_CONFIG_BIG_ENDIAN == 1
	return (uint8_t)((ipAddr & 0x000000FF));
#else
	return (uint8_t)((ipAddr & 0xFF000000) >> 24);
#endif
}

//----------------------------------------------------------------------//
//- IP manipulation methods                                            -//
//----------------------------------------------------------------------//

/*!
 * Convert an ip to a string of the form A.B.C.D
 * \param[in]	ipAddr						IP address to convert to a string.
 * \param[out]	pBuffer						Pointer on an allocated buffer than can hold ip address as a string.
 * \param[in]	maxSize						Maximum number of chars that can be stored in pBuffer including the NULL char.
 */
SBG_COMMON_LIB_API void sbgNetworkIpToString(sbgIpAddress ipAddr, char *pBuffer, size_t maxSize);

/*!
 * Convert an ip address stored in a string of the form A.B.C.D to an sbgIpAddress object.
 * \param[in]	pBuffer						IP address as a string of the form A.B.C.D
 * \return									IP address parsed from the string or 0.0.0.0 if the IP is invalid.
 */
SBG_COMMON_LIB_API sbgIpAddress sbgNetworkIpFromString(const char *pBuffer);

//----------------------------------------------------------------------//
//- IP operations                                                      -//
//----------------------------------------------------------------------//

/*!
 * Given an ip address and the netmask, returns the network part (ip & subnetMask)
 * \param[in]	ipAddress						The ip address stored in an uint32_t (host endianness).
 * \param[in]	netmask							The netmask stored in an uint32_t (host endianness).
 * \return										The network part of the ip address.
 */
SBG_INLINE sbgIpAddress sbgIpGetNetworkAddr(sbgIpAddress ipAddress, sbgIpAddress netmask)
{
	return (ipAddress & netmask);
}

/*!
 * Given an ip address and the netmask, returns the host part (ip & ~subnetMask)
 * \param[in]	ipAddress						The ip address stored in an uint32_t (host endianness).
 * \param[in]	netmask							The netmask stored in an uint32_t (host endianness).
 * \return										The host part of the ip address.
 */
SBG_INLINE sbgIpAddress sbgIpGetHostAddr(sbgIpAddress ipAddress, sbgIpAddress netmask)
{
	return (ipAddress & ~netmask);
}

//----------------------------------------------------------------------//
//- IP validation methods                                              -//
//----------------------------------------------------------------------//

/*!
 * Returns true if the provided IP address is unspecified ie (0.0.0.0)
 * \param[in]	ipAddress						The ip address to test
 * \return										true if the ip address is unspecified or false otherwise.
 */
SBG_INLINE bool sbgIpAddressIsUnspecified(sbgIpAddress ipAddress)
{
	if (ipAddress == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*!
 * Check if an IpV4 address is valid. The ip address format is A.B.C.D and A should respect 0 < A < 224
 * \param[in]	ipAddress						The ip address stored in an uint32_t (host endianness).
 * \return										true if the ip address is valid ie contiguous.
 */
SBG_INLINE bool sbgIpAddressValid(sbgIpAddress ipAddress)
{
	//
	// Check the if A part of the ip address is within 1 and 223
	//
	if ((sbgIpAddrGetA(ipAddress) > 0) && (sbgIpAddrGetA(ipAddress) < 224))
	{
		//
		// The ip address is valid
		//
		return true;
	}
	else
	{
		//
		// The ip address is not valid
		//
		return false;
	}
}

/*!
 * Given an ip address and the netmask, returns true if this ip address is within the subnet.
 * \param[in]	ipAddress						The ip address stored in an uint32_t (host endianness).
 * \param[in]	netmask							The netmask stored in an uint32_t (host endianness).
 * \return										true if this ip address is within the subnet or false otherwise.
 */
SBG_INLINE sbgIpAddress sbgIpAddrWithinSubnet(sbgIpAddress ipAddress, sbgIpAddress netmask)
{
	//
	// Just check if the host part is equals to zero
	//
	if (sbgIpGetHostAddr(ipAddress, netmask) == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/*!
 * Check if two ip addresses are in the same network given the subnet.
 * \param[in]	firstIpAddr						The first IP address to check.
 * \param[in]	secondIpAddr					The second IP address to check.
 * \param[in]	netmask							The netmask of the network.
 * \return										true if the two ip addresses are in the same network.
 */
SBG_INLINE bool sbgIpAddrIsSameNetwork(sbgIpAddress firstIpAddr, sbgIpAddress secondIpAddr, sbgIpAddress netmask)
{
	if ((firstIpAddr & netmask) == (secondIpAddr & netmask))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*!
 * Check if an IpV4 netmask is valid, the mask should be contiguous (1111 followed by 0)
 * \param[in]	netmask							The netmask stored in an uint32_t (host endianness).
 * \return										true if the netmask is valid ie contiguous.
 */
SBG_COMMON_LIB_API bool sbgIpNetMaskValid(sbgIpAddress netmask);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif /* SBG_NETWORK_H */
