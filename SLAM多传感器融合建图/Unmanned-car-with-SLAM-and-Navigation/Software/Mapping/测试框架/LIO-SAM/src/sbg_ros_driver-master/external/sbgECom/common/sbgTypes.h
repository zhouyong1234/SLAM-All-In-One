/*!
 *	\file		sbgTypes.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file that defines all scalar types.
 *	
 *	The platform endianness should be defined here.
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
#ifndef SBG_TYPES_H
#define SBG_TYPES_H

// Standard headers
#include <stdint.h>

//----------------------------------------------------------------------//
//- Limits definitions                                                 -//
//----------------------------------------------------------------------//
#define SBG_MIN_INT_24					(-8388608l)
#define SBG_MAX_INT_24					(8388607l)
#define SBG_MAX_UINT_24					(16777215ul)

#define SBG_MIN_INT_40					(-549755813887ll - 1)
#define SBG_MAX_INT_40					(549755813887ll)
#define SBG_MAX_UINT_40					(1099511627775ull)	

#define SBG_MIN_INT_48					(-140737488355327ll - 1)
#define SBG_MAX_INT_48					(140737488355327ll)
#define SBG_MAX_UINT_48					(281474976710655ull)	

#define SBG_MIN_INT_56					(-36028797018963967ll - 1)
#define SBG_MAX_INT_56					(36028797018963967ll)
#define SBG_MAX_UINT_56					(72057594037927935ull)	

//----------------------------------------------------------------------//
//- DEPRECATED: Scalar types definitions                               -//
//----------------------------------------------------------------------//
typedef unsigned	char			uint8;		//  8 bits
typedef unsigned	short			uint16;		// 16 bits
typedef unsigned	int				uint32;		// 32 bits
typedef unsigned	long long int	uint64;		// 64 bits

typedef signed		char			int8;		//  8 bits
typedef signed		short			int16;		// 16 bits
typedef signed		int				int32;		// 32 bits
typedef signed		long long int	int64;		// 64 bits


//----------------------------------------------------------------------//
//- Misc types definitions                                             -//
//----------------------------------------------------------------------//
typedef uint32_t						sbgIpAddress;					/*!< Define an IP v4 address stored in 4 bytes. The format is A.B.C.D, each component is 8 bits and stored in Big Endian. */

//------------------------------------------------------------------//
//- Type punning safe conversion unions                            -//
//------------------------------------------------------------------//

/*!
 * Used to get a uint32_t from a uint8_t array.
 */
typedef union _Uint8PtrToUint32Ptr
{
	uint8_t		*m_pointerUint8;				/*!< Set the address used to access the uint32_t. */
	uint32_t	*m_pointerUint32;				/*!< Store the unint32 value. */
} Uint8PtrToUint32Ptr;

/*!
 * Union used to convert a buffer or 2 unit8 two's complement values to a int16_t
 */
typedef union _Uint8ToInt16
{
	int16_t		value;
	uint8_t		buffer[2];
} Uint8ToInt16;

/*!
 * Union used to convert a buffer or 2 unit8 values to a uint16_t
 */
typedef union _Uint8ToUint16
{
	uint16_t	value;
	uint8_t		buffer[2];
} Uint8ToUint16;

/*!
 * Union used to convert a buffer or 4 unit8 two's complement values to a int32_t
 */
typedef union _Uint8ToInt32
{
	int32_t		value;
	uint8_t		buffer[4];
} Uint8ToInt32;

/*!
 * Union used to convert a buffer or 4 unit8 values to a uint32_t
 */
typedef union _Uint8ToUint32
{
	uint32_t	value;
	uint8_t		buffer[4];
} Uint8ToUint32;

/*!
 * Union used to convert a buffer or 8 unit8 two's complement values to a int64_t
 */
typedef union _Uint8ToInt64
{
	int64_t		value;
	uint8_t		buffer[8];
} Uint8ToInt64;

/*!
 * Union used to convert a buffer or 8 unit8 values to a uint64_t
 */
typedef union _Uint8ToUint64
{
	uint64_t	value;
	uint8_t		buffer[8];
} Uint8ToUint64;

/*!
 * Union that allows type punning (access to a floating point number bits)
 */
typedef union _FloatNint
{
	float		valF;
	int32_t		valI;
	uint32_t	valU;
} FloatNint;

/*!
 * Union that allows type punning (access to a double number bits)
 */
typedef union _DoubleNint
{
	double		valF;
	uint64_t	valU;
	int64_t		valI;
} DoubleNint;

/*!
 * Structure that splits a 64bits
 */
typedef struct _Split64
{
	uint32_t	high;
	uint32_t	low;
} Split64;

/*!
 * This structure defines a date
 */
typedef struct _DateStructure
{
	uint16_t	year;
	uint8_t		month;
	uint8_t		day;
} DateStructure;

#endif	/* SBG_TYPES_H */
