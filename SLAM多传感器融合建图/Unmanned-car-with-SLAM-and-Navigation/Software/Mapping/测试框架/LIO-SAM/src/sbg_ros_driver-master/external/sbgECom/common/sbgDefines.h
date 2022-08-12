/*!
 *	\file		sbgDefines.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file that contains all common definitions.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2015, SBG Systems SAS. All rights reserved.
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
#ifndef SBG_DEFINES_H
#define SBG_DEFINES_H

// Standard headers
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

//
// XXX If NDEBUG is defined, most libraries define assert() as ((void)0), which may
// cause "defined but not used" warnings. Redefine assert() in a way that safely
// prevents this warning, i.e. without triggering the expression side effects.
//
#ifdef NDEBUG
#undef assert
#define assert(expression) ((void)sizeof(expression))
#endif // NDEBUG

/*!
 * Macro used to handle export and import methods of the sbgCommon library
 */
#ifdef _MSC_VER
	#if defined(SBG_COMMON_LIB_API_EXPORT)
		#define SBG_COMMON_LIB_API __declspec(dllexport)
	#elif defined(SBG_COMMON_LIB_API_IMPORT)
		#define SBG_COMMON_LIB_API __declspec(dllimport)
	#else
		#define SBG_COMMON_LIB_API
	#endif
#else
	#define SBG_COMMON_LIB_API
#endif

//----------------------------------------------------------------------//
//- Global definitions                                                 -//
//----------------------------------------------------------------------//
#ifndef SBG_DISABLE
	#define SBG_DISABLE	(0)
#endif

#ifndef SBG_ENABLE
	#define SBG_ENABLE	(1)
#endif

#ifndef FALSE
    #define FALSE        (0)
#endif

#ifndef TRUE
    #define TRUE        (1)
#endif

#ifndef NULL
	#define NULL		(0)
#endif

#ifndef SBG_INVALID_HANDLE
	#define SBG_INVALID_HANDLE	(0x00000000u)
#endif

#ifndef SBG_NOT_FOUND
	#define SBG_NOT_FOUND		(0xFFFFFFFFu)
#endif

#ifndef SBG_UNDEFINED
	#define SBG_UNDEFINED		(0xFFFFFFFFu)
#endif

#ifndef SBG_ARRAY_SIZE
	#define SBG_ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#endif

#ifndef SBG_STRLEN
	#define SBG_STRLEN(s)		(sizeof(s) - 1)
#endif

#ifndef SBG_CONTAINER_OF
	#define SBG_CONTAINER_OF(ptr, type, member)		((type *)((char *)(ptr) - offsetof(type, member)))
#endif

#ifndef SBG_LIKELY
	#if defined(__GNUC__)
		#define SBG_LIKELY(expr)		__builtin_expect((bool)(expr), true)
	#else
		#define SBG_LIKELY(expr)		(expr)
	#endif
#endif

#ifndef SBG_UNLIKELY
	#if defined(__GNUC__)
		#define SBG_UNLIKELY(expr)		__builtin_expect((bool)(expr), false)
	#else
		#define SBG_UNLIKELY(expr)		(expr)
	#endif
#endif

/*!
 * XXX Visual C (not C++) doesn't provide anything to implement typeof(). As a result,
 * This macro is private and shouldn't be relied on.
 */
#ifndef __SBG_TYPEOF
	#ifdef __cplusplus
		#define __SBG_TYPEOF(x)			decltype(x)
	#elif defined(__GNUC__)
		#define __SBG_TYPEOF(x)			typeof(x)
	#elif defined(__TI_COMPILER_VERSION__)
		#define __SBG_TYPEOF(x)			typeof(x)
	#endif
#endif

#ifndef SBG_CONST_CAST_AA
	#ifdef __SBG_TYPEOF
		#define SBG_CONST_CAST_AA(x)	((const __SBG_TYPEOF((x)[0][0])(*)[SBG_ARRAY_SIZE((x)[0])])(x))
	#else
		#define SBG_CONST_CAST_AA(x)	x
	#endif
#endif

#ifndef SBG_CONST_CAST_PP
	#ifdef __SBG_TYPEOF
		#define SBG_CONST_CAST_PP(x)	((const __SBG_TYPEOF(**(x))**)(x))
	#else
		#define SBG_CONST_CAST_PP(x)	x
	#endif
#endif

/*!
 *	__BASE_FILE__ is gcc specific
 */
#ifndef __GNUC__
#ifndef __BASE_FILE__
	#define __BASE_FILE__ __FILE__
#endif
#endif

#ifdef __cplusplus
	#define SBG_DELETE(p)		if (p){delete (p); (p) = NULL;}
	#define SBG_DELETE_ARRAY(p)	if (p){delete[] (p); (p) = NULL;}
	#define SBG_FREE(p)			if (p){free(p); (p) = NULL;}
	#define SBG_FREE_ARRAY(p)	if (p){free(p); (p) = NULL;}
#else
	#define SBG_DELETE			if (p){free(p); (p) = NULL;}
	#define SBG_DELETE_ARRAY	if (p){free(p); (p) = NULL;}
	#define SBG_FREE(p)			if (p){free(p); (p) = NULL;}
	#define SBG_FREE_ARRAY(p)	if (p){free(p); (p) = NULL;}
#endif

//----------------------------------------------------------------------//
//- Compiler definitions                                               -//
//----------------------------------------------------------------------//

/*!
 *	Macro used to abstract the compiler specific inline keyword.
 */
#ifndef SBG_INLINE
	#if defined(_MSC_VER)
		#define SBG_INLINE			__inline
	#else
		#define SBG_INLINE          static inline
	#endif
#endif

/*!
 *	Macro used to avoid compiler warning when a variable is not used.
 */
#ifndef SBG_UNUSED_PARAMETER
	#define SBG_UNUSED_PARAMETER(x)		(void)(x)
#endif

/*!
 * The fallthrough attribute is used to avoid compiler warning in swith case statements
 * when an intentional break is missing
 */
#ifndef SBG_FALLTHROUGH
	#if __cplusplus >= 201703L
		#define SBG_FALLTHROUGH			[[fallthrough]]					/* introduced in C++ 17 */
	#elif defined(__GNUC__)
		#define SBG_FALLTHROUGH			__attribute__ ((fallthrough))
	#else
		#define SBG_FALLTHROUGH
	#endif
#endif

//----------------------------------------------------------------------//
//- Macro used to defined packed structures                            -//
//----------------------------------------------------------------------//

/*!
 * This macro is used to define a new section of packed structures.
 * All structures defined after this macro will be packed.
 */
#ifdef __GNUC__
	#define SBG_BEGIN_PACKED()
#elif defined(__TI_COMPILER_VERSION__)
	#define SBG_BEGIN_PACKED()
#elif defined(_MSC_VER)
	#define SBG_BEGIN_PACKED()	__pragma(pack(push, 1))
#else
	#error you must byte-align these structures with the appropriate compiler directives
#endif

/*!
 * This macro is used to specify that a structure is packed.
 */
#ifdef __GNUC__
	#define SBG_PACKED			__attribute__((packed))
#elif defined(__TI_COMPILER_VERSION__)
	#define SBG_PACKED			__attribute__((packed))
#elif defined(_MSC_VER)
	#define SBG_PACKED
#else
	#error you must byte-align these structures with the appropriate compiler directives
#endif

/*!
 * This macro is used to close the section of packed structures and return to the default packing.
 */
#ifdef __GNUC__
	#define SBG_END_PACKED()
#elif defined(__TI_COMPILER_VERSION__)
	#define SBG_END_PACKED()
#elif defined(_MSC_VER)
	#define SBG_END_PACKED()		__pragma(pack(pop))
#else
	#error you must byte-align these structures with the appropriate compiler directives
#endif

//----------------------------------------------------------------------//
//- Deprecation definitions				                               -//
//----------------------------------------------------------------------//

/*!
 *	Macro used to indicate that a function is deprecated.
 */
#ifdef __GNUC__
	#define SBG_DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(__TI_COMPILER_VERSION__)
	#define SBG_DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
	#define SBG_DEPRECATED(func) __declspec(deprecated) func
#else
	//#warning "WARNING: You need to implement SBG_DEPRECATED for this compiler"
	#define SBG_DEPRECATED(func) func
#endif

/*!
 *	Macro used to indicate that a macro is deprecated.
 */
#ifdef __GNUC__
	#define SBG_DEPRECATED_MACRO(func) __pragma(deprecated(func))
#elif defined(_MSC_VER)
#define SBG_DEPRECATED_MACRO(func) __pragma(deprecated(func))
#else
	//#warning "WARNING: You need to implement SBG_DEPRECATED_MACRO for this compiler"
	#define SBG_DEPRECATED_MACRO(func) func
#endif

//----------------------------------------------------------------------//
//- Basic maths definitions                                            -//
//----------------------------------------------------------------------//
#ifndef SBG_PI
	#define SBG_PI 3.14159265358979323846
#endif

#ifndef SBG_PI_F
	#define SBG_PI_F 3.14159265358979323846f
#endif

/*!
 *	Returns the absolute value of x.
 *
 *	\param[in]	x					Signed integer value.
 *	\return							The absolute value of x.
 */
#ifndef sbgAbs
	#define sbgAbs(x)				(((x) < 0) ? -(x) : (x))
#endif

/*!
 *	Returns the maximum between a and b
 *	\param[in]	a					First operand.
 *	\param[in]	b					Second operand.
 *	\return							The maximum between a and b.
 */
#ifndef sbgMax
	#define sbgMax(a,b)            (((a) > (b)) ? (a) : (b))
#endif

/*!
 *	Returns the minimum between a and b
 *	\param[in]	a					First operand.
 *	\param[in]	b					Second operand.
 *	\return							The minimum between a and b.
 */
#ifndef sbgMin
	#define sbgMin(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/*!
 * Clamp a value between minValue and maxValue ie minValue <= value <= maxValue
 * \param[in]	value				First operand.
 * \param[in]	minValue			First operand.
 * \param[in]	maxValue			Second operand.
 * \return							The clamped value.
 */
#ifndef sbgClamp
	#define sbgClamp(value, minValue, maxValue)            (((value) < (minValue))?(minValue): ((value) > (maxValue)?maxValue:value))
#endif

/*!
 * Integer division with a result rounded up.
 * \param[in]	n					Dividend.
 * \param[in]	d					Divisor.
 */
#ifndef sbgDivCeil
	#define sbgDivCeil(n, d)		(((n) + (d) - 1) / (d))
#endif

/*!
 *	Convert an angle from radians to degrees using double precision.
 *	\param[in]	angle				The angle to convert in radians.
 *	\return							The converted angle in degrees.
 */
SBG_INLINE double sbgRadToDegD(double angle)
{
	return angle * 180.0 / SBG_PI;
}

/*!
 *	Convert an angle from degrees to radians using double precision.
 *	\param[in]	angle				The angle to convert in degrees.
 *	\return							The converted angle in radians.
 */
SBG_INLINE double sbgDegToRadD(double angle)
{
	return angle * SBG_PI / 180.0;
}

/*!
 *	Convert an angle from radians to degrees using single (float) precision.
 *	\param[in]	angle				The angle to convert in radians.
 *	\return							The converted angle in degrees.
 */
SBG_INLINE float sbgRadToDegF(float angle)
{
	return angle * 180.0f / SBG_PI_F;
}

/*!
 *	Convert an angle from degrees to radians using single (float) precision.
 *	\param[in]	angle				The angle to convert in degrees.
 *	\return							The converted angle in radians.
 */
SBG_INLINE float sbgDegToRadF(float angle)
{
	return angle * SBG_PI_F / 180.0f;
}

#endif	/* __SBG_DEFINES_H__ */
