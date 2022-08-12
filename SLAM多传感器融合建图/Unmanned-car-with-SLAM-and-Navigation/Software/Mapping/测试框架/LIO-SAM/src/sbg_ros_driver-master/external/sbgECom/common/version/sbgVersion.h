/*!
 *	\file		sbgVersion.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		23 April 2015
 *
 *	\brief		Helper methods and definitions used to handle version.
 *
 *	Version information is stored within a single uint32_t.
 *	There are two different versions schemes to better handle software revisions and file format / hardware ones.
 *	
 *	Software version are only used for firmware and software.
 *		This versions is defined as a <Major>.<Minor>.<Build>-<Qualifier> for example: 1.3.1845-RC
 *		Major and Minor can range from 0 to 63.
 *		Build is an uint16_t ranging from 0 to 65535.
 *		Qualifier is an enum encoded on 3 bits.
 *
 *	Basic version is used for hardware or file formats and is the legacy one for software.
 *		This versions is defined as a <Major>.<Minor>.<Revision>.<Build>
 *		Each element is stored in an uint8_t.
 *		The Major version should NEVER go above 31
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
#ifndef SBG_VERSION_H
#define SBG_VERSION_H

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Version related definitions                                        -//
//----------------------------------------------------------------------//
#define SBG_VERSION_SOFT_SCHEME						(0x00000001u << 31)	/*!< Set to 1 to indicate a version number that uses the software scheme. Set to 0 means a basic version scheme. */

#define SBG_VERSION_SOFT_SCHEME_QUALIFIER_MASK		(0x07)				/*!< Mask used to keep only the version qualifier enum. */
#define SBG_VERSION_SOFT_SCHEME_QUALIFIER_SHIFT		(28)				/*!< Bitshift to apply to get the qualifier enum. */

#define SBG_VERSION_SOFT_SCHEME_MAJOR_MASK			(0x3F)				/*!< Mask used to keep only the major version field. */
#define SBG_VERSION_SOFT_SCHEME_MAJOR_SHIFT			(22)				/*!< Bitshift used to get the major version field. */

#define SBG_VERSION_SOFT_SCHEME_MINOR_MASK			(0x3F)				/*!< Mask used to keep only the minor version field. */
#define SBG_VERSION_SOFT_SCHEME_MINOR_SHIFT			(16)				/*!< Bitshift used to get the minor version field. */

#define SBG_VERSION_SOFT_SCHEME_BUILD_MASK			(0xFFFF)			/*!< Mask used to keep only the build version field. */
#define SBG_VERSION_SOFT_SCHEME_BUILD_SHIFT			(0)					/*!< Bitshift used to get the build version field. */

/*!
 *	Version qualifier enum definition.
 *	This enum is used to identify if we have a basic version scheme or a software one.
 *	If it's a software one, we can easly know if the software version is a beta, a stable one.
 */
typedef enum _SbgVersionQualifier
{
	SBG_VERSION_QUALIFIER_DEV				= 0,							/*!< Development only version or pre-alpha. Customer shouldn't get this version qualifier. */
	SBG_VERSION_QUALIFIER_ALPHA				= 1,							/*!< Alpha version, missing features, can be unstable and could cause crashes or data loss. API can still change. */
	SBG_VERSION_QUALIFIER_BETA				= 2,							/*!< Beta version, features are freezed, can be unstable and could cause crashes or data loss. API shouldn't change. */
	SBG_VERSION_QUALIFIER_RC				= 3,							/*!< Release Candidate, features are freezed, with no known bug. API is freezed. */
	SBG_VERSION_QUALIFIER_STABLE			= 4,							/*!< Stable release, the version is the standard delivered one. */
	SBG_VERSION_QUALIFIER_HOT_FIX			= 5								/*!< Hot fixes were applied on a stable release. It should be bug fixes. This qualifier is temporary as the version should return to stable release as soon as the test procedure has been performed. */
} SbgVersionQualifier;

/*!
 *	Version qualifier enum definition.
 *	This enum is used to identify if we have a basic version scheme or a software one.
 *	If it's a software one, we can easly know if the software version is a beta, a stable one.
 */
typedef enum _SbgVersionCmpThresold
{
	SBG_VERSION_CMP_THRESOLD_MAJOR			= 0,							/*!< Compare only the major field. */
	SBG_VERSION_CMP_THRESOLD_MINOR			= 1,							/*!< Compare the major and minor fields. */
	SBG_VERSION_CMP_THRESOLD_REVISION		= 2,							/*!< Compare the major, minor and revision fields (only for basic versions). */
	SBG_VERSION_CMP_THRESOLD_BUILD			= 3,							/*!< Compare the major, minor, revision and build fields. */
	SBG_VERSION_CMP_THRESOLD_QUALIFIER		= 4								/*!< Compare the major, minor, revision, build and qualifier fields (only for software scheme). */
} SbgVersionCmpThresold;

/*!
 * This structure contains all the fields provided by a version number.
 *	It handles both basic and software version numbers.
 */
typedef struct _SbgVersion
{
	bool					softwareScheme;				/*!< Set to true if it's a software scheme or false if it's a basic one. */
	SbgVersionQualifier		qualifier;					/*!< Qualifier of the current version */
	uint8_t					major;						/*!< Major version */
	uint8_t					minor;						/*!< Minor version */
	uint8_t					rev;						/*!< Revision number - left to 0 in case of software version */
	uint16_t				build;						/*!< Build number */
} SbgVersion;

//----------------------------------------------------------------------//
//- Macro definitions for encode versions                              -//
//----------------------------------------------------------------------//

/*!
 *	Compile a version number using the basic scheme based on major, minor, revision and build information
 *	\param[in]	maj							The major version between 0 to 127.
 *	\param[in]	min							The minor version between 0 to 255.
 *	\param[in]	rev							The revision version between 0 to 255.
 *	\param[in]	build						The build number between 0 to 255.
 *	\return									The encoded version number.
 */
#define SBG_VERSION_BASIC(major, minor, rev, build)					(	(((uint32_t)(major)) << 24)	|		\
																		(((uint32_t)(minor)) << 16)	|		\
																		(((uint32_t)(rev)) << 8)		|		\
																		(((uint32_t)(build)))			)

/*!
 *	Compile a version number using the software scheme based on major, minor, build, qualifier.
 *	\param[in]	major						The major version between 0 to 63.
 *	\param[in]	minor						The major version between 0 to 63.
 *	\param[in]	build						The major version between 0 to 65535.
 *	\param[in]	qualifier					The version qualifier within the SbgVersionQualifier enum.
 *	\return									The encoded version number.
 */
#define SBG_VERSION_SOFTWARE(major, minor, build, qualifier)		(	SBG_VERSION_SOFT_SCHEME |																						\
																		((((uint32_t)(qualifier)) & SBG_VERSION_SOFT_SCHEME_QUALIFIER_MASK) << SBG_VERSION_SOFT_SCHEME_QUALIFIER_SHIFT) |	\
																		((((uint32_t)(major)) & SBG_VERSION_SOFT_SCHEME_MAJOR_MASK) << SBG_VERSION_SOFT_SCHEME_MAJOR_SHIFT)		|		\
																		((((uint32_t)(minor)) & SBG_VERSION_SOFT_SCHEME_MINOR_MASK) << SBG_VERSION_SOFT_SCHEME_MINOR_SHIFT)		|		\
																		((((uint32_t)(build)) & SBG_VERSION_SOFT_SCHEME_BUILD_MASK) << SBG_VERSION_SOFT_SCHEME_BUILD_SHIFT)		)

//----------------------------------------------------------------------//
//- Version encoding / decoding methods                                -//
//----------------------------------------------------------------------//

/*!
 *	Fill a SbgVersion structure based on an uint32_t that stores the 'compiled' version information.
 *	\param[in]	encodedVersion											The version information stored within a uint32_t.
 *	\param[out]	pVersionInfo											Pointer on an allocated SbgVersion structure to fill.
 */
SBG_COMMON_LIB_API void sbgVersionDecode(uint32_t encodedVersion, SbgVersion *pVersionInfo);

/*!
 *	Construct a uint32_t containing a version information based on a SbgVersion structure.
 *	\param[in]	pVersionInfo											Pointer on a read only version structure to encode.
 *	\return																The encoded version information on an uint32_t or 0 if an error has occurred.
 */
SBG_COMMON_LIB_API uint32_t sbgVersionEncode(const SbgVersion *pVersionInfo);

//----------------------------------------------------------------------//
//- Version parameters methods                                         -//
//----------------------------------------------------------------------//

/*!
 *	Returns true if this encoded version number is using a software scheme.
 *	\param[in]	encodedVersion											The encoded version number to test stored in a uint32_t.
 *	\return																true if the version number is using a software scheme.
 */
SBG_INLINE bool sbgVersionIsUsingSoftwareScheme(uint32_t encodedVersion)
{
	SbgVersion	decodedVersion;

	//
	// Decode the version first
	//
	sbgVersionDecode(encodedVersion, &decodedVersion);

	//
	// Returns if the version is using a software scheme
	//
	return decodedVersion.softwareScheme;
}

//----------------------------------------------------------------------//
//- Version comparaison methods                                        -//
//----------------------------------------------------------------------//

/*!
 *	Compare two version information structures and return if the version A is greater, less or equal than the version B.
 *	The computation is roughly result = version A - version B
 *	We can define how far we will check if the version A is greater than the version B.
 *	For example, we can only check the major or major and minor fields.
 *	\param[in]	pVersionA					The first version to compare.
 *	\param[in]	pVersionB					The second version to compare.
 *	\param[in]	thresold					The comparaison thresold to know if we are checking the major, minor, revision, build, ...
 *	\return									A positive value if the version A is greater than B, a negative one if version A is less than B and 0 if the two versions are the same (according to the thresold).
 */
SBG_COMMON_LIB_API int32_t sbgVersionCompare(const SbgVersion *pVersionA, const SbgVersion *pVersionB, SbgVersionCmpThresold thresold);

/*!
 *	Compare two encoded versions and return if the version A is greater, less or equal than the version B.
 *	The computation is roughly result = version A - version B
 *	We can define how far we will check if the version A is greater than the version B.
 *	For example, we can only check the major or major and minor fields.
 *	\param[in]	versionA					The first compiled version to compare.
 *	\param[in]	versionB					The second compiled version to compare.
 *	\param[in]	thresold					The comparaison thresold to know if we are checking the major, minor, revision, build, ...
 *	\return									A positive value if the version A is greater than B, a negative one if version A is less than B and 0 if the two versions are the same (according to the thresold).
 */
SBG_COMMON_LIB_API int32_t sbgVersionCompareEncoded(uint32_t versionA, uint32_t versionB, SbgVersionCmpThresold thresold);

/*!
 * Check if the provided version is between the provided version interval.
 * All versions should have the same scheme.
 * 
 * \param[in]	pLowerVersion				The lower version bound of the interval.
 * \param[in]	pHigherVersion				The hiver version bound of the interval.
 * \param[in]	pVersion					The version to check.
 * \return									A negative value if the version is stricly below the lower version bound
 *											Zero if the version if equal or greater than lower version and equal or smaller than higer version
 *											A positive value if the version is strictly above the higher version bound
 */
SBG_COMMON_LIB_API int32_t sbgVersionIsWithinRange(const SbgVersion *pLowerVersion, const SbgVersion *pHigherVersion, const SbgVersion *pVersion);

/*!
 * Check if the provided encoded version is between the provided version interval.
 * All versions should have the same scheme.
 *
 * \param[in]	lowerVersion				The lower version bound of the interval.
 * \param[in]	higherVersion				The hiver version bound of the interval.
 * \param[in]	version						The version to check.
 * \return									A negative value if the version is stricly below the lower version bound
 *											Zero if the version if equal or greater than lower version and equal or smaller than higer version
 *											A positive value if the version is strictly above the higher version bound
 */
SBG_COMMON_LIB_API int32_t sbgVersionIsWithinRangeEncoded(uint32_t lowerVersion, uint32_t higherVersion, uint32_t version);


//----------------------------------------------------------------------//
//- Version to string methods                                          -//
//----------------------------------------------------------------------//

/*!
 *	Convert a version information to a human readable string.
 *	\param[in]	pVersionInfo											Pointer on a read only version structure to convert to a human readable string.
 *	\param[out]	pBuffer													Buffer to store the version as a human readable null terminated string.
 *	\param[in]	sizeOfBuffer											Maximum buffer size including the null terminated char.
 *	\return																SBG_NO_ERROR if the version information has been converted to a string.
 *																		SBG_BUFFER_OVERFLOW is the buffer isn't big enough to store the converted version string.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgVersionToString(const SbgVersion *pVersionInfo, char *pBuffer, uint32_t sizeOfBuffer);

/*!
 *	Convert an encoded version number to a human readable string.
 *	\param[in]	version													Encoded version value to to convert to a human readable string.
 *	\param[out]	pBuffer													Buffer to store the version as a human readable null terminated string.
 *	\param[in]	sizeOfBuffer											Maximum buffer size including the null terminated char.
 *	\return																SBG_NO_ERROR if the version information has been converted to a string.
 *																		SBG_BUFFER_OVERFLOW is the buffer isn't big enough to store the converted version string.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgVersionToStringEncoded(uint32_t version, char *pBuffer, uint32_t sizeOfBuffer);

//----------------------------------------------------------------------//
//- String to version methods                                          -//
//----------------------------------------------------------------------//

/*!
 *	Convert a human readable string to a version structure.
 *	\param[in]	pVersionStr												The string containing the version to convert.
 *	\param[out]	pVersionInfo											Pointer to a version structure to store the parsed version info.
 *	\return																SBG_NO_ERROR if the version information has been converted from a string.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgVersionFromString(const char *pVersionStr, SbgVersion *pVersionInfo);

/*!
 *	Convert an encoded version number to a human readable string.
 *	\param[in]	pVersionStr												The string containing the version to convert.
 *	\param[out]	pVersion												Returned encoded version value parsed from the string.
 *	\return																SBG_NO_ERROR if the version information has been converted from a string.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgVersionFromStringEncoded(const char *pVersionStr, uint32_t *pVersion);

//----------------------------------------------------------------------//
//- Legacy version system methods                                      -//
//----------------------------------------------------------------------//

/*!
 *  Define all these methods as deprecated.
 */
SBG_DEPRECATED(SBG_INLINE uint32_t SBG_VERSION(uint8_t major, uint8_t minor, uint8_t rev, uint8_t build));
SBG_DEPRECATED(SBG_INLINE uint8_t SBG_VERSION_GET_MAJOR(uint32_t encodedVersion));
SBG_DEPRECATED(SBG_INLINE uint8_t SBG_VERSION_GET_MINOR(uint32_t encodedVersion));
SBG_DEPRECATED(SBG_INLINE uint8_t SBG_VERSION_GET_REV(uint32_t encodedVersion));
SBG_DEPRECATED(SBG_INLINE uint8_t SBG_VERSION_GET_BUILD(uint32_t encodedVersion));

/*!
 *	DEPRECATED macro, please update your code
 *	Compile a version number using the basic scheme based on major, minor, revision and build information
 *	\param[in]	maj							The major version between 0 to 127.
 *	\param[in]	min							The minor version between 0 to 255.
 *	\param[in]	rev							The revision version between 0 to 255.
 *	\param[in]	build						The build number between 0 to 255.
 *	\return									The encoded version number.
 */
SBG_INLINE uint32_t SBG_VERSION(uint8_t major, uint8_t minor, uint8_t rev, uint8_t build)
{
    //
    // The SBG_VERSION macro is the basic version scheme.
    //
    return SBG_VERSION_BASIC(major, minor, rev, build);
}

/*!
 *	DEPRECATED method, please update your code.
 *	Extract a basic version scheme using legacy methods.
 *	\param[in]	encodedVersion					The encoded version to extract the major version.
 *	\return										The major version.
 */
SBG_INLINE uint8_t SBG_VERSION_GET_MAJOR(uint32_t encodedVersion)
{
	SbgVersion	versionInfo;

	//
	// Decode the version information
	//
	sbgVersionDecode(encodedVersion, &versionInfo);

	//
	// Return the major version
	//
	return versionInfo.major;
}

/*!
 *	DEPRECATED method, please update your code.
 *	Extract a basic version scheme using legacy methods.
 *	\param[in]	encodedVersion					The encoded version to extract the major version.
 *	\return										The major version.
 */
SBG_INLINE uint8_t SBG_VERSION_GET_MINOR(uint32_t encodedVersion)
{
	SbgVersion	versionInfo;

	//
	// Decode the version information
	//
	sbgVersionDecode(encodedVersion, &versionInfo);

	//
	// Return the major version
	//
	return versionInfo.minor;
}

/*!
 *	DEPRECATED method, please update your code.
 *	Extract a basic version scheme using legacy methods.
 *	\param[in]	encodedVersion					The encoded version to extract the major version.
 *	\return										The major version.
 */
SBG_INLINE uint8_t SBG_VERSION_GET_REV(uint32_t encodedVersion)
{
	SbgVersion	versionInfo;

	//
	// Decode the version information
	//
	sbgVersionDecode(encodedVersion, &versionInfo);

	//
	// Return the major version
	//
	return versionInfo.rev;
}

/*!
 *	DEPRECATED method, please update your code.
 *	Extract a basic version scheme using legacy methods.
 *	\param[in]	encodedVersion					The encoded version to extract the major version.
 *	\return										The major version.
 */
SBG_INLINE uint8_t SBG_VERSION_GET_BUILD(uint32_t encodedVersion)
{
	SbgVersion	versionInfo;

	//
	// Decode the version information
	//
	sbgVersionDecode(encodedVersion, &versionInfo);

	//
	// Return the major version
	//
	return (uint8_t)versionInfo.build;
}

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif	/* SBG_VERSION_H */
