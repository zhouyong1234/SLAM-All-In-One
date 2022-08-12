#include "sbgVersion.h"

//----------------------------------------------------------------------//
//- Version encoding / decoding methods                                -//
//----------------------------------------------------------------------//

/*!
 *	Fill a SbgVersion structure based on an uint32_t that stores the 'compiled' version information.
 *	\param[in]	encodedVersion											The version information stored within a uint32_t.
 *	\param[out]	pVersionInfo											Pointer on an allocated SbgVersion structure to fill.
 */
SBG_COMMON_LIB_API void sbgVersionDecode(uint32_t encodedVersion, SbgVersion *pVersionInfo)
{
	//
	// Check input parameters
	//
	assert(pVersionInfo);

	//
	// Test if we have a software version scheme
	//
	if (encodedVersion&SBG_VERSION_SOFT_SCHEME)
	{
		//
		// We have a software scheme, decode it
		//
		pVersionInfo->softwareScheme = true;

		//
		// Decode the software scheme fields
		//
		pVersionInfo->qualifier	= (SbgVersionQualifier)((encodedVersion >> SBG_VERSION_SOFT_SCHEME_QUALIFIER_SHIFT) & SBG_VERSION_SOFT_SCHEME_QUALIFIER_MASK);
		pVersionInfo->major		= (encodedVersion >> SBG_VERSION_SOFT_SCHEME_MAJOR_SHIFT) & SBG_VERSION_SOFT_SCHEME_MAJOR_MASK;
		pVersionInfo->minor		= (encodedVersion >> SBG_VERSION_SOFT_SCHEME_MINOR_SHIFT) & SBG_VERSION_SOFT_SCHEME_MINOR_MASK;
		pVersionInfo->build		= (encodedVersion >> SBG_VERSION_SOFT_SCHEME_BUILD_SHIFT) & SBG_VERSION_SOFT_SCHEME_BUILD_MASK;

		//
		// Set the revision to zero as it's not used
		//
		pVersionInfo->rev		= 0;
	}
	else
	{
		//
		// We have a basic scheme, decode it
		//
		pVersionInfo->softwareScheme = false;

		//
		// Decode the software scheme fields
		//
		pVersionInfo->major		= (encodedVersion >> 24) & 0xFF;
		pVersionInfo->minor		= (encodedVersion >> 16) & 0xFF;
		pVersionInfo->rev		= (encodedVersion >>  8) & 0xFF;
		pVersionInfo->build		= (encodedVersion >>  0) & 0xFF;

		//
		// Set the qualifier to zero
		//
		pVersionInfo->qualifier	= SBG_VERSION_QUALIFIER_DEV;
	}
}

/*!
 *	Construct a uint32_t containing a version information based on a SbgVersion structure.
 *	\param[in]	pVersionInfo											Pointer on a read only version structure to encode.
 *	\return																The encoded version information on an uint32_t or 0 if an error has occurred.
 */
SBG_COMMON_LIB_API uint32_t sbgVersionEncode(const SbgVersion *pVersionInfo)
{
	uint32_t	encodedVersion = 0;

	//
	// Check input parameter
	//
	assert(pVersionInfo);

	//
	// Test if we have a software scheme or a basic one
	//
	if (pVersionInfo->softwareScheme)
	{
		//
		// We have a software, scheme, so test that the version is valid
		//
		assert((pVersionInfo->major <= 63) && (pVersionInfo->minor <= 63) && (pVersionInfo->rev == 0));

		//
		// Indicate that we have a software version scheme
		//
		encodedVersion = SBG_VERSION_SOFTWARE(pVersionInfo->major, pVersionInfo->minor, pVersionInfo->build, pVersionInfo->qualifier);
	}
	else
	{
		//
		// We have a basic version scheme so check parameter validty
		//
		assert(pVersionInfo->major <= 127);

		//
		// Encode the basic version information
		//
		encodedVersion = SBG_VERSION_BASIC(pVersionInfo->major, pVersionInfo->minor, pVersionInfo->rev, pVersionInfo->build);
	}

	return encodedVersion;
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
SBG_COMMON_LIB_API int32_t sbgVersionCompare(const SbgVersion *pVersionA, const SbgVersion *pVersionB, SbgVersionCmpThresold thresold)
{
	int32_t	result;

	//
	// Check input parameter
	//
	assert((pVersionA) && (pVersionB));

	//
	// Check that the two versions are using the same scheme
	//
	assert(pVersionA->softwareScheme == pVersionB->softwareScheme);

	//
	// Do the comparaison according to the selected thresold
	// Start by compairing the major field
	//
	result = pVersionA->major - pVersionB->major;

	//
	// Test if we have to also compare the minor field
	//
	if ( (result == 0) &&  ((thresold == SBG_VERSION_CMP_THRESOLD_MINOR) || (thresold == SBG_VERSION_CMP_THRESOLD_REVISION) || (thresold == SBG_VERSION_CMP_THRESOLD_BUILD) || (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER)) )
	{
		//
		// Update the result using the minor indication
		//
		result = pVersionA->minor - pVersionB->minor;

		//
		// Test if we have to also compare the revision field (for basic version only)
		//
		if ( (result == 0) &&  ((thresold == SBG_VERSION_CMP_THRESOLD_REVISION) || (thresold == SBG_VERSION_CMP_THRESOLD_BUILD) || (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER)) )
		{
			//
			// Test if we have a software scheme or a basic scheme version
			//
			if (pVersionA->softwareScheme)
			{
				//
				// We have a software scheme so set the result to 0
				//
				result = 0;
			}
			else
			{
				//
				// We have a basic scheme so we can compare the revision field
				//
				result = pVersionA->rev - pVersionB->rev;
			}

			//
			// Test if we have to also compare the build field
			//
			if ( (result == 0) &&  ((thresold == SBG_VERSION_CMP_THRESOLD_BUILD) || (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER)) )
			{
				//
				// Compare the build field
				//
				result = pVersionA->build - pVersionB->build;

				//
				// Test if we have to also compare the qualifier field
				//
				if ( (result == 0) && (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER) )
				{
					//
					// Test if we have a software scheme
					//
					if (pVersionA->softwareScheme)
					{
						//
						// We have a software scheme so set the result to 0
						//
						result = pVersionA->qualifier - pVersionB->qualifier;
					}
					else
					{
						//
						// We have a basic scheme so set the result to 0
						//
						result = 0;
					}
				}
			}
		}
	}

	return result;
}

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
SBG_COMMON_LIB_API int32_t sbgVersionCompareEncoded(uint32_t versionA, uint32_t versionB, SbgVersionCmpThresold thresold)
{
	SbgVersion	versionAInfo;
	SbgVersion	versionBInfo;

	//
	// Decode the versions
	//
	sbgVersionDecode(versionA, &versionAInfo);
	sbgVersionDecode(versionB, &versionBInfo);

	//
	// Do the comparaison
	//
	return sbgVersionCompare(&versionAInfo, &versionBInfo, thresold);
}

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
SBG_COMMON_LIB_API int32_t sbgVersionIsWithinRange(const SbgVersion *pLowerVersion, const SbgVersion *pHigherVersion, const SbgVersion *pVersion)
{
	assert(pLowerVersion);
	assert(pHigherVersion);
	assert(pVersion);

	//
	// Use the encoded version to speed up the comparaison
	//
	return sbgVersionIsWithinRangeEncoded(sbgVersionEncode(pLowerVersion), sbgVersionEncode(pHigherVersion), sbgVersionEncode(pVersion));
}

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
SBG_COMMON_LIB_API int32_t sbgVersionIsWithinRangeEncoded(uint32_t lowerVersion, uint32_t higherVersion, uint32_t version)
{
	//
	// Make sure that all versions are using the same scheme
	//
	assert(lowerVersion <= higherVersion);
	assert((	(sbgVersionIsUsingSoftwareScheme(lowerVersion) == sbgVersionIsUsingSoftwareScheme(higherVersion)) &&
				(sbgVersionIsUsingSoftwareScheme(lowerVersion) == sbgVersionIsUsingSoftwareScheme(version))) ||
			(	(sbgVersionIsUsingSoftwareScheme(lowerVersion) != sbgVersionIsUsingSoftwareScheme(higherVersion)) &&
				(sbgVersionIsUsingSoftwareScheme(lowerVersion) != sbgVersionIsUsingSoftwareScheme(version))));
	
	//
	// We can compare safely the encoded version directly
	//
	if (version < lowerVersion)
	{
		return -1;
	}
	else if (version > higherVersion)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

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
SBG_COMMON_LIB_API SbgErrorCode sbgVersionToString(const SbgVersion *pVersionInfo, char *pBuffer, uint32_t sizeOfBuffer)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;

	//
	// Check input parameters
	//
	assert((pVersionInfo) && (pBuffer) && (sizeOfBuffer>0));

	//
	// Test if we have a basic or software version scheme
	//
	if (pVersionInfo->softwareScheme)
	{
		//
		// We have a software version scheme
		// Test that the buffer is big enough to store the generated string (31.31.65535-hotfix)
		//
		if (sizeOfBuffer >= 19)
		{
			//
			// Generate the string version
			//
			sprintf(pBuffer, "%u.%u.%u-", pVersionInfo->major, pVersionInfo->minor, pVersionInfo->build);

			//
			// Append the qualifier
			//
			switch (pVersionInfo->qualifier)
			{
			case SBG_VERSION_QUALIFIER_DEV:
				strcat(pBuffer, "dev");
				break;
			case SBG_VERSION_QUALIFIER_ALPHA:
				strcat(pBuffer, "alpha");
				break;
			case SBG_VERSION_QUALIFIER_BETA:
				strcat(pBuffer, "beta");
				break;
			case SBG_VERSION_QUALIFIER_RC:
				strcat(pBuffer, "rc");
				break;
			case SBG_VERSION_QUALIFIER_STABLE:
				strcat(pBuffer, "stable");
				break;
			case SBG_VERSION_QUALIFIER_HOT_FIX:
				strcat(pBuffer, "hotfix");
				break;
			default:
				break;
			}
		}
		else
		{
			//
			// Output buffer is to small
			//
			errorCode = SBG_BUFFER_OVERFLOW;
		}

	}
	else
	{
		//
		// We have a basic version scheme, generate the string
		// Test that the buffer is big enough to store the generated string
		//
		if (sizeOfBuffer >= 16)
		{
			//
			// Generate the string version
			//
			sprintf(pBuffer, "%u.%u.%u.%u", pVersionInfo->major, pVersionInfo->minor, pVersionInfo->rev, pVersionInfo->build);
		}
		else
		{
			//
			// Output buffer is to small
			//
			errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// Return status of operation
	//
	return errorCode;
}

/*!
 *	Convert an encoded version number to a human readable string.
 *	\param[in]	version													Encoded version value to to convert to a human readable string.
 *	\param[out]	pBuffer													Buffer to store the version as a human readable null terminated string.
 *	\param[in]	sizeOfBuffer											Maximum buffer size including the null terminated char.
 *	\return																SBG_NO_ERROR if the version information has been converted to a string.
 *																		SBG_BUFFER_OVERFLOW is the buffer isn't big enough to store the converted version string.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgVersionToStringEncoded(uint32_t version, char *pBuffer, uint32_t sizeOfBuffer)
{
	SbgVersion	versionInfo;

	//
	// Decode the versions
	//
	sbgVersionDecode(version, &versionInfo);

	//
	// Return the version as a string
	//
	return sbgVersionToString(&versionInfo, pBuffer, sizeOfBuffer);
}

//----------------------------------------------------------------------//
//- String to version methods                                          -//
//----------------------------------------------------------------------//

/*!
 *	Convert a human readable string to a version structure.
 *	\param[in]	pVersionStr												The string containing the version to convert.
 *	\param[out]	pVersionInfo											Pointer to a version structure to store the parsed version info.
 *	\return																SBG_NO_ERROR if the version information has been converted from a string.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgVersionFromString(const char *pVersionStr, SbgVersion *pVersionInfo)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	char			qualifierStr[32];
	uint32_t			major;
	uint32_t			minor;
	uint32_t			rev;
	uint32_t			build;

	//
	// Check input parameters
	//
	assert((pVersionStr) && (pVersionInfo));

	//
	// Zero init the returned version struct
	//
	memset(pVersionInfo, 0x00, sizeof(SbgVersion));

	//
	// Try to parse a basic version
	//
	if (sscanf(pVersionStr, "%u.%u.%u.%u", &major, &minor, &rev, &build) == 4)
	{
		//
		// We have read successfully a basic version
		//
		pVersionInfo->softwareScheme = false;

		//
		// Fill the version numbers
		//
		pVersionInfo->major	= (uint8_t)major;
		pVersionInfo->minor	= (uint8_t)minor;
		pVersionInfo->rev	= (uint8_t)rev;
		pVersionInfo->build	= (uint8_t)build;
	}
	else if (sscanf(pVersionStr, "%u.%u.%u-%s", &major, &minor, &build, qualifierStr) == 4)
	{
		//
		// We have read successfully a software scheme version
		//
		pVersionInfo->softwareScheme = true;

		//
		// Fill the version numbers
		//
		pVersionInfo->major	= (uint8_t)major;
		pVersionInfo->minor	= (uint8_t)minor;
		pVersionInfo->build	= (uint16_t)build;

		//
		// Also convert the qualifier
		//
		if (!strcmp(qualifierStr, "dev"))
		{
			//
			// Dev qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_DEV;
		}
		else if (!strcmp(qualifierStr, "alpha"))
		{
			//
			// Alpha qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_ALPHA;
		}
		else if (!strcmp(qualifierStr, "beta"))
		{
			//
			// Beta qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_BETA;
		}
		else if (!strcmp(qualifierStr, "rc"))
		{
			//
			// Release Candidate qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_RC;
		}
		else if (!strcmp(qualifierStr, "stable"))
		{
			//
			// Stable qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_STABLE;
		}
		else if (!strcmp(qualifierStr, "hotfix"))
		{
			//
			// Hot Fix qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_HOT_FIX;
		}
		else
		{
			//
			// Unknown qualifier
			//
			errorCode = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		//
		// Invalid format
		//
		errorCode = SBG_INVALID_PARAMETER;
	}
	
	return errorCode;
}

/*!
 *	Convert an encoded version number to a human readable string.
 *	\param[in]	pVersionStr												The string containing the version to convert.
 *	\param[out]	pVersion												Returned encoded version value parsed from the string.
 *	\return																SBG_NO_ERROR if the version information has been converted from a string.
 */
SBG_COMMON_LIB_API SbgErrorCode sbgVersionFromStringEncoded(const char *pVersionStr, uint32_t *pVersion)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	SbgVersion		versionInfo;

	//
	// Check input parameters
	//
	assert((pVersionStr) && (pVersion));

	//
	// Parse the version from a string
	//
	errorCode = sbgVersionFromString(pVersionStr, &versionInfo);

	//
	// Test that no error has occurred
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Encode the version and return it
		//
		*pVersion = sbgVersionEncode(&versionInfo);
	}
	else
	{
		//
		// An error has occurred so return a zero version
		//
		*pVersion = 0;
	}

	//
	// Return error
	//
	return errorCode;
}
