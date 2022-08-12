#include <sbgCommon.h>
#include <stdarg.h>
#include <time.h>

//----------------------------------------------------------------------//
//- Include specific header for WIN32 and UNIX platforms               -//
//----------------------------------------------------------------------//
#ifdef WIN32
	#include <windows.h>
#elif defined(__APPLE__)
	#include <mach/mach_time.h>
#else
	#include <unistd.h>
#endif

//----------------------------------------------------------------------//
//- Specific timing methods to reimplement for your platform           -//
//----------------------------------------------------------------------//

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
SBG_COMMON_LIB_API uint32_t sbgGetTime(void)
{
#ifdef WIN32
	//
	// Return the current time in ms
	//
	return clock() / (CLOCKS_PER_SEC / 1000);
#elif defined(__APPLE__)
	mach_timebase_info_data_t	timeInfo;
	mach_timebase_info(&timeInfo);

	//
	// Return the current time in ms
	//
	return (mach_absolute_time() * timeInfo.numer / timeInfo.denom) / 1000000.0;
#else
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);

	//
	// Return the current time in ms
	//
	return now.tv_sec * 1000 + now.tv_nsec / 1000000;
#endif
}

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
SBG_COMMON_LIB_API void sbgSleep(uint32_t ms)
{
	//
	// Implementation valid for both WIN and UNIX systems
	//
	#ifdef WIN32
		Sleep(ms);
	#else
		usleep(ms*1000);
	#endif
}

//----------------------------------------------------------------------//
//- Specific logging methods to reimplement for your platform          -//
//----------------------------------------------------------------------//

/*!
 *	The method is called when one of the SBG_LOG_ERROR, SBG_LOG_WARNING, SBG_LOG_INFO or SBG_LOG_VERBOSE is called.
 *	It logs an error message with debug information and support a variable list of arguments
 *	\param[in]	pFileName					File name where the error occurred.
 *	\param[in]	pFunctionName				Function name where the error occurred.
 *	\param[in]	line						Line number where the error occurred.
 *	\param[in]	logType						Define if we have an error, a warning, an info or a verbose log.
 *	\param[in]	errorCode					The error code associated with the message.
 *	\param[in]	pFormat						The error message that will be used with the variable list of arguments.
 */
SBG_COMMON_LIB_API void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...)
{
	char		errorMsg[SBG_CONFIG_LOG_MAX_SIZE];
	va_list		args;
	
	//
	// Initialize the list of variable arguments on the latest function argument
	//
	va_start(args, pFormat);

	//
	// Generate the error message string
	//
	vsprintf(errorMsg, pFormat, args);

	//
	// Close the list of variable arguments
	//
	va_end(args);

	//
	// Log the correct message according to the log type
	//
	switch (logType)
	{
	case SBG_DEBUG_LOG_TYPE_ERROR:
		fprintf(stderr, "*ERR * [%s]%s: %s\n\r", sbgErrorCodeToString(errorCode), pFunctionName, errorMsg);
		break;
	case SBG_DEBUG_LOG_TYPE_WARNING:
		fprintf(stderr, "*WARN* [%s]%s: %s\n\r", sbgErrorCodeToString(errorCode), pFunctionName, errorMsg);
		break;
	case SBG_DEBUG_LOG_TYPE_INFO:
		fprintf(stderr, "*INFO* %s\n\r", errorMsg);
		break;
	case SBG_DEBUG_LOG_TYPE_DEBUG:
		fprintf(stderr, "*DBG * %s\n\r", errorMsg);
		break;
	default:
		fprintf(stderr, "*UKNW*\t[%s]%s(%u): %s\n\r", sbgErrorCodeToString(errorCode), pFunctionName, line, errorMsg);
	}
}
