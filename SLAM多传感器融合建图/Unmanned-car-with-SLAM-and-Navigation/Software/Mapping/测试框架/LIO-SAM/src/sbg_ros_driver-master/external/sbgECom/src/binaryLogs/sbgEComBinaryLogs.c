#include "sbgEComBinaryLogs.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Communication protocol operations                                  -//
//----------------------------------------------------------------------//

/*!
 *	Parse an incoming log and fill the output union.
 *	\param[in]	msgClass					Received message class
 *	\param[in]	msg							Received message ID
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output union that stores parsed data.
 */
SbgErrorCode sbgEComBinaryLogParse(SbgEComClass msgClass, SbgEComMsgId msg, const void *pPayload, size_t payloadSize, SbgBinaryLogData *pOutputData)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		inputStream;

	assert(pPayload);
	assert(payloadSize > 0);
	assert(pOutputData);

	//
	// Handle the different classes of messages differently
	//
	if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
	{
		//
		// Create an input stream buffer that points to the frame payload so we can easily parse it's content
		//
		sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

		//
		// Parse the incoming log according to its type
		//
		switch (msg)
		{
		case SBG_ECOM_LOG_STATUS:
			errorCode = sbgEComBinaryLogParseStatusData(&inputStream, &pOutputData->statusData);
			break;
		case SBG_ECOM_LOG_IMU_DATA:
			errorCode = sbgEComBinaryLogParseImuData(&inputStream, &pOutputData->imuData);
			break;
		case SBG_ECOM_LOG_IMU_SHORT:
			errorCode = sbgEComBinaryLogParseImuShort(&inputStream, &pOutputData->imuShort);
			break;
		case SBG_ECOM_LOG_EKF_EULER:
			errorCode = sbgEComBinaryLogParseEkfEulerData(&inputStream, &pOutputData->ekfEulerData);
			break;
		case SBG_ECOM_LOG_EKF_QUAT:
			errorCode = sbgEComBinaryLogParseEkfQuatData(&inputStream, &pOutputData->ekfQuatData);
			break;
		case SBG_ECOM_LOG_EKF_NAV:
			errorCode = sbgEComBinaryLogParseEkfNavData(&inputStream, &pOutputData->ekfNavData);
			break;
		case SBG_ECOM_LOG_SHIP_MOTION:
		case SBG_ECOM_LOG_SHIP_MOTION_HP:
			errorCode = sbgEComBinaryLogParseShipMotionData(&inputStream, &pOutputData->shipMotionData);
			break;
		case SBG_ECOM_LOG_ODO_VEL:
			errorCode = sbgEComBinaryLogParseOdometerData(&inputStream, &pOutputData->odometerData);
			break;
		case SBG_ECOM_LOG_UTC_TIME:
			errorCode = sbgEComBinaryLogParseUtcData(&inputStream, &pOutputData->utcData);
			break;
		case SBG_ECOM_LOG_GPS1_VEL:
		case SBG_ECOM_LOG_GPS2_VEL:
			errorCode = sbgEComBinaryLogParseGpsVelData(&inputStream, &pOutputData->gpsVelData);
			break;
		case SBG_ECOM_LOG_GPS1_POS:
		case SBG_ECOM_LOG_GPS2_POS:
			errorCode = sbgEComBinaryLogParseGpsPosData(&inputStream, &pOutputData->gpsPosData);
			break;
		case SBG_ECOM_LOG_GPS1_HDT:
		case SBG_ECOM_LOG_GPS2_HDT:
			errorCode = sbgEComBinaryLogParseGpsHdtData(&inputStream, &pOutputData->gpsHdtData);
			break;
		case SBG_ECOM_LOG_GPS1_RAW:
		case SBG_ECOM_LOG_GPS2_RAW:
			errorCode = sbgEComBinaryLogParseGpsRawData(&inputStream, &pOutputData->gpsRawData);
			break;
		case SBG_ECOM_LOG_MAG:
			errorCode = sbgEComBinaryLogParseMagData(&inputStream, &pOutputData->magData);
			break;
		case SBG_ECOM_LOG_MAG_CALIB:
			errorCode = sbgEComBinaryLogParseMagCalibData(&inputStream, &pOutputData->magCalibData);
			break;
		case SBG_ECOM_LOG_DVL_BOTTOM_TRACK:
			errorCode = sbgEComBinaryLogParseDvlData(&inputStream, &pOutputData->dvlData);
			break;
		case SBG_ECOM_LOG_DVL_WATER_TRACK:
			errorCode = sbgEComBinaryLogParseDvlData(&inputStream, &pOutputData->dvlData);
			break;
		case SBG_ECOM_LOG_AIR_DATA:
			errorCode = sbgEComBinaryLogParseAirData(&inputStream, &pOutputData->airData);
			break;
		case SBG_ECOM_LOG_USBL:
			errorCode = sbgEComBinaryLogParseUsblData(&inputStream, &pOutputData->usblData);
			break;
		case SBG_ECOM_LOG_DEPTH:
			errorCode = sbgEComBinaryLogParseDepth(&inputStream, &pOutputData->depthData);
			break;
		case SBG_ECOM_LOG_EVENT_A:
		case SBG_ECOM_LOG_EVENT_B:
		case SBG_ECOM_LOG_EVENT_C:
		case SBG_ECOM_LOG_EVENT_D:
		case SBG_ECOM_LOG_EVENT_E:
		case SBG_ECOM_LOG_EVENT_OUT_A:
		case SBG_ECOM_LOG_EVENT_OUT_B:
			errorCode = sbgEComBinaryLogParseEvent(&inputStream, &pOutputData->eventMarker);
			break;
		case SBG_ECOM_LOG_DEBUG_0:
		case SBG_ECOM_LOG_DEBUG_1:
		case SBG_ECOM_LOG_DEBUG_2:
		case SBG_ECOM_LOG_DEBUG_3:
			errorCode = sbgEComBinaryLogParseDebugData(&inputStream, &pOutputData->debugData);
			break;
		case SBG_ECOM_LOG_IMU_RAW_DATA:
			errorCode = sbgEComBinaryLogParseImuRawData(&inputStream, &pOutputData->imuRawData);
			break;
		case SBG_ECOM_LOG_DIAG:
			errorCode = sbgEComBinaryLogParseDiagData(&inputStream, &pOutputData->diagData);
			break;
		default:
			//
			// This log isn't handled
			//
			errorCode = SBG_ERROR;
		}
	}
	else if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_1)
	{
		//
		// Create an input stream buffer that points to the frame payload so we can easily parse it's content
		//
		sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

		//
		// Parse the message depending on the message ID
		//
		switch ((SbgEComLog1)msg)
		{
		case SBG_ECOM_LOG_FAST_IMU_DATA:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseFastImuData(&inputStream, &pOutputData->fastImuData);
			break;
		default:
			//
			// This log isn't handled
			//
			errorCode = SBG_ERROR;
		}
	}
	else
	{
		//
		// Un-handled message class
		//
		errorCode = SBG_ERROR;
	}

	return errorCode;
}
