// File header
#include "message_wrapper.h"

// Project headers
#include <sbg_vector3.h>

using sbg::MessageWrapper;

/*!
 * Class to wrap the SBG logs into ROS messages.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

MessageWrapper::MessageWrapper(void):
m_first_valid_utc_(false)
{
}

//---------------------------------------------------------------------//
//- Internal methods                                                  -//
//---------------------------------------------------------------------//

float MessageWrapper::wrapAngle2Pi(float angle_rad) const
{
  if ((angle_rad < -SBG_PI_F * 2.0f) || (angle_rad > SBG_PI_F * 2.0f))
  {
    angle_rad = fmodf(angle_rad, SBG_PI_F * 2.0f);
  }

  if (angle_rad < 0.0f)
  {
    angle_rad = SBG_PI_F * 2.0f + angle_rad;
  }

  return angle_rad;
}

float MessageWrapper::wrapAngle360(float angle_deg) const
{
  if ( (angle_deg < -360.0f) || (angle_deg > 360.0f) )
  {
    angle_deg = fmodf(angle_deg, 360.0f);
  }

  if (angle_deg < 0.0f)
  {
    angle_deg = 360.0f + angle_deg;
  }

  return angle_deg;
}

const std_msgs::Header MessageWrapper::createRosHeader(uint32_t device_timestamp) const
{
  std_msgs::Header header;

  header.frame_id = m_frame_id_;

  if (m_first_valid_utc_ && (m_time_reference_ == sbg::TimeReference::INS_UNIX))
  {
    header.stamp = convertInsTimeToUnix(device_timestamp);
  }
  else
  {
    header.stamp = ros::Time::now();
  }

  return header;
}

const ros::Time MessageWrapper::convertInsTimeToUnix(uint32_t device_timestamp) const
{
  //
  // Convert the UTC time to epoch from the last received message.
  // Add the SBG timestamp difference (timestamp is in microsecond).
  //
  ros::Time utc_to_epoch;
  uint32_t  device_timestamp_diff;
  uint64_t  nanoseconds;

  utc_to_epoch          = convertUtcToUnix(m_last_sbg_utc_);
  device_timestamp_diff = device_timestamp - m_last_sbg_utc_.time_stamp;

  nanoseconds = utc_to_epoch.toNSec() + static_cast<uint64_t>(device_timestamp_diff) * 1000;

  utc_to_epoch.fromNSec(nanoseconds);

  return utc_to_epoch;
}

const sbg_driver::SbgEkfStatus MessageWrapper::createEkfStatusMessage(uint32_t ekf_status) const
{
  sbg_driver::SbgEkfStatus ekf_status_message;

  ekf_status_message.solution_mode    = sbgEComLogEkfGetSolutionMode(ekf_status);
  ekf_status_message.attitude_valid   = (ekf_status & SBG_ECOM_SOL_ATTITUDE_VALID) != 0;
  ekf_status_message.heading_valid    = (ekf_status & SBG_ECOM_SOL_HEADING_VALID) != 0;
  ekf_status_message.velocity_valid   = (ekf_status & SBG_ECOM_SOL_VELOCITY_VALID) != 0;
  ekf_status_message.position_valid   = (ekf_status & SBG_ECOM_SOL_POSITION_VALID) != 0;

  ekf_status_message.vert_ref_used    = (ekf_status & SBG_ECOM_SOL_VERT_REF_USED) != 0;
  ekf_status_message.mag_ref_used     = (ekf_status & SBG_ECOM_SOL_MAG_REF_USED) != 0;

  ekf_status_message.gps1_vel_used    = (ekf_status & SBG_ECOM_SOL_GPS1_VEL_USED) != 0;
  ekf_status_message.gps1_pos_used    = (ekf_status & SBG_ECOM_SOL_GPS1_POS_USED) != 0;
  ekf_status_message.gps1_course_used = (ekf_status & SBG_ECOM_SOL_GPS1_HDT_USED) != 0;
  ekf_status_message.gps1_hdt_used    = (ekf_status & SBG_ECOM_SOL_GPS1_HDT_USED) != 0;

  ekf_status_message.gps2_vel_used    = (ekf_status & SBG_ECOM_SOL_GPS2_VEL_USED) != 0;
  ekf_status_message.gps2_pos_used    = (ekf_status & SBG_ECOM_SOL_GPS2_POS_USED) != 0;
  ekf_status_message.gps2_course_used = (ekf_status & SBG_ECOM_SOL_GPS2_POS_USED) != 0;
  ekf_status_message.gps2_hdt_used    = (ekf_status & SBG_ECOM_SOL_GPS2_HDT_USED) != 0;

  ekf_status_message.odo_used         = (ekf_status & SBG_ECOM_SOL_ODO_USED) != 0;

  return ekf_status_message;
}

const sbg_driver::SbgGpsPosStatus MessageWrapper::createGpsPosStatusMessage(const SbgLogGpsPos& ref_log_gps_pos) const
{
  sbg_driver::SbgGpsPosStatus gps_pos_status_message;

  gps_pos_status_message.status       = sbgEComLogGpsPosGetStatus(ref_log_gps_pos.status);
  gps_pos_status_message.type         = sbgEComLogGpsPosGetType(ref_log_gps_pos.status);

  gps_pos_status_message.gps_l1_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GPS_L1_USED) != 0;
  gps_pos_status_message.gps_l2_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GPS_L2_USED) != 0;
  gps_pos_status_message.gps_l5_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GPS_L5_USED) != 0;

  gps_pos_status_message.glo_l1_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GLO_L1_USED) != 0;
  gps_pos_status_message.glo_l2_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GLO_L2_USED) != 0;

  return gps_pos_status_message;
}

const sbg_driver::SbgGpsVelStatus MessageWrapper::createGpsVelStatusMessage(const SbgLogGpsVel& ref_log_gps_vel) const
{
  sbg_driver::SbgGpsVelStatus gps_vel_status_message;

  gps_vel_status_message.vel_status = sbgEComLogGpsVelGetStatus(ref_log_gps_vel.status);
  gps_vel_status_message.vel_type   = sbgEComLogGpsVelGetType(ref_log_gps_vel.status);

  return gps_vel_status_message;
}

const sbg_driver::SbgImuStatus MessageWrapper::createImuStatusMessage(uint16_t sbg_imu_status) const
{
  sbg_driver::SbgImuStatus imu_status_message;

  imu_status_message.imu_com              = (sbg_imu_status & SBG_ECOM_IMU_COM_OK) != 0;
  imu_status_message.imu_status           = (sbg_imu_status & SBG_ECOM_IMU_STATUS_BIT) != 0 ;
  imu_status_message.imu_accels_in_range  = (sbg_imu_status & SBG_ECOM_IMU_ACCELS_IN_RANGE) != 0;
  imu_status_message.imu_gyros_in_range   = (sbg_imu_status & SBG_ECOM_IMU_GYROS_IN_RANGE) != 0;

  imu_status_message.imu_accel_x = (sbg_imu_status & SBG_ECOM_IMU_ACCEL_X_BIT) != 0;
  imu_status_message.imu_accel_y = (sbg_imu_status & SBG_ECOM_IMU_ACCEL_Y_BIT) != 0;
  imu_status_message.imu_accel_z = (sbg_imu_status & SBG_ECOM_IMU_ACCEL_Z_BIT) != 0;

  imu_status_message.imu_gyro_x = (sbg_imu_status & SBG_ECOM_IMU_GYRO_X_BIT) != 0;
  imu_status_message.imu_gyro_y = (sbg_imu_status & SBG_ECOM_IMU_GYRO_Y_BIT) != 0;
  imu_status_message.imu_gyro_z = (sbg_imu_status & SBG_ECOM_IMU_GYRO_Z_BIT) != 0;

  return imu_status_message;
}

const sbg_driver::SbgMagStatus MessageWrapper::createMagStatusMessage(const SbgLogMag& ref_log_mag) const
{
  sbg_driver::SbgMagStatus mag_status_message;

  mag_status_message.mag_x            = (ref_log_mag.status & SBG_ECOM_MAG_MAG_X_BIT) != 0;
  mag_status_message.mag_y            = (ref_log_mag.status & SBG_ECOM_MAG_MAG_Y_BIT) != 0;
  mag_status_message.mag_z            = (ref_log_mag.status & SBG_ECOM_MAG_MAG_Z_BIT) != 0;

  mag_status_message.accel_x          = (ref_log_mag.status & SBG_ECOM_MAG_ACCEL_X_BIT) != 0;
  mag_status_message.accel_y          = (ref_log_mag.status & SBG_ECOM_MAG_ACCEL_Y_BIT) != 0;
  mag_status_message.accel_z          = (ref_log_mag.status & SBG_ECOM_MAG_ACCEL_Z_BIT) != 0;

  mag_status_message.mags_in_range    = (ref_log_mag.status & SBG_ECOM_MAG_MAGS_IN_RANGE) != 0;
  mag_status_message.accels_in_range  = (ref_log_mag.status & SBG_ECOM_MAG_ACCELS_IN_RANGE) != 0;
  mag_status_message.calibration      = (ref_log_mag.status & SBG_ECOM_MAG_CALIBRATION_OK) != 0;

  return mag_status_message;
}

const sbg_driver::SbgShipMotionStatus MessageWrapper::createShipMotionStatusMessage(const SbgLogShipMotionData& ref_log_ship_motion) const
{
  sbg_driver::SbgShipMotionStatus ship_motion_status_message;

  ship_motion_status_message.heave_valid      = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_VALID) != 0;
  ship_motion_status_message.heave_vel_aided  = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_VEL_AIDED) != 0;
  ship_motion_status_message.period_available = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_PERIOD_INCLUDED) != 0;
  ship_motion_status_message.period_valid     = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_PERIOD_VALID) != 0;

  return ship_motion_status_message;
}

const sbg_driver::SbgStatusAiding MessageWrapper::createStatusAidingMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatusAiding status_aiding_message;

  status_aiding_message.gps1_pos_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_POS_RECV) != 0;
  status_aiding_message.gps1_vel_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_VEL_RECV) != 0;
  status_aiding_message.gps1_hdt_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_HDT_RECV) != 0;
  status_aiding_message.gps1_utc_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_UTC_RECV) != 0;

  status_aiding_message.mag_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_MAG_RECV) != 0;
  status_aiding_message.odo_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_ODO_RECV) != 0;
  status_aiding_message.dvl_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_DVL_RECV) != 0;

  return status_aiding_message;
}

const sbg_driver::SbgStatusCom MessageWrapper::createStatusComMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatusCom status_com_message;

  status_com_message.port_a = (ref_log_status.comStatus & SBG_ECOM_PORTA_VALID) != 0;
  status_com_message.port_b = (ref_log_status.comStatus & SBG_ECOM_PORTB_VALID) != 0;
  status_com_message.port_c = (ref_log_status.comStatus & SBG_ECOM_PORTC_VALID) != 0;
  status_com_message.port_d = (ref_log_status.comStatus & SBG_ECOM_PORTD_VALID) != 0;
  status_com_message.port_e = (ref_log_status.comStatus & SBG_ECOM_PORTE_VALID) != 0;

  status_com_message.port_a_rx = (ref_log_status.comStatus & SBG_ECOM_PORTA_RX_OK) != 0;
  status_com_message.port_a_tx = (ref_log_status.comStatus & SBG_ECOM_PORTA_TX_OK) != 0;
  status_com_message.port_b_rx = (ref_log_status.comStatus & SBG_ECOM_PORTB_RX_OK) != 0;
  status_com_message.port_b_tx = (ref_log_status.comStatus & SBG_ECOM_PORTB_TX_OK) != 0;
  status_com_message.port_c_rx = (ref_log_status.comStatus & SBG_ECOM_PORTC_RX_OK) != 0;
  status_com_message.port_c_tx = (ref_log_status.comStatus & SBG_ECOM_PORTC_TX_OK) != 0;
  status_com_message.port_d_rx = (ref_log_status.comStatus & SBG_ECOM_PORTD_RX_OK) != 0;
  status_com_message.port_d_tx = (ref_log_status.comStatus & SBG_ECOM_PORTD_TX_OK) != 0;
  status_com_message.port_e_rx = (ref_log_status.comStatus & SBG_ECOM_PORTE_RX_OK) != 0;
  status_com_message.port_e_tx = (ref_log_status.comStatus & SBG_ECOM_PORTE_TX_OK) != 0;

  status_com_message.can_rx     = (ref_log_status.comStatus & SBG_ECOM_CAN_RX_OK) != 0;
  status_com_message.can_tx     = (ref_log_status.comStatus & SBG_ECOM_CAN_TX_OK) != 0;
  status_com_message.can_status = (ref_log_status.comStatus & SBG_ECOM_CAN_VALID) != 0;

  return status_com_message;
}

const sbg_driver::SbgStatusGeneral MessageWrapper::createStatusGeneralMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatusGeneral status_general_message;

  status_general_message.main_power   = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_MAIN_POWER_OK) != 0;
  status_general_message.imu_power    = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_IMU_POWER_OK) != 0;
  status_general_message.gps_power    = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_GPS_POWER_OK) != 0;
  status_general_message.settings     = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_SETTINGS_OK) != 0;
  status_general_message.temperature  = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_TEMPERATURE_OK) != 0;

  return status_general_message;
}

const sbg_driver::SbgUtcTimeStatus MessageWrapper::createUtcStatusMessage(const SbgLogUtcData& ref_log_utc) const
{
  sbg_driver::SbgUtcTimeStatus utc_status_message;

  utc_status_message.clock_stable     = (ref_log_utc.status & SBG_ECOM_CLOCK_STABLE_INPUT) != 0;
  utc_status_message.clock_utc_sync   = (ref_log_utc.status & SBG_ECOM_CLOCK_UTC_SYNC) != 0;

  utc_status_message.clock_status     = static_cast<uint8_t>(sbgEComLogUtcGetClockStatus(ref_log_utc.status));
  utc_status_message.clock_utc_status = static_cast<uint8_t>(sbgEComLogUtcGetClockUtcStatus(ref_log_utc.status));

  return utc_status_message;
}

uint32_t MessageWrapper::getNumberOfDaysInYear(uint16_t year) const
{
  if (isLeapYear(year))
  {
    return 366;
  }
  else
  {
    return 365;
  }
}

uint32_t MessageWrapper::getNumberOfDaysInMonth(uint16_t year, uint8_t month_index) const
{
  if ((month_index == 4) || (month_index == 6) || (month_index == 9) || (month_index == 11))
  {
    return 30;
  }
  else if ((month_index == 2))
  {
    if (isLeapYear(year))
    {
      return 29;
    }
    else
    {
      return 28;
    }
  }
  else
  {
    return 31;
  }
}

bool MessageWrapper::isLeapYear(uint16_t year) const
{
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

const ros::Time MessageWrapper::convertUtcToUnix(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const
{
  ros::Time utc_to_epoch;
  uint32_t  days;
  uint64_t  nanoseconds;

  //
  // Convert the UTC time to Epoch(Unix) time, which is the elapsed seconds since 1 Jan 1970.
  //
  days        = 0;
  nanoseconds = 0;

  for (uint16_t yearIndex = 1970; yearIndex < ref_sbg_utc_msg.year; yearIndex++)
  {
    days += getNumberOfDaysInYear(yearIndex);
  }

  for (uint8_t monthIndex = 1; monthIndex < ref_sbg_utc_msg.month; monthIndex++)
  {
    days += getNumberOfDaysInMonth(ref_sbg_utc_msg.year, monthIndex);
  }

  days += ref_sbg_utc_msg.day - 1;

  nanoseconds = days * 24;
  nanoseconds = (nanoseconds + ref_sbg_utc_msg.hour) * 60;
  nanoseconds = (nanoseconds + ref_sbg_utc_msg.min) * 60;
  nanoseconds = nanoseconds + ref_sbg_utc_msg.sec;
  nanoseconds = nanoseconds * 1000000000 + ref_sbg_utc_msg.nanosec;

  utc_to_epoch.fromNSec(nanoseconds);

  return utc_to_epoch;
}

const sbg_driver::SbgAirDataStatus MessageWrapper::createAirDataStatusMessage(const SbgLogAirData& ref_sbg_air_data) const
{
  sbg_driver::SbgAirDataStatus air_data_status_message;

  air_data_status_message.is_delay_time         = (ref_sbg_air_data.status & SBG_ECOM_AIR_DATA_TIME_IS_DELAY) != 0;
  air_data_status_message.pressure_valid        = (ref_sbg_air_data.status & SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID) != 0;
  air_data_status_message.altitude_valid        = (ref_sbg_air_data.status & SBG_ECOM_AIR_DATA_ALTITUDE_VALID) != 0;
  air_data_status_message.pressure_diff_valid   = (ref_sbg_air_data.status & SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID) != 0;
  air_data_status_message.air_speed_valid       = (ref_sbg_air_data.status & SBG_ECOM_AIR_DATA_AIRPSEED_VALID) != 0;
  air_data_status_message.air_temperature_valid = (ref_sbg_air_data.status & SBG_ECOM_AIR_DATA_TEMPERATURE_VALID) != 0;

  return air_data_status_message;
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

void MessageWrapper::setTimeReference(TimeReference time_reference)
{
  m_time_reference_ = time_reference;
}

void MessageWrapper::setFrameId(const std::string &frame_id)
{
  m_frame_id_ = frame_id;
}

void MessageWrapper::setUseEnu(bool enu)
{
  m_use_enu_ = enu;
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

const sbg_driver::SbgEkfEuler MessageWrapper::createSbgEkfEulerMessage(const SbgLogEkfEulerData& ref_log_ekf_euler) const
{
  sbg_driver::SbgEkfEuler ekf_euler_message;

  ekf_euler_message.header      = createRosHeader(ref_log_ekf_euler.timeStamp);
  ekf_euler_message.time_stamp  = ref_log_ekf_euler.timeStamp;
  ekf_euler_message.status      = createEkfStatusMessage(ref_log_ekf_euler.status);

  if (m_use_enu_)
  {
    ekf_euler_message.angle.x  = ref_log_ekf_euler.euler[0];
    ekf_euler_message.angle.y  = -ref_log_ekf_euler.euler[1];
    ekf_euler_message.angle.z  = wrapAngle2Pi((SBG_PI_F / 2.0f) - ref_log_ekf_euler.euler[2]);
  }
  else
  {
    ekf_euler_message.angle.x = ref_log_ekf_euler.euler[0];
    ekf_euler_message.angle.y = ref_log_ekf_euler.euler[1];
    ekf_euler_message.angle.z = ref_log_ekf_euler.euler[2];
  }

  ekf_euler_message.accuracy.x  = ref_log_ekf_euler.eulerStdDev[0];
  ekf_euler_message.accuracy.y  = ref_log_ekf_euler.eulerStdDev[1];
  ekf_euler_message.accuracy.z  = ref_log_ekf_euler.eulerStdDev[2];

  return ekf_euler_message;
}

const sbg_driver::SbgEkfNav MessageWrapper::createSbgEkfNavMessage(const SbgLogEkfNavData& ref_log_ekf_nav) const
{
  sbg_driver::SbgEkfNav ekf_nav_message;

  ekf_nav_message.header     = createRosHeader(ref_log_ekf_nav.timeStamp);
  ekf_nav_message.time_stamp = ref_log_ekf_nav.timeStamp;
  ekf_nav_message.status     = createEkfStatusMessage(ref_log_ekf_nav.status);
  ekf_nav_message.undulation = ref_log_ekf_nav.undulation;

  ekf_nav_message.latitude  = ref_log_ekf_nav.position[0];
  ekf_nav_message.longitude = ref_log_ekf_nav.position[1];
  ekf_nav_message.altitude  = ref_log_ekf_nav.position[2];

  if (m_use_enu_)
  {
    ekf_nav_message.velocity.x = ref_log_ekf_nav.velocity[1];
    ekf_nav_message.velocity.y = ref_log_ekf_nav.velocity[0];
    ekf_nav_message.velocity.z = -ref_log_ekf_nav.velocity[2];

    ekf_nav_message.velocity_accuracy.x = ref_log_ekf_nav.velocityStdDev[1];
    ekf_nav_message.velocity_accuracy.y = ref_log_ekf_nav.velocityStdDev[0];
    ekf_nav_message.velocity_accuracy.z = ref_log_ekf_nav.velocityStdDev[2];

    ekf_nav_message.position_accuracy.x = ref_log_ekf_nav.positionStdDev[1];
    ekf_nav_message.position_accuracy.y = ref_log_ekf_nav.positionStdDev[0];
    ekf_nav_message.position_accuracy.z = ref_log_ekf_nav.positionStdDev[2];
  }
  else
  {
    ekf_nav_message.velocity.x = ref_log_ekf_nav.velocity[0];
    ekf_nav_message.velocity.y = ref_log_ekf_nav.velocity[1];
    ekf_nav_message.velocity.z = ref_log_ekf_nav.velocity[2];

    ekf_nav_message.velocity_accuracy.x = ref_log_ekf_nav.velocityStdDev[0];
    ekf_nav_message.velocity_accuracy.y = ref_log_ekf_nav.velocityStdDev[1];
    ekf_nav_message.velocity_accuracy.z = ref_log_ekf_nav.velocityStdDev[2];

    ekf_nav_message.position_accuracy.x = ref_log_ekf_nav.positionStdDev[0];
    ekf_nav_message.position_accuracy.y = ref_log_ekf_nav.positionStdDev[1];
    ekf_nav_message.position_accuracy.z = ref_log_ekf_nav.positionStdDev[2];
  }

  return ekf_nav_message;
}

const sbg_driver::SbgEkfQuat MessageWrapper::createSbgEkfQuatMessage(const SbgLogEkfQuatData& ref_log_ekf_quat) const
{
  sbg_driver::SbgEkfQuat  ekf_quat_message;

  ekf_quat_message.header       = createRosHeader(ref_log_ekf_quat.timeStamp);
  ekf_quat_message.time_stamp   = ref_log_ekf_quat.timeStamp;
  ekf_quat_message.status       = createEkfStatusMessage(ref_log_ekf_quat.status);

  ekf_quat_message.accuracy.x   = ref_log_ekf_quat.eulerStdDev[0];
  ekf_quat_message.accuracy.y   = ref_log_ekf_quat.eulerStdDev[1];
  ekf_quat_message.accuracy.z   = ref_log_ekf_quat.eulerStdDev[2];

  if (m_use_enu_)
  {
    ekf_quat_message.quaternion.x = ref_log_ekf_quat.quaternion[1];
    ekf_quat_message.quaternion.y = -ref_log_ekf_quat.quaternion[2];
    ekf_quat_message.quaternion.z = -ref_log_ekf_quat.quaternion[3];
    ekf_quat_message.quaternion.w = ref_log_ekf_quat.quaternion[0];
  }
  else
  {
    ekf_quat_message.quaternion.x = ref_log_ekf_quat.quaternion[1];
    ekf_quat_message.quaternion.y = ref_log_ekf_quat.quaternion[2];
    ekf_quat_message.quaternion.z = ref_log_ekf_quat.quaternion[3];
    ekf_quat_message.quaternion.w = ref_log_ekf_quat.quaternion[0];
  }

  return ekf_quat_message;
}

const sbg_driver::SbgEvent MessageWrapper::createSbgEventMessage(const SbgLogEvent& ref_log_event) const
{
  sbg_driver::SbgEvent event_message;

  event_message.header      = createRosHeader(ref_log_event.timeStamp);
  event_message.time_stamp  = ref_log_event.timeStamp;

  event_message.overflow        = (ref_log_event.status & SBG_ECOM_EVENT_OVERFLOW) != 0;
  event_message.offset_0_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_0_VALID) != 0;
  event_message.offset_1_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_1_VALID) != 0;
  event_message.offset_2_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_2_VALID) != 0;
  event_message.offset_3_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_3_VALID) != 0;

  event_message.time_offset_0   = ref_log_event.timeOffset0;
  event_message.time_offset_1   = ref_log_event.timeOffset1;
  event_message.time_offset_2   = ref_log_event.timeOffset2;
  event_message.time_offset_3   = ref_log_event.timeOffset3;

  return event_message;
}

const sbg_driver::SbgGpsHdt MessageWrapper::createSbgGpsHdtMessage(const SbgLogGpsHdt& ref_log_gps_hdt) const
{
  sbg_driver::SbgGpsHdt gps_hdt_message;

  gps_hdt_message.header      = createRosHeader(ref_log_gps_hdt.timeStamp);
  gps_hdt_message.time_stamp  = ref_log_gps_hdt.timeStamp;
  gps_hdt_message.status      = ref_log_gps_hdt.status;
  gps_hdt_message.tow         = ref_log_gps_hdt.timeOfWeek;

  if (m_use_enu_)
  {
    gps_hdt_message.true_heading = wrapAngle360(90.0f - ref_log_gps_hdt.heading);
    gps_hdt_message.pitch        = -ref_log_gps_hdt.pitch;
  }
  else
  {
    gps_hdt_message.true_heading = ref_log_gps_hdt.heading;
    gps_hdt_message.pitch        = ref_log_gps_hdt.pitch;
  }

  return gps_hdt_message;
}

const sbg_driver::SbgGpsPos MessageWrapper::createSbgGpsPosMessage(const SbgLogGpsPos& ref_log_gps_pos) const
{
  sbg_driver::SbgGpsPos gps_pos_message;

  gps_pos_message.header      = createRosHeader(ref_log_gps_pos.timeStamp);
  gps_pos_message.time_stamp  = ref_log_gps_pos.timeStamp;

  gps_pos_message.status           = createGpsPosStatusMessage(ref_log_gps_pos);
  gps_pos_message.gps_tow          = ref_log_gps_pos.timeOfWeek;
  gps_pos_message.undulation       = ref_log_gps_pos.undulation;
  gps_pos_message.num_sv_used      = ref_log_gps_pos.numSvUsed;
  gps_pos_message.base_station_id  = ref_log_gps_pos.baseStationId;
  gps_pos_message.diff_age         = ref_log_gps_pos.differentialAge;

  gps_pos_message.latitude   = ref_log_gps_pos.latitude;
  gps_pos_message.longitude  = ref_log_gps_pos.longitude;
  gps_pos_message.altitude   = ref_log_gps_pos.altitude;

  if (m_use_enu_)
  {
    gps_pos_message.position_accuracy.x = ref_log_gps_pos.longitudeAccuracy;
    gps_pos_message.position_accuracy.y = ref_log_gps_pos.latitudeAccuracy;
    gps_pos_message.position_accuracy.z = ref_log_gps_pos.altitudeAccuracy;
  }
  else
  {
    gps_pos_message.position_accuracy.x = ref_log_gps_pos.latitudeAccuracy;
    gps_pos_message.position_accuracy.y = ref_log_gps_pos.longitudeAccuracy;
    gps_pos_message.position_accuracy.z = ref_log_gps_pos.altitudeAccuracy;
  }

  return gps_pos_message;
}

const sbg_driver::SbgGpsRaw MessageWrapper::createSbgGpsRawMessage(const SbgLogGpsRaw& ref_log_gps_raw) const
{
  sbg_driver::SbgGpsRaw gps_raw_message;

  gps_raw_message.data.assign(ref_log_gps_raw.rawBuffer, ref_log_gps_raw.rawBuffer + ref_log_gps_raw.bufferSize);

  return gps_raw_message;
}

const sbg_driver::SbgGpsVel MessageWrapper::createSbgGpsVelMessage(const SbgLogGpsVel& ref_log_gps_vel) const
{
  sbg_driver::SbgGpsVel gps_vel_message;

  gps_vel_message.header      = createRosHeader(ref_log_gps_vel.timeStamp);
  gps_vel_message.time_stamp  = ref_log_gps_vel.timeStamp;
  gps_vel_message.status      = createGpsVelStatusMessage(ref_log_gps_vel);
  gps_vel_message.gps_tow     = ref_log_gps_vel.timeOfWeek;
  gps_vel_message.course_acc  = ref_log_gps_vel.courseAcc;

  if (m_use_enu_)
  {
    gps_vel_message.velocity.x = ref_log_gps_vel.velocity[1];
    gps_vel_message.velocity.y = ref_log_gps_vel.velocity[0];
    gps_vel_message.velocity.z = -ref_log_gps_vel.velocity[2];

    gps_vel_message.velocity_accuracy.x = ref_log_gps_vel.velocityAcc[1];
    gps_vel_message.velocity_accuracy.y = ref_log_gps_vel.velocityAcc[0];
    gps_vel_message.velocity_accuracy.z = ref_log_gps_vel.velocityAcc[2];

    gps_vel_message.course  = wrapAngle360(90.0f - ref_log_gps_vel.course);
  }
  else
  {
    gps_vel_message.velocity.x = ref_log_gps_vel.velocity[0];
    gps_vel_message.velocity.y = ref_log_gps_vel.velocity[1];
    gps_vel_message.velocity.z = ref_log_gps_vel.velocity[2];

    gps_vel_message.velocity_accuracy.x = ref_log_gps_vel.velocityAcc[0];
    gps_vel_message.velocity_accuracy.y = ref_log_gps_vel.velocityAcc[1];
    gps_vel_message.velocity_accuracy.z = ref_log_gps_vel.velocityAcc[2];

    gps_vel_message.course  = ref_log_gps_vel.course;
  }

  return gps_vel_message;
}

const sbg_driver::SbgImuData MessageWrapper::createSbgImuDataMessage(const SbgLogImuData& ref_log_imu_data) const
{
  sbg_driver::SbgImuData  imu_data_message;

  imu_data_message.header       = createRosHeader(ref_log_imu_data.timeStamp);
  imu_data_message.time_stamp   = ref_log_imu_data.timeStamp;
  imu_data_message.imu_status   = createImuStatusMessage(ref_log_imu_data.status);
  imu_data_message.temp         = ref_log_imu_data.temperature;

  if (m_use_enu_)
  {
    imu_data_message.accel.x        = ref_log_imu_data.accelerometers[0];
    imu_data_message.accel.y        = -ref_log_imu_data.accelerometers[1];
    imu_data_message.accel.z        = -ref_log_imu_data.accelerometers[2];

    imu_data_message.gyro.x         = ref_log_imu_data.gyroscopes[0];
    imu_data_message.gyro.y         = -ref_log_imu_data.gyroscopes[1];
    imu_data_message.gyro.z         = -ref_log_imu_data.gyroscopes[2];

    imu_data_message.delta_vel.x    = ref_log_imu_data.deltaVelocity[0];
    imu_data_message.delta_vel.y    = -ref_log_imu_data.deltaVelocity[1];
    imu_data_message.delta_vel.z    = -ref_log_imu_data.deltaVelocity[2];

    imu_data_message.delta_angle.x  = ref_log_imu_data.deltaAngle[0];
    imu_data_message.delta_angle.y  = -ref_log_imu_data.deltaAngle[1];
    imu_data_message.delta_angle.z  = -ref_log_imu_data.deltaAngle[2];
  }
  else
  {
    imu_data_message.accel.x       = ref_log_imu_data.accelerometers[0];
    imu_data_message.accel.y       = ref_log_imu_data.accelerometers[1];
    imu_data_message.accel.z       = ref_log_imu_data.accelerometers[2];

    imu_data_message.gyro.x        = ref_log_imu_data.gyroscopes[0];
    imu_data_message.gyro.y        = ref_log_imu_data.gyroscopes[1];
    imu_data_message.gyro.z        = ref_log_imu_data.gyroscopes[2];

    imu_data_message.delta_vel.x   = ref_log_imu_data.deltaVelocity[0];
    imu_data_message.delta_vel.y   = ref_log_imu_data.deltaVelocity[1];
    imu_data_message.delta_vel.z   = ref_log_imu_data.deltaVelocity[2];

    imu_data_message.delta_angle.x = ref_log_imu_data.deltaAngle[0];
    imu_data_message.delta_angle.y = ref_log_imu_data.deltaAngle[1];
    imu_data_message.delta_angle.z = ref_log_imu_data.deltaAngle[2];
  }

  return imu_data_message;
}

const sbg_driver::SbgMag MessageWrapper::createSbgMagMessage(const SbgLogMag& ref_log_mag) const
{
  sbg_driver::SbgMag  mag_message;

  mag_message.header      = createRosHeader(ref_log_mag.timeStamp);
  mag_message.time_stamp  = ref_log_mag.timeStamp;
  mag_message.status      = createMagStatusMessage(ref_log_mag);

  if (m_use_enu_)
  {
    mag_message.mag.x   = ref_log_mag.magnetometers[0];
    mag_message.mag.y   = -ref_log_mag.magnetometers[1];
    mag_message.mag.z   = -ref_log_mag.magnetometers[2];

    mag_message.accel.x = ref_log_mag.accelerometers[0];
    mag_message.accel.y = -ref_log_mag.accelerometers[1];
    mag_message.accel.z = -ref_log_mag.accelerometers[2];
  }
  else
  {
    mag_message.mag.x   = ref_log_mag.magnetometers[0];
    mag_message.mag.y   = ref_log_mag.magnetometers[1];
    mag_message.mag.z   = ref_log_mag.magnetometers[2];

    mag_message.accel.x = ref_log_mag.accelerometers[0];
    mag_message.accel.y = ref_log_mag.accelerometers[1];
    mag_message.accel.z = ref_log_mag.accelerometers[2];
  }

  return mag_message;
}

const sbg_driver::SbgMagCalib MessageWrapper::createSbgMagCalibMessage(const SbgLogMagCalib& ref_log_mag_calib) const
{
  sbg_driver::SbgMagCalib mag_calib_message;

  // TODO. SbgMagCalib is not implemented.
  mag_calib_message.header = createRosHeader(ref_log_mag_calib.timeStamp);

  return mag_calib_message;
}

const sbg_driver::SbgOdoVel MessageWrapper::createSbgOdoVelMessage(const SbgLogOdometerData& ref_log_odo) const
{
  sbg_driver::SbgOdoVel odo_vel_message;

  odo_vel_message.header      = createRosHeader(ref_log_odo.timeStamp);
  odo_vel_message.time_stamp  = ref_log_odo.timeStamp;

  odo_vel_message.status  = ref_log_odo.status;
  odo_vel_message.vel     = ref_log_odo.velocity;

  return odo_vel_message;
}

const sbg_driver::SbgShipMotion MessageWrapper::createSbgShipMotionMessage(const SbgLogShipMotionData& ref_log_ship_motion) const
{
  sbg_driver::SbgShipMotion ship_motion_message;

  ship_motion_message.header        = createRosHeader(ref_log_ship_motion.timeStamp);
  ship_motion_message.time_stamp    = ref_log_ship_motion.timeStamp;
  ship_motion_message.status        = createShipMotionStatusMessage(ref_log_ship_motion);

  ship_motion_message.ship_motion.x   = ref_log_ship_motion.shipMotion[0];
  ship_motion_message.ship_motion.y   = ref_log_ship_motion.shipMotion[1];
  ship_motion_message.ship_motion.z   = ref_log_ship_motion.shipMotion[2];

  ship_motion_message.acceleration.x  = ref_log_ship_motion.shipAccel[0];
  ship_motion_message.acceleration.y  = ref_log_ship_motion.shipAccel[1];
  ship_motion_message.acceleration.z  = ref_log_ship_motion.shipAccel[2];

  ship_motion_message.velocity.x      = ref_log_ship_motion.shipVel[0];
  ship_motion_message.velocity.y      = ref_log_ship_motion.shipVel[1];
  ship_motion_message.velocity.z      = ref_log_ship_motion.shipVel[2];

  return ship_motion_message;
}

const sbg_driver::SbgStatus MessageWrapper::createSbgStatusMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatus status_message;

  status_message.header       = createRosHeader(ref_log_status.timeStamp);
  status_message.time_stamp   = ref_log_status.timeStamp;

  status_message.status_general = createStatusGeneralMessage(ref_log_status);
  status_message.status_com     = createStatusComMessage(ref_log_status);
  status_message.status_aiding  = createStatusAidingMessage(ref_log_status);

  return status_message;
}

const sbg_driver::SbgUtcTime MessageWrapper::createSbgUtcTimeMessage(const SbgLogUtcData& ref_log_utc)
{
  sbg_driver::SbgUtcTime utc_time_message;

  utc_time_message.header     = createRosHeader(ref_log_utc.timeStamp);
  utc_time_message.time_stamp = ref_log_utc.timeStamp;

  utc_time_message.clock_status = createUtcStatusMessage(ref_log_utc);
  utc_time_message.year         = ref_log_utc.year;
  utc_time_message.month        = ref_log_utc.month;
  utc_time_message.day          = ref_log_utc.day;
  utc_time_message.hour         = ref_log_utc.hour;
  utc_time_message.min          = ref_log_utc.minute;
  utc_time_message.sec          = ref_log_utc.second;
  utc_time_message.nanosec      = ref_log_utc.nanoSecond;
  utc_time_message.gps_tow      = ref_log_utc.gpsTimeOfWeek;

  if (!m_first_valid_utc_)
  {
    if (utc_time_message.clock_status.clock_stable && utc_time_message.clock_status.clock_utc_sync)
    {
      if (utc_time_message.clock_status.clock_status == SBG_ECOM_CLOCK_VALID)
      {
        m_first_valid_utc_ = true;
        ROS_INFO("A full valid UTC log has been detected, timestamp will be synchronized with the UTC data.");
      }
    }
  }

  //
  // Store the last UTC message.
  //
  m_last_sbg_utc_ = utc_time_message;

  return utc_time_message;
}

const sbg_driver::SbgAirData MessageWrapper::createSbgAirDataMessage(const SbgLogAirData& ref_air_data_log) const
{
  sbg_driver::SbgAirData air_data_message;

  air_data_message.header           = createRosHeader(ref_air_data_log.timeStamp);
  air_data_message.time_stamp       = ref_air_data_log.timeStamp;
  air_data_message.status           = createAirDataStatusMessage(ref_air_data_log);
  air_data_message.pressure_abs     = ref_air_data_log.pressureAbs;
  air_data_message.altitude         = ref_air_data_log.altitude;
  air_data_message.pressure_diff    = ref_air_data_log.pressureDiff;
  air_data_message.true_air_speed   = ref_air_data_log.trueAirspeed;
  air_data_message.air_temperature  = ref_air_data_log.airTemperature;

  return air_data_message;
}

const sbg_driver::SbgImuShort MessageWrapper::createSbgImuShortMessage(const SbgLogImuShort& ref_short_imu_log) const
{
  sbg_driver::SbgImuShort imu_short_message;

  imu_short_message.header          = createRosHeader(ref_short_imu_log.timeStamp);
  imu_short_message.time_stamp      = ref_short_imu_log.timeStamp;
  imu_short_message.imu_status      = createImuStatusMessage(ref_short_imu_log.status);
  imu_short_message.temperature     = ref_short_imu_log.temperature;

  if (m_use_enu_)
  {
    imu_short_message.delta_velocity.x  = ref_short_imu_log.deltaVelocity[0];
    imu_short_message.delta_velocity.y  = -ref_short_imu_log.deltaVelocity[1];
    imu_short_message.delta_velocity.z  = -ref_short_imu_log.deltaVelocity[2];

    imu_short_message.delta_angle.x     = ref_short_imu_log.deltaAngle[0];
    imu_short_message.delta_angle.y     = -ref_short_imu_log.deltaAngle[1];
    imu_short_message.delta_angle.z     = -ref_short_imu_log.deltaAngle[2];
  }
  else
  {
    imu_short_message.delta_velocity.x  = ref_short_imu_log.deltaVelocity[0];
    imu_short_message.delta_velocity.y  = ref_short_imu_log.deltaVelocity[1];
    imu_short_message.delta_velocity.z  = ref_short_imu_log.deltaVelocity[2];

    imu_short_message.delta_angle.x     = ref_short_imu_log.deltaAngle[0];
    imu_short_message.delta_angle.y     = ref_short_imu_log.deltaAngle[1];
    imu_short_message.delta_angle.z     = ref_short_imu_log.deltaAngle[2];
  }

  return imu_short_message;
}

const sensor_msgs::Imu MessageWrapper::createRosImuMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg, const sbg_driver::SbgEkfQuat& ref_sbg_quat_msg) const
{
  sensor_msgs::Imu imu_ros_message;

  imu_ros_message.header = createRosHeader(ref_sbg_imu_msg.time_stamp);

  imu_ros_message.orientation               = ref_sbg_quat_msg.quaternion;
  imu_ros_message.angular_velocity          = ref_sbg_imu_msg.delta_angle;
  imu_ros_message.linear_acceleration       = ref_sbg_imu_msg.delta_vel;

  imu_ros_message.orientation_covariance[0] = ref_sbg_quat_msg.accuracy.x * ref_sbg_quat_msg.accuracy.x;
  imu_ros_message.orientation_covariance[4] = ref_sbg_quat_msg.accuracy.y * ref_sbg_quat_msg.accuracy.y;
  imu_ros_message.orientation_covariance[8] = ref_sbg_quat_msg.accuracy.z * ref_sbg_quat_msg.accuracy.z;

  //
  // Angular velocity and linear acceleration covariances are not provided.
  //
  for (size_t i = 0; i < 9; i++)
  {
    imu_ros_message.angular_velocity_covariance[i]    = 0.0;
    imu_ros_message.linear_acceleration_covariance[i] = 0.0;
  }

  return imu_ros_message;
}

const sensor_msgs::Temperature MessageWrapper::createRosTemperatureMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg) const
{
  sensor_msgs::Temperature temperature_message;

  temperature_message.header      = createRosHeader(ref_sbg_imu_msg.time_stamp);
  temperature_message.temperature = ref_sbg_imu_msg.temp;
  temperature_message.variance    = 0.0;

  return temperature_message;
}

const sensor_msgs::MagneticField MessageWrapper::createRosMagneticMessage(const sbg_driver::SbgMag& ref_sbg_mag_msg) const
{
  sensor_msgs::MagneticField magnetic_message;

  magnetic_message.header         = createRosHeader(ref_sbg_mag_msg.time_stamp);
  magnetic_message.magnetic_field = ref_sbg_mag_msg.mag;

  return magnetic_message;
}

const geometry_msgs::TwistStamped MessageWrapper::createRosTwistStampedMessage(const sbg_driver::SbgEkfEuler& ref_sbg_ekf_euler_msg, const sbg_driver::SbgEkfNav& ref_sbg_ekf_nav_msg, const sbg_driver::SbgImuData& ref_sbg_imu_msg) const
{
  sbg::SbgMatrix3f tdcm;
  tdcm.makeDcm(sbg::SbgVector3f(ref_sbg_ekf_euler_msg.angle.x, ref_sbg_ekf_euler_msg.angle.y, ref_sbg_ekf_euler_msg.angle.z));
  tdcm.transpose();

  const sbg::SbgVector3f res = tdcm * sbg::SbgVector3f(ref_sbg_ekf_nav_msg.velocity.x, ref_sbg_ekf_nav_msg.velocity.y, ref_sbg_ekf_nav_msg.velocity.z);

  return createRosTwistStampedMessage(res, ref_sbg_imu_msg);
}

const geometry_msgs::TwistStamped MessageWrapper::createRosTwistStampedMessage(const sbg_driver::SbgEkfQuat& ref_sbg_ekf_quat_msg, const sbg_driver::SbgEkfNav& ref_sbg_ekf_nav_msg, const sbg_driver::SbgImuData& ref_sbg_imu_msg) const
{
  sbg::SbgMatrix3f tdcm;
  tdcm.makeDcm(ref_sbg_ekf_quat_msg.quaternion.w, ref_sbg_ekf_quat_msg.quaternion.x, ref_sbg_ekf_quat_msg.quaternion.y, ref_sbg_ekf_quat_msg.quaternion.z);
  tdcm.transpose();

  const sbg::SbgVector3f res = tdcm * sbg::SbgVector3f(ref_sbg_ekf_nav_msg.velocity.x, ref_sbg_ekf_nav_msg.velocity.y, ref_sbg_ekf_nav_msg.velocity.z);
  return createRosTwistStampedMessage(res, ref_sbg_imu_msg);
}

const geometry_msgs::TwistStamped MessageWrapper::createRosTwistStampedMessage(const sbg::SbgVector3f& body_vel, const sbg_driver::SbgImuData& ref_sbg_imu_msg) const
{
  geometry_msgs::TwistStamped twist_stamped_message;

  twist_stamped_message.header        = createRosHeader(ref_sbg_imu_msg.time_stamp);
  twist_stamped_message.twist.angular = ref_sbg_imu_msg.delta_angle;

  twist_stamped_message.twist.linear.x = body_vel(0);
  twist_stamped_message.twist.linear.y = body_vel(1);
  twist_stamped_message.twist.linear.z = body_vel(2);

  return twist_stamped_message;
}

const geometry_msgs::PointStamped MessageWrapper::createRosPointStampedMessage(const sbg_driver::SbgEkfNav& ref_sbg_ekf_msg) const
{
  geometry_msgs::PointStamped point_stamped_message;

  point_stamped_message.header = createRosHeader(ref_sbg_ekf_msg.time_stamp);

  //
  // Conversion from Geodetic coordinates to ECEF is based on World Geodetic System 1984 (WGS84).
  // Radius are expressed in meters, and latitude/longitude in radian.
  //
  double equatorial_radius;
  double polar_radius;
  double prime_vertical_radius;
  double eccentricity;
  double latitude;
  double longitude;

  equatorial_radius = 6378137.0;
  polar_radius      = 6356752.314245;
  eccentricity      = 1 - pow(polar_radius, 2) / pow(equatorial_radius, 2);
  latitude          = sbgDegToRadD(ref_sbg_ekf_msg.latitude);
  longitude         = sbgDegToRadD(ref_sbg_ekf_msg.longitude);

  prime_vertical_radius = equatorial_radius / sqrt(1.0 - pow(eccentricity, 2) * pow(sin(latitude), 2));

  point_stamped_message.point.x = (prime_vertical_radius + ref_sbg_ekf_msg.altitude) * cos(latitude) * cos(longitude);
  point_stamped_message.point.y = (prime_vertical_radius + ref_sbg_ekf_msg.altitude) * cos(latitude) * sin(longitude);
  point_stamped_message.point.z = ((pow(polar_radius, 2) / pow(equatorial_radius, 2)) * prime_vertical_radius + ref_sbg_ekf_msg.altitude) * sin(latitude);

  return point_stamped_message;
}

const sensor_msgs::TimeReference MessageWrapper::createRosUtcTimeReferenceMessage(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const
{
  sensor_msgs::TimeReference utc_reference_message;

  //
  // This message is defined to have comparison between the System time and the Utc reference.
  // Header of the ROS message will always be the System time, and the source is the computed time from Utc data.
  //
  utc_reference_message.header.stamp  = ros::Time::now();
  utc_reference_message.time_ref      = convertInsTimeToUnix(ref_sbg_utc_msg.time_stamp);
  utc_reference_message.source        = "UTC time from device converted to Epoch";

  return utc_reference_message;
}

const sensor_msgs::NavSatFix MessageWrapper::createRosNavSatFixMessage(const sbg_driver::SbgGpsPos& ref_sbg_gps_msg) const
{
  sensor_msgs::NavSatFix nav_sat_fix_message;

  nav_sat_fix_message.header = createRosHeader(ref_sbg_gps_msg.time_stamp);

  if (ref_sbg_gps_msg.status.type == SBG_ECOM_POS_NO_SOLUTION)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_NO_FIX;
  }
  else if (ref_sbg_gps_msg.status.type == SBG_ECOM_POS_SBAS)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_SBAS_FIX;
  }
  else
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_FIX;
  }

  if (ref_sbg_gps_msg.status.glo_l1_used || ref_sbg_gps_msg.status.glo_l2_used)
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GLONASS;
  }
  else
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GPS;
  }

  nav_sat_fix_message.latitude  = ref_sbg_gps_msg.latitude;
  nav_sat_fix_message.longitude = ref_sbg_gps_msg.longitude;
  nav_sat_fix_message.altitude  = ref_sbg_gps_msg.altitude + ref_sbg_gps_msg.undulation;

  nav_sat_fix_message.position_covariance[0] = ref_sbg_gps_msg.position_accuracy.x * ref_sbg_gps_msg.position_accuracy.x;
  nav_sat_fix_message.position_covariance[4] = ref_sbg_gps_msg.position_accuracy.y * ref_sbg_gps_msg.position_accuracy.y;
  nav_sat_fix_message.position_covariance[8] = ref_sbg_gps_msg.position_accuracy.z * ref_sbg_gps_msg.position_accuracy.z;

  nav_sat_fix_message.position_covariance_type = nav_sat_fix_message.COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return nav_sat_fix_message;
}

const sensor_msgs::FluidPressure MessageWrapper::createRosFluidPressureMessage(const sbg_driver::SbgAirData& ref_sbg_air_msg) const
{
  sensor_msgs::FluidPressure fluid_pressure_message;

  fluid_pressure_message.header         = createRosHeader(ref_sbg_air_msg.time_stamp);
  fluid_pressure_message.fluid_pressure = ref_sbg_air_msg.pressure_abs;
  fluid_pressure_message.variance       = 0.0;

  return fluid_pressure_message;
}
