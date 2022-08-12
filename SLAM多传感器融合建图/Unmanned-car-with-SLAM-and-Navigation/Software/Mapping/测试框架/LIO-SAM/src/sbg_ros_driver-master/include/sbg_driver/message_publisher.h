/*!
*	\file         message_publisher.h
*	\author       SBG Systems
*	\date         13/03/2020
*	
*	\brief        Manage publishment of messages from logs.
*	
*	\section CodeCopyright Copyright Notice
*	MIT License
*	
*	Copyright (c) 2020 SBG Systems
*	
*	Permission is hereby granted, free of charge, to any person obtaining a copy
*	of this software and associated documentation files (the "Software"), to deal
*	in the Software without restriction, including without limitation the rights
*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*	copies of the Software, and to permit persons to whom the Software is
*	furnished to do so, subject to the following conditions:
*	
*	The above copyright notice and this permission notice shall be included in all
*	copies or substantial portions of the Software.
*	
*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*	SOFTWARE.
*/

#ifndef SBG_ROS_MESSAGE_PUBLISHER_H
#define SBG_ROS_MESSAGE_PUBLISHER_H

// Project headers
#include <config_store.h>
#include <message_wrapper.h>

namespace sbg
{
/*!
 * Class to publish all SBG-ROS messages to the corresponding publishers.
 */
class MessagePublisher
{
private:

  ros::Publisher          m_sbgStatus_pub_;
  ros::Publisher          m_sbgUtcTime_pub_;
  ros::Publisher          m_sbgImuData_pub_;
  ros::Publisher          m_sbgEkfEuler_pub_;
  ros::Publisher          m_sbgEkfQuat_pub_;
  ros::Publisher          m_sbgEkfNav_pub_;
  ros::Publisher          m_sbgShipMotion_pub_;
  ros::Publisher          m_sbgMag_pub_;
  ros::Publisher          m_sbgMagCalib_pub_;
  ros::Publisher          m_sbgGpsVel_pub_;
  ros::Publisher          m_sbgGpsPos_pub_;
  ros::Publisher          m_sbgGpsHdt_pub_;
  ros::Publisher          m_sbgGpsRaw_pub_;
  ros::Publisher          m_sbgOdoVel_pub_;
  ros::Publisher          m_sbgEventA_pub_;
  ros::Publisher          m_sbgEventB_pub_;
  ros::Publisher          m_sbgEventC_pub_;
  ros::Publisher          m_sbgEventD_pub_;
  ros::Publisher          m_sbgEventE_pub_;
  ros::Publisher          m_sbgImuShort_pub_;
  ros::Publisher          m_sbgAirData_pub_;

  ros::Publisher          m_imu_pub_;
  sbg_driver::SbgImuData  m_sbg_imu_message_;
  sbg_driver::SbgEkfQuat  m_sbg_ekf_quat_message_;
  sbg_driver::SbgEkfNav   m_sbg_ekf_nav_message_;
  sbg_driver::SbgEkfEuler m_sbg_ekf_euler_message_;

  ros::Publisher          m_temp_pub_;
  ros::Publisher          m_mag_pub_;
  ros::Publisher          m_fluid_pub_;
  ros::Publisher          m_pos_ecef_pub_;
  ros::Publisher          m_velocity_pub_;
  ros::Publisher          m_utc_reference_pub_;
  ros::Publisher          m_nav_sat_fix_pub_;

  MessageWrapper          m_message_wrapper_;
  uint32_t                m_max_messages_;
  std::string             m_frame_id_;

  //---------------------------------------------------------------------//
  //- Private methods                                                   -//
  //---------------------------------------------------------------------//

  /*!
   * Get the corresponding topic name output for the SBG output mode.
   *
   * \param[in] sbg_message_id          SBG message ID.
   * \return                            Output topic name.
   */
  std::string getOutputTopicName(SbgEComMsgId sbg_message_id) const;

  /*!
   * Initialize the publisher for the specified SBG Id, and the output configuration.
   *
   * \param[in] ref_ros_node_handle     Ros NodeHandle to advertise the publisher.
   * \param[in] sbg_msg_id              Id of the SBG message.
   * \param[in] output_conf             Output configuration.
   * \param[in] ref_output_topic        Output topic for the publisher.
   */
  void initPublisher(ros::NodeHandle& ref_ros_node_handle, SbgEComMsgId sbg_msg_id, SbgEComOutputMode output_conf, const std::string &ref_output_topic);

  /*!
   * Define standard ROS publishers.
   *
   * \param[in] ref_ros_node_handle     Ros NodeHandle to advertise the publisher.
   */
  void defineRosStandardPublishers(ros::NodeHandle& ref_ros_node_handle);

  /*!
   * Publish a received SBG IMU log.
   *
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishIMUData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Process a ROS Velocity standard message.
   */
  void processRosVelMessage(void);

  /*!
   * Process a ROS IMU standard message.
   */
  void processRosImuMessage(void);

  /*!
   * Publish a received SBG Magnetic log.
   *
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishMagData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Publish a received SBG Fluid pressure log.
   *
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishFluidPressureData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Publish a received SBG EkfNav log.
   *
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishEkfNavigationData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Publish a received SBG UTC log.
   *
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishUtcData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Publish a received SBG GpsPos log.
   *
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishGpsPosData(const SbgBinaryLogData &ref_sbg_log);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  MessagePublisher(void);

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Initialize the publishers for the output configuration.
   *
   * \param[in] ref_ros_node_handle     Ros NodeHandle to advertise the publisher.
   * \param[in] ref_config_store        Store configuration for the publishers.
   */
  void initPublishers(ros::NodeHandle& ref_ros_node_handle, const ConfigStore &ref_config_store);

  /*!
   * Publish the received SbgLog if the corresponding publisher is defined.
   *
   * \param[in] sbg_msg_class           Class ID of the SBG message.
   * \param[in] sbg_msg_id              Id of the SBG message.
   * \param[in] ref_sbg_log             SBG binary log.
   */
  void publish(SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id, const SbgBinaryLogData &ref_sbg_log);
};
}

#endif // SBG_ROS_MESSAGE_PUBLISHER_H
