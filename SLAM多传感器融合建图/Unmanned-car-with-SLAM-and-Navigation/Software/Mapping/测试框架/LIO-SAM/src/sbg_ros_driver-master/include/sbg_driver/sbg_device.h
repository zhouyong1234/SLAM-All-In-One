/*!
*	\file         sbg_device.h
*	\author       SBG Systems
*	\date         13/03/2020
*	
*	\brief        Handle a connected SBG device.
*
*   Methods to communicate with the device.
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

#ifndef SBG_ROS_SBG_DEVICE_H
#define SBG_ROS_SBG_DEVICE_H

// Standard headers
#include <iostream>
#include <map>
#include <string>

// ROS headers
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

// Project headers
#include <config_applier.h>
#include <config_store.h>
#include <message_publisher.h>

namespace sbg
{
/*!
 * Class to handle a connected SBG device.
 */
class SbgDevice
{
private:

  //---------------------------------------------------------------------//
  //- Static members definition                                         -//
  //---------------------------------------------------------------------//

  static std::map<SbgEComMagCalibQuality, std::string>    g_mag_calib_quality_;
  static std::map<SbgEComMagCalibConfidence, std::string> g_mag_calib_confidence_;
  static std::map<SbgEComMagCalibMode, std::string>       g_mag_calib_mode_;
  static std::map<SbgEComMagCalibBandwidth, std::string>  g_mag_calib_bandwidth;

  //---------------------------------------------------------------------//
  //- Private variables                                                 -//
  //---------------------------------------------------------------------//

  SbgEComHandle           m_com_handle_;
  SbgInterface            m_sbg_interface_;
  ros::NodeHandle&        m_ref_node_;
  MessagePublisher        m_message_publisher_;
  ConfigStore             m_config_store_;

  uint32_t                m_rate_frequency_;

  bool                    m_mag_calibration_ongoing_;
  bool                    m_mag_calibration_done_;
  SbgEComMagCalibResults  m_magCalibResults;
  ros::ServiceServer      m_calib_service_;
  ros::ServiceServer      m_calib_save_service_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   *  Callback definition called each time a new log is received.
   *
   *  \param[in]  pHandle         Valid handle on the sbgECom instance that has called this callback.
   *  \param[in]  msg_class       Class of the message we have received
   *  \param[in]  msg             Message ID of the log received.
   *  \param[in]  p_log_data      Contains the received log data as an union.
   *  \param[in]  p_user_arg      Optional user supplied argument.
   *  \return                     SBG_NO_ERROR if the received log has been used successfully.
   */
  static SbgErrorCode onLogReceivedCallback(SbgEComHandle* p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData* p_log_data, void* p_user_arg);

  /*!
   * Function to handle the received log.
   *
   * \param[in]  msg_class        Class of the message we have received
   * \param[in]  msg              Message ID of the log received.
   * \param[in]  ref_sbg_data     Contains the received log data as an union.
   */
  void onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData& ref_sbg_data);

  /*!
   * Load the parameters.
   */
  void loadParameters(void);

  /*!
   * Create the connection to the SBG device.
   *
   * \throw                       Unable to connect to the SBG device.
   */
  void connect(void);

  /*!
   * Read the device informations.
   *
   * \throw                       Unable to read the device information.
   */
  void readDeviceInfo(void);

  /*!
   * Get the SBG version as a string.
   *
   * \param[in] sbg_version_enc   SBG version encoded.
   * \return                      String version decoded.
   */
  std::string getVersionAsString(uint32 sbg_version_enc) const;

  /*!
   * Initialize the publishers according to the configuration.
   */
  void initPublishers(void);

  /*!
   * Configure the connected SBG device.
   * This function will configure the device if the config file allows it.
   * It will log warning for unavailable parameters for the connected device.
   *
   * \throw                       Unable to configure the connected device.
   */
  void configure(void);

  /*!
   * Process the magnetometer calibration.
   *
   * \param[in] ref_ros_request   ROS service request.
   * \param[in] ref_ros_response  ROS service response.
   * \return                      Return true if the calibration process has been succesfull.
   */
  bool processMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response);

  /*!
   * Save the magnetometer calibration.
   *
   * \param[in] ref_ros_request   ROS service request.
   * \param[in] ref_ros_response  ROS service response.
   * \return                      Return true if the calibration has been saved.
   */
  bool saveMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response);

  /*!
   * Start the magnetometer calibration process.
   *
   * \return                      True if the calibration process has started successfully.
   */
  bool startMagCalibration(void);

  /*!
   * End the magnetometer calibration process.
   *
   * \return                      True if the calibration process has ended successfully.
   */
  bool endMagCalibration(void);

  /*!
   * Upload the magnetometers calibration results to the device.
   *
   * \return                      True if the magnetometers calibration has been successfully uploaded to the device.
   */
  bool uploadMagCalibrationToDevice(void);

  /*!
   * Display magnetometers calibration status result.
   */
  void displayMagCalibrationStatusResult(void) const;

  /*!
   * Export magnetometers calibration results.
   */
  void exportMagCalibrationResults(void) const;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   *
   * \param[in] ref_node_handle   ROS NodeHandle.
   */
  SbgDevice(ros::NodeHandle& ref_node_handle);

  /*!
   * Default destructor.
   */
  ~SbgDevice(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Get the frequency to update the main rate loop for device handling.
   *
   * \return                      Device frequency to read the logs (in Hz).
   */
  uint32_t getUpdateFrequency(void) const;

  //---------------------------------------------------------------------//
  //- Public  methods                                                   -//
  //---------------------------------------------------------------------//

  /*!
   * Initialize the SBG device for receiving data.
   *
   * \throw                       Unable to initialize the SBG device.
   */
  void initDeviceForReceivingData(void);

  /*!
   * Initialize the device for magnetometers calibration.
   */
  void initDeviceForMagCalibration(void);

  /*!
   * Periodic handle of the connected SBG device.
   */
  void periodicHandle(void);
};
}

#endif // SBG_ROS_SBG_DEVICE_H
