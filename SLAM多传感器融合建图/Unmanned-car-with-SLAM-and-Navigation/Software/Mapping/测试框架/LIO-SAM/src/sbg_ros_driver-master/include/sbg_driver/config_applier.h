/*!
*	\file         config_applier.h
*	\author       SBG Systems
*	\date         13/03/2020
*	
*	\brief        Apply configuration to the device.
*
*   Class takes a configuration from config_store and send all commands to the
*   device to apply it.
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


#ifndef CONFIG_APPLIER_H
#define CONFIG_APPLIER_H

// Standard headers
#include <limits>
#include <string>

// Project headers
#include <config_store.h>

namespace sbg
{
/*!
 * Class to apply configuration to a device.
 */
class ConfigApplier
{
private:

  bool            m_reboot_needed_;
  SbgEComHandle&  m_ref_sbg_com_handle;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Check if the configuration getter worked properly.
   * This function will log a warning information if the parameter is not available for this device.
   *
   * \param[in] ref_sbg_error_code          Error code from the configuration getter.
   * \param[in] ref_conf_title              String to identify the configuration.
   * \throw                                 Unable to get the configuration.
   */
  void checkConfigurationGet(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title) const;

  /*!
   * Check if the configuration has been applied correctly.
   * This function will log if the configuration has been applied.
   * It will log a warning if the parameter is not available for this device.
   * It will be the user responsability to check.
   *
   * \param[in] ref_sbg_error_code          Error code from the configuration getter.
   * \param[in] ref_conf_title              String to identify the configuration.
   * \throw                                 Unable to configure.
   */
  void checkConfigurationApplied(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title);

  /*!
   * Configure the initial condition parameters.
   *
   * \param[in] ref_init_condition          Initial condition conf to apply.
   */
  void configureInitCondition(const SbgEComInitConditionConf& ref_init_condition);

  /*!
   * Configure the motion profile.
   *
   * \param[in] ref_motion_profile          Motion profile configuration to apply.
   */
  void configureMotionProfile(const SbgEComModelInfo& ref_motion_profile);

  /*!
   * Configure the IMU alignement.
   *
   * \param[in] ref_sensor_align            Sensor IMU alignement configuration to apply.
   * \param[in] ref_level_arms              X, Y, Z level arms to apply.
   */
  void configureImuAlignement(const SbgEComSensorAlignmentInfo& ref_sensor_align, const SbgVector3<float>& ref_level_arms);

  /*!
   * Configure the aiding assignement.
   *
   * \param[in] ref_aiding_assign           Aiding assignement configuration to apply.
   */
  void configureAidingAssignement(const SbgEComAidingAssignConf& ref_aiding_assign);

  /*!
   * Configure the magnetometers model.
   *
   * \param[in] ref_mag_model               Magnetometers model configuration to apply.
   */
  void configureMagModel(const SbgEComModelInfo& ref_mag_model);

  /*!
   * Configure the magnetometers rejection.
   *
   * \param[in] ref_mag_rejection           Magnetometers rejection configuration to apply.
   */
  void configureMagRejection(const SbgEComMagRejectionConf& ref_mag_rejection);

  /*!
   * Configure the Gnss model.
   *
   * \param[in] ref_gnss_model              Gnss model configuration to apply.
   */
  void configureGnssModel(const SbgEComModelInfo& ref_gnss_model);

  /*!
   * Configure the Gnss installation.
   *
   * \param[in] ref_gnss_installation       Gnss installation configuration to apply.
   */
  void configureGnssInstallation(const SbgEComGnssInstallation& ref_gnss_installation);

  /*!
   * Configure the Gnss rejection.
   *
   * \param[in] ref_gnss_rejection          Gnss rejection configuration to apply.
   */
  void configureGnssRejection(const SbgEComGnssRejectionConf& ref_gnss_rejection);

  /*!
   * Configure the odometer.
   *
   * \param[in] ref_odometer                Odometer configuration to apply.
   */
  void configureOdometer(const SbgEComOdoConf& ref_odometer);

  /*!
   * Configure the odometer level arm.
   *
   * \param[in] odometer_level_arms         X,Y,Z odometer level arms to apply.
   */
  void configureOdometerLevelArm(const SbgVector3<float>& odometer_level_arms);

  /*!
   * Configure the odometer rejection.
   *
   * \param[in] ref_odometer_rejection      Odometer rejection configuration to apply.
   */
  void configureOdometerRejection(const SbgEComOdoRejectionConf& ref_odometer_rejection);

  /*!
   * Configure the output for the SBG log.
   * If a Log is not available for the connected device, a warning will be logged.
   * It will be user responsability to check.
   *
   * \param[in] output_port       Output communication port.
   * \param[in] ref_log_output    Log output to configure.
   * \throw                       Unable to configure the output.
   */
  void configureOutput(SbgEComOutputPort output_port, const ConfigStore::SbgLogOutput &ref_log_output);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   *
   * \param[in] ref_com_handle    SBG communication handle.
   */
  ConfigApplier(SbgEComHandle &ref_sbg_com_handle);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Apply a configuration to the SBG device.
   *
   * \param[in] ref_config_store            Configuration to apply.
   */
  void applyConfiguration(const ConfigStore& ref_config_store);

  /*!
   * Save the configuration to the device.
   */
  void saveConfiguration(void);
};
}

#endif // CONFIG_APPLIER_H
