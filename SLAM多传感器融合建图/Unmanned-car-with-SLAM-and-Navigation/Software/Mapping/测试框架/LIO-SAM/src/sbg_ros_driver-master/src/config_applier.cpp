// File header
#include <config_applier.h>

using sbg::ConfigApplier;

/*!
 * Class to apply configuration to a device.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

ConfigApplier::ConfigApplier(SbgEComHandle &ref_sbg_com_handle):
m_reboot_needed_(false),
m_ref_sbg_com_handle(ref_sbg_com_handle)
{

}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

void ConfigApplier::checkConfigurationGet(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title) const
{
  if (ref_sbg_error_code == SBG_INVALID_PARAMETER)
  {
    ROS_WARN("SBG_DRIVER - [Config] Configuration %s is not available for the connected device.", ref_conf_title.c_str());
  }
  else if (ref_sbg_error_code != SBG_NO_ERROR)
  {
    std::string error_message("[Config] Unable to get the ");
    error_message.append(ref_conf_title);
    error_message.append(" configuration : ");
    error_message.append(sbgErrorCodeToString(ref_sbg_error_code));

    throw ros::Exception(error_message);
  }
}

void ConfigApplier::checkConfigurationApplied(const SbgErrorCode& ref_sbg_error_code, const std::string& ref_conf_title)
{
  if (ref_sbg_error_code == SBG_INVALID_PARAMETER)
  {
    ROS_WARN("SBG_DRIVER - [Config] Configuration %s is not available for the connected device.", ref_conf_title.c_str());
  }
  else if (ref_sbg_error_code != SBG_NO_ERROR)
  {
    std::string error_message("[Config] Unable to set the ");
    error_message.append(ref_conf_title);
    error_message.append(" configuration : ");
    error_message.append(sbgErrorCodeToString(ref_sbg_error_code));

    throw ros::Exception(error_message);
  }
  else
  {
    ROS_INFO("SBG_DRIVER - [Config] %s updated on the device.", ref_conf_title.c_str());
    m_reboot_needed_ = true;
  }
}

void ConfigApplier::configureInitCondition(const SbgEComInitConditionConf& ref_init_condition)
{
  //
  // Get the initial condition of the device, compare with the loaded parameters.
  // If the conditions are different, update the device configuration with the loaded parameters.
  //
  SbgEComInitConditionConf  init_condition;
  SbgErrorCode              error_code;

  error_code = sbgEComCmdSensorGetInitCondition(&m_ref_sbg_com_handle, &init_condition);

  checkConfigurationGet(error_code, std::string("Init conditions"));

  if ((init_condition.year != ref_init_condition.year)
  ||  (init_condition.month != ref_init_condition.month)
  ||  (init_condition.day != ref_init_condition.day)
  || !areEquals(init_condition.altitude, ref_init_condition.altitude)
  || !areEquals(init_condition.latitude, ref_init_condition.latitude)
  || !areEquals(init_condition.longitude, ref_init_condition.longitude))
  {
    error_code = sbgEComCmdSensorSetInitCondition(&m_ref_sbg_com_handle, &ref_init_condition);

    checkConfigurationApplied(error_code, std::string("Init conditions"));
  }
}

void ConfigApplier::configureMotionProfile(const SbgEComModelInfo& ref_motion_profile)
{
  //
  // Get the motion profile ID, and compare with the loaded one parameter.
  // If the profiles are different, update the device with the loaded one.
  //
  SbgEComModelInfo  motion_profile;
  SbgErrorCode      error_code;

  error_code = sbgEComCmdSensorGetMotionProfileInfo(&m_ref_sbg_com_handle, &motion_profile);

  checkConfigurationGet(error_code, std::string("Motion profile"));

  if (motion_profile.id != ref_motion_profile.id)
  {
    error_code = sbgEComCmdSensorSetMotionProfileId(&m_ref_sbg_com_handle, ref_motion_profile.id);

    checkConfigurationApplied(error_code, std::string("Motion profile"));
  }
}

void ConfigApplier::configureImuAlignement(const SbgEComSensorAlignmentInfo& ref_sensor_align, const sbg::SbgVector3<float>& ref_level_arms)
{
  //
  // Get the IMU alignement and level arms, and compare with the parameters.
  // If the alignement are differents, update the device with the loaded parameters.
  //
  SbgErrorCode                error_code;
  SbgEComSensorAlignmentInfo  sensor_alignement;
  float                       level_arms_device[3];

  error_code = sbgEComCmdSensorGetAlignmentAndLeverArm(&m_ref_sbg_com_handle, &sensor_alignement, level_arms_device);

  checkConfigurationGet(error_code, std::string("IMU alignement"));

  SbgVector3<float> level_arms_vector = SbgVector3<float>(level_arms_device, 3);

  if ((level_arms_vector != ref_level_arms)
  ||  (sensor_alignement.axisDirectionX != ref_sensor_align.axisDirectionX)
  ||  (sensor_alignement.axisDirectionY != ref_sensor_align.axisDirectionY)
  || !areEquals(sensor_alignement.misRoll, ref_sensor_align.misRoll)
  || !areEquals(sensor_alignement.misPitch, ref_sensor_align.misPitch)
  || !areEquals(sensor_alignement.misYaw, ref_sensor_align.misYaw))
  {
    error_code = sbgEComCmdSensorSetAlignmentAndLeverArm(&m_ref_sbg_com_handle, &ref_sensor_align, level_arms_vector.data());

    checkConfigurationApplied(error_code, std::string("IMU alignement"));
  }
}

void ConfigApplier::configureAidingAssignement(const SbgEComAidingAssignConf& ref_aiding_assign)
{
  //
  // Get the aiding assignement, and compare with the loaded parameters.
  // If the assignement are differents, udpdate the device with the loaded parameters.
  //
  SbgEComAidingAssignConf aiding_assign;
  SbgErrorCode            error_code;

  error_code = sbgEComCmdSensorGetAidingAssignment(&m_ref_sbg_com_handle, &aiding_assign);

  checkConfigurationGet(error_code, std::string("Aiding assignement"));

  if ((aiding_assign.gps1Port != ref_aiding_assign.gps1Port)
  ||  (aiding_assign.gps1Sync != ref_aiding_assign.gps1Sync)
  ||  (aiding_assign.odometerPinsConf != ref_aiding_assign.odometerPinsConf)
  ||  (aiding_assign.rtcmPort != ref_aiding_assign.rtcmPort))
  {
    error_code = sbgEComCmdSensorSetAidingAssignment(&m_ref_sbg_com_handle, &aiding_assign);

    checkConfigurationApplied(error_code, std::string("Aiding assignement"));
  }
}

void ConfigApplier::configureMagModel(const SbgEComModelInfo& ref_mag_model)
{
  //
  // Get the magnetometer model, and compare with the loaded parameter.
  // If the model are different, update the device with the loaded parameter.
  //
  SbgEComModelInfo  model_info;
  SbgErrorCode      error_code;

  error_code = sbgEComCmdMagGetModelInfo(&m_ref_sbg_com_handle, &model_info);

  checkConfigurationGet(error_code, std::string("Magnetometer model"));

  if (model_info.id != ref_mag_model.id)
  {
    error_code = sbgEComCmdMagSetModelId(&m_ref_sbg_com_handle, ref_mag_model.id);

    checkConfigurationApplied(error_code, std::string("Magnetometer model"));
  }
}

void ConfigApplier::configureMagRejection(const SbgEComMagRejectionConf& ref_mag_rejection)
{
  //
  // Get the magnetometer rejection model, and compare with the loaded parameter.
  // If the model are different, update the device with the loaded parameter.
  //
  SbgEComMagRejectionConf mag_rejection;
  SbgErrorCode            error_code;

  error_code = sbgEComCmdMagGetRejection(&m_ref_sbg_com_handle, &mag_rejection);

  checkConfigurationGet(error_code, std::string("Magnetometer rejection"));

  if (mag_rejection.magneticField != ref_mag_rejection.magneticField)
  {
    error_code = sbgEComCmdMagSetRejection(&m_ref_sbg_com_handle, &ref_mag_rejection);

    checkConfigurationApplied(error_code, std::string("Magnetometer rejection"));
  }
}

void ConfigApplier::configureGnssModel(const SbgEComModelInfo& ref_gnss_model)
{
  //
  // Get the Gnss model, and compare with the loaded model.
  // If the models are different, update the device with the loaded model.
  //
  SbgEComModelInfo  model_info;
  SbgErrorCode      error_code;

  error_code = sbgEComCmdGnss1GetModelInfo(&m_ref_sbg_com_handle, &model_info);

  checkConfigurationGet(error_code, std::string("Gnss model"));

  if (model_info.id != ref_gnss_model.id)
  {
    error_code = sbgEComCmdGnss1SetModelId(&m_ref_sbg_com_handle, ref_gnss_model.id);

    checkConfigurationApplied(error_code, std::string("Gnss model"));
  }
}

void ConfigApplier::configureGnssInstallation(const SbgEComGnssInstallation& ref_gnss_installation)
{
  //
  // Get the Gnss level arm, and compare with the loaded parameters.
  // If the level arms are different, update the device with the loaded parameters.
  //
  SbgEComGnssInstallation gnss_installation;
  SbgErrorCode            error_code;
  SbgVector3<float>       gnss_device_primary;
  SbgVector3<float>       gnss_device_secondary;
  SbgVector3<float>       gnss_config_primary;
  SbgVector3<float>       gnss_config_secondary;

  error_code = sbgEComCmdGnss1InstallationGet(&m_ref_sbg_com_handle, &gnss_installation);

  checkConfigurationGet(error_code, std::string("Gnss level arms"));

  gnss_device_primary   = SbgVector3<float>(gnss_installation.leverArmPrimary, 3);
  gnss_device_secondary = SbgVector3<float>(gnss_installation.leverArmSecondary, 3);
  gnss_config_primary   = SbgVector3<float>(ref_gnss_installation.leverArmPrimary, 3);
  gnss_config_secondary = SbgVector3<float>(ref_gnss_installation.leverArmSecondary, 3);

  if ((gnss_device_primary != gnss_config_primary)
  || (gnss_device_secondary != gnss_config_secondary)
  || (gnss_installation.leverArmPrimaryPrecise != ref_gnss_installation.leverArmPrimaryPrecise)
  || (gnss_installation.leverArmSecondaryMode != ref_gnss_installation.leverArmSecondaryMode))
  {
    error_code = sbgEComCmdGnss1InstallationSet(&m_ref_sbg_com_handle, &ref_gnss_installation);

    checkConfigurationApplied(error_code, std::string("Gnss level arms"));
  }
}

void ConfigApplier::configureGnssRejection(const SbgEComGnssRejectionConf& ref_gnss_rejection)
{
  //
  // Get the Gnss rejection, and compare with the loaded parameters.
  // If the rejection are different, update the device with the loaded parameters.
  //
  SbgEComGnssRejectionConf  rejection;
  SbgErrorCode              error_code;

  error_code = sbgEComCmdGnss1GetRejection(&m_ref_sbg_com_handle, &rejection);

  checkConfigurationGet(error_code, std::string("Gnss rejection"));

  if ((rejection.hdt != ref_gnss_rejection.hdt)
  ||  (rejection.position != ref_gnss_rejection.position)
  ||  (rejection.velocity != ref_gnss_rejection.velocity))
  {
    error_code = sbgEComCmdGnss1SetRejection(&m_ref_sbg_com_handle, &ref_gnss_rejection);

    checkConfigurationApplied(error_code, std::string("Gnss rejection"));
  }
}

void ConfigApplier::configureOdometer(const SbgEComOdoConf& ref_odometer)
{
  //
  // Get the odometer configuration, and compare with the loaded parameters.
  // If the conf are different, update the device with the loaded parameters.
  //
  SbgEComOdoConf  odom_conf;
  SbgErrorCode    error_code;

  error_code = sbgEComCmdOdoGetConf(&m_ref_sbg_com_handle, &odom_conf);

  checkConfigurationGet(error_code, std::string("Odometer"));

  if (!areEquals(odom_conf.gain, ref_odometer.gain)
  ||  (odom_conf.gainError != ref_odometer.gainError)
  ||  (odom_conf.reverseMode != ref_odometer.reverseMode))
  {
    error_code = sbgEComCmdOdoSetConf(&m_ref_sbg_com_handle, &ref_odometer);

    checkConfigurationApplied(error_code, std::string("Odometer"));
  }
}

void ConfigApplier::configureOdometerLevelArm(const SbgVector3<float>& odometer_level_arms)
{
  //
  // Get the odometer level arm, and compare with the loaded parameters.
  // If the level arms are different, update the device with the loaded parameters.
  //
  float         lever_arm[3];
  SbgErrorCode  error_code;

  error_code = sbgEComCmdOdoGetLeverArm(&m_ref_sbg_com_handle, lever_arm);

  checkConfigurationGet(error_code, std::string("Odometer level arms"));

  SbgVector3<float> lever_arm_device = SbgVector3<float>(lever_arm, 3);

  if (lever_arm_device != odometer_level_arms)
  {
    error_code = sbgEComCmdOdoSetLeverArm(&m_ref_sbg_com_handle, lever_arm_device.data());

    checkConfigurationApplied(error_code, std::string("Odometer level arms"));
  }
}

void ConfigApplier::configureOdometerRejection(const SbgEComOdoRejectionConf& ref_odometer_rejection)
{
  //
  // Get the odometer rejection mode, and compare with the loaded parameter.
  // If the mode are different, update the device with the loaded parameter.
  //
  SbgEComOdoRejectionConf odom_rejection;
  SbgErrorCode            error_code;

  error_code = sbgEComCmdOdoGetRejection(&m_ref_sbg_com_handle, &odom_rejection);

  checkConfigurationGet(error_code, std::string("Odometer rejection"));

  if (odom_rejection.velocity != ref_odometer_rejection.velocity)
  {
    error_code = sbgEComCmdOdoSetRejection(&m_ref_sbg_com_handle, &ref_odometer_rejection);

    checkConfigurationApplied(error_code, std::string("Odometer rejection"));
  }
}

void ConfigApplier::configureOutput(SbgEComOutputPort output_port, const ConfigStore::SbgLogOutput &ref_log_output)
{
  SbgErrorCode      error_code;
  SbgEComOutputMode current_output_mode;

  //
  // Get the current output mode for the device and the selected log ID.
  // If output modes are different, udpate the device mode with the one loaded from the parameters.
  //
  error_code = sbgEComCmdOutputGetConf(&m_ref_sbg_com_handle, output_port, ref_log_output.message_class, ref_log_output.message_id, &current_output_mode);

  if (error_code == SBG_INVALID_PARAMETER)
  {
    ROS_WARN("SBG_DRIVER - [Config] Output is not available for this device : Class [%d] - Id [%d]", ref_log_output.message_class, ref_log_output.message_id);
  }
  else if (error_code != SBG_NO_ERROR)
  {
    std::string error_message("[Config] Unable to get output for the device : Class [");
    error_message.append(std::to_string(ref_log_output.message_class));
    error_message.append("] - Id [");
    error_message.append(std::to_string(ref_log_output.message_id));
    error_message.append("] : ");
    error_message.append(sbgErrorCodeToString(error_code));

    throw ros::Exception(error_message);
  }
  else if (current_output_mode != ref_log_output.output_mode)
  {
    error_code = sbgEComCmdOutputSetConf(&m_ref_sbg_com_handle, output_port, ref_log_output.message_class, ref_log_output.message_id, ref_log_output.output_mode);

    if (error_code != SBG_NO_ERROR)
    {
      std::string error_message("[Config] Unable to set the output configuration : Class[");
      error_message.append(std::to_string(ref_log_output.message_class));
      error_message.append("] - Id [");
      error_message.append(std::to_string(ref_log_output.message_id));
      error_message.append("] : ");
      error_message.append(sbgErrorCodeToString(error_code));

      throw ros::Exception(error_message);
    }
    else
    {
      m_reboot_needed_ = true;
    }
  }
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void ConfigApplier::applyConfiguration(const ConfigStore& ref_config_store)
{
  //
  // Configure the connected device.
  //
  configureInitCondition(ref_config_store.getInitialConditions());
  configureMotionProfile(ref_config_store.getMotionProfile());
  configureImuAlignement(ref_config_store.getSensorAlignement(), ref_config_store.getSensorLevelArms());
  configureAidingAssignement(ref_config_store.getAidingAssignement());
  configureMagModel(ref_config_store.getMagnetometerModel());
  configureMagRejection(ref_config_store.getMagnetometerRejection());
  configureGnssModel(ref_config_store.getGnssModel());
  configureGnssInstallation(ref_config_store.getGnssInstallation());
  configureGnssRejection(ref_config_store.getGnssRejection());
  configureOdometer(ref_config_store.getOdometerConf());
  configureOdometerLevelArm(ref_config_store.getOdometerLevelArms());
  configureOdometerRejection(ref_config_store.getOdometerRejection());

  //
  // Configure the output, with all output defined in the store.
  //
  const std::vector<ConfigStore::SbgLogOutput>& ref_output_modes = ref_config_store.getOutputModes();

  for (const ConfigStore::SbgLogOutput& ref_output : ref_output_modes)
  {
    configureOutput(ref_config_store.getOutputPort(), ref_output);
  }

  //
  // Save configuration if needed.
  //
  if (m_reboot_needed_)
  {
    saveConfiguration();
  }
}

void ConfigApplier::saveConfiguration(void)
{
  SbgErrorCode error_code;

  error_code = sbgEComCmdSettingsAction(&m_ref_sbg_com_handle, SBG_ECOM_SAVE_SETTINGS);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to save the settings on the SBG device - %s", sbgErrorCodeToString(error_code));
  }
  else
  {
    ROS_INFO("SBG_DRIVER - Settings saved and device rebooted.");
  }
}
