# Release a new version for the sbg_driver

This page will explain how the sbg_driver should be released, according to these docs : 
* [ROS/ReleasingAPackage](http://wiki.ros.org/ROS/ReleasingAPackage)
* [bloom/ReleaseCatkinPackage](http://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage)
* [bloom/FirstTimeRelease](http://wiki.ros.org/bloom/Tutorials/FirstTimeRelease#Configure_a_Release_Track)
* [ROS/PrereleaseTests](http://wiki.ros.org/regression_tests#How_do_I_setup_my_system_to_run_a_prerelease.3F)

## Update the changelog
Once all changes have been committed in the master branch, and the driver is ready for release, update the changelog file.

In the source directory of the driver, execute the following command. 
Clean up the changelog file, but do not touch the "Forthcoming" title, it will be updated by bloom during the release process.
Once it has been modified, commit the changelog.

```
catkin_generate_changelog
```

## Update the sbg_driver version
Use the catkin command to prepare the driver for release. You can add the '--bump' option, to choose the version increment.
By default, it will increase the patch version, 2.0.2 -> 2.0.3

```
catkin_prepare_release --bump {major, minor, patch}
```

## Running prerelease tests
A completed prerelease test is a quite good guarantee that the package will run perfectly. Follow the instructions from the 
prerelease doc pages.

Install the python3-ros-buildfarm package.
```
sudo apt-get install python3-ros-buildfarm
```

The following command could be used to generate the prerelease executables.
```
generate_prerelease_script.py \
https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml \
melodic default ubuntu bionic amd64 sbg_driver --level 0 --output-dir . 
```

## Release the package
Execute the bloom command to release, for every required rosdistro (Melodic, Kinetic).
These command will ask for automatic push to the sbg_driver_release repository, and pull request to the rosdistro repository.

```
bloom-release sbg_driver --rosdistro <rosdistro>
```

In case you want to release the driver for a new distro, use the following command, 
and refer to the documentation page to release a package for the first time.

```
bloom-release --rosdistro <ros_distro> --track <ros_distro> sbg_driver --edit
```
