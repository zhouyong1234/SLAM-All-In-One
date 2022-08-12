# Changelog

## v1.3.0 - 2020-11-10

### Added

- Add RSHELIOS support
- Add RSM1 (B3) support
- Add Windows support
- Add rs_driver_viewer, a small tool to show point cloud
- Add save_by_rows argument
- Add multi-cast support
- Add points transformation function

### Changed

- Update some decoding part for LiDARs
- Change the definition of packet message
- Update documents



## v1.2.1 - 2020-09-04

### Fixed

- Fix the timestamp calculation for RS16,RS32 & RSBP. Now the output lidar timestamp will be UTC time and will not be affected by system time zone.

## v1.2.0 - 2020-09-01

### Added

- Add interface in driver core to get lidar temperature
- Add support for point type XYZIRT (R - ring id)(T - timestamp)
- Add RS80 support
- Add interface in driver core to get camera trigger info

### Changed

- Update the decoding part for ruby in echo-dual mode
- Update the compiler version from C++11 to C++14

## v1.1.0 - 2020-07-01

### Added

- Add the limit of the length of the msop queue 
- Add the exception capture when loading .csv file

### Fixed
- Fix the bug in calculating the timestamp of 128
- Fix the bug in calculating RPM

### Changed
- Update some functions' names
- Update the program structure

### Removed
- Remove unused variables in point cloud message

## v1.0.0 - 2020-06-01

### Added 

- New program structure

- Easy to do advanced development

- Remove the redundant code in old driver.

  