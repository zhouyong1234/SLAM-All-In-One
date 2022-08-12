# workarounds that need to be applied before the `project(...)` call

# macOS < 10.15 requires that we set CMAKE_OSX_DEPLOYMENT_TARGET to
# 10.15 before `project(...)` is called, otherwise clang thinks
# certain C++17 STL features such as std::visit and std::filesystem
# are not available.
if(APPLE)
  # Note: CMAKE_SYSTEM_VERSION doesn't work before `project(...)`
  execute_process(COMMAND sw_vers -productVersion OUTPUT_VARIABLE _macos_version)
  string(REGEX REPLACE "\n$" "" _macos_version "${_macos_version}")
  if (_macos_version VERSION_LESS 10.15.0)
    message(STATUS "Detected macOS version '${_macos_version}',  which is earlier than macOS 10.15 Catalina. Applying workarounds for clang and libc++...")

    # Ensure libc++ enables all features.
    # See: https://stackoverflow.com/a/53868971/1813258
    # See: https://stackoverflow.com/a/53887048/1813258
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "Minimum OS X deployment version")
    message(STATUS "... setting deployment target to '${CMAKE_OSX_DEPLOYMENT_TARGET}' to trick libc++ into not disabling some features (like std::visit)")
    message(STATUS "... compiler set to '${CMAKE_C_COMPILER}' and '${CMAKE_CXX_COMPILER}'")
  else()
    message(STATUS "Detected macOS version '${_macos_version}', which is newer or equal to macOS 10.15 Catalina. Not applying workarounds.")
  endif()
endif()

