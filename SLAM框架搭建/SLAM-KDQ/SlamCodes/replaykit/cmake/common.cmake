ZZ_PROJECT(replaykit)

include(sdk/grpc)
include(sdk/nanomsg)

ZZ_MODULE(
  NAME replaykit
  TYPE SHARED
  INCS
    ${REPLAYKIT_ROOT}/include
  SOURCES
    ${REPLAYKIT_ROOT}/src
  INTERFACES
    ${REPLAYKIT_ROOT}/include
  IDL
    ${REPLAYKIT_ROOT}/proto/replaykit.proto
  PKG replaykit
)

ZZ_MODULE(
  NAME test_replaykit
  TYPE APP
  SOURCES
  ${REPLAYKIT_ROOT}/test/main.cpp
  LINK_LIBS replaykit
)

ZZ_MODULE(
  NAME nanobag
  TYPE APP
  SOURCES
  ${REPLAYKIT_ROOT}/test/nanobag.cpp
  LINK_LIBS replaykit
  PKG replaykit
)