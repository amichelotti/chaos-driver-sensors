cmake_minimum_required(VERSION 2.6)
include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)

SET(BasicSensor_src core/main.cpp core/BasicSensor.cpp core/BasicSensorDriver.cpp )

SET(BasicSensorClient_src test/BasicSensorClient.cpp )

ADD_EXECUTABLE(BasicSensor ${BasicSensor_src})
ADD_EXECUTABLE(BasicSensorClient ${BasicSensorClient_src})

IF(BUILD_FORCE_STATIC)
  SET(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--whole-archive -lchaos_common -Wl,--no-whole-archive")
TARGET_LINK_LIBRARIES(BasicSensor chaos_cutoolkit ${FrameworkLib})
TARGET_LINK_LIBRARIES(BasicSensorClient chaos_uitoolkit ${FrameworkLib})
ELSE()
TARGET_LINK_LIBRARIES(BasicSensor chaos_cutoolkit chaos_common ${FrameworkLib})
TARGET_LINK_LIBRARIES(BasicSensorClient chaos_uitoolkit chaos_common ${FrameworkLib})
ENDIF()


INSTALL_TARGETS(/bin BasicSensor)
INSTALL_TARGETS(/bin BasicSensorClient)
