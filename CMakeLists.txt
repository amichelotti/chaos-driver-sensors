cmake_minimum_required(VERSION 2.6)
project(sensors)
include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)

SET(sensors_src core/BasicSensor.cpp core/AbstractSensorDriver.cpp core/SensorDriverInterface.cpp models/VTemperatureDriver.cpp)

SET(BasicSensorClient_src test/sensorsClient.cpp)

 ADD_EXECUTABLE(sensorsServer test/sensorsServer)
 ADD_EXECUTABLE(sensorsClient test/sensorsClient)

IF(BUILD_FORCE_STATIC)
  SET(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--whole-archive -lchaos_common -Wl,--no-whole-archive")
   ADD_LIBRARY(chaos_driver_sensors STATIC ${sensors_src})		 

ELSE()
    ADD_LIBRARY(chaos_driver_sensors SHARED ${sensors_src})		 

ENDIF()

 TARGET_LINK_LIBRARIES(chaos_driver_sensors chaos_cutoolkit chaos_common ${FrameworkLib})
 TARGET_LINK_LIBRARIES(sensorsServer chaos_driver_sensors chaos_cutoolkit chaos_common ${FrameworkLib})
 TARGET_LINK_LIBRARIES(sensorsClient chaos_uitoolkit chaos_common ${FrameworkLib})

 INSTALL_TARGETS(/bin sensorsServer)
 INSTALL_TARGETS(/bin sensorsClient)
 INSTALL_TARGETS(/lib chaos_driver_sensors)

