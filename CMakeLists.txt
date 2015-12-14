cmake_minimum_required(VERSION 2.6)
project(chaos_driver_sensors)
include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)

SET(sensors_src core/BasicSensor.cpp core/AbstractSensorDriver.cpp core/SensorDriverInterface.cpp models/ZBSensorCollector.cpp models/ZBSensorNode.cpp models/ZBSensorNodeC.cpp models/ZBSensorNodeD.cpp models/ZBSensorNodeS.cpp models/VTemperatureDriver.cpp models/VBPMSync.cpp models/LucidTemperatureDriver.cpp models/plugins.cpp)

SET(BasicSensorClient_src test/sensorsClient.cpp)

 ADD_EXECUTABLE(sensorsServer test/sensorsServer)
 ADD_EXECUTABLE(sensorsClient test/sensorsClient)

IF(BUILD_FORCE_STATIC)
  SET(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--whole-archive -lchaos_common -Wl,--no-whole-archive")
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 

ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 

ENDIF()

 TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common common_serial ${FrameworkLib})
 TARGET_LINK_LIBRARIES(sensorsServer ${PROJECT_NAME} chaos_cutoolkit chaos_common common_serial ${FrameworkLib})
 TARGET_LINK_LIBRARIES(sensorsClient chaos_uitoolkit chaos_common ${FrameworkLib})

 INSTALL_TARGETS(/bin sensorsServer)
 INSTALL_TARGETS(/bin sensorsClient)
 INSTALL_TARGETS(/lib ${PROJECT_NAME})

