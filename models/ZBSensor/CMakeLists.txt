cmake_minimum_required(VERSION 2.6)
project(chaos_driver_sensors_ZB)

SET(sensors_src ZBSensorCollector.cpp ZBSensorNode.cpp ZBSensorNodeC.cpp ZBSensorNodeD.cpp ZBSensorNodeS.cpp)

IF(BUILD_FORCE_STATIC)
  SET(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--whole-archive -lchaos_common -Wl,--no-whole-archive")
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 

ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 

ENDIF()
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
 TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_driver_sensors chaos_cutoolkit chaos_common common_serial ${FrameworkLib})
FILE(GLOB src *.h)    
INSTALL(FILES ${src} DESTINATION include/driver/sensors/models/ZBSensor ) 

 INSTALL_TARGETS(/lib ${PROJECT_NAME})

