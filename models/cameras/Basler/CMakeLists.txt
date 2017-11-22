cmake_minimum_required(VERSION 2.6)
project(chaos_driver_sensors_basler)

SET(sensors_src BaslerScoutDriver.cpp)

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
ENDIF()
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common common_serial chaos_driver_sensors ${FrameworkLib})

INSTALL_TARGETS(/lib ${PROJECT_NAME})

