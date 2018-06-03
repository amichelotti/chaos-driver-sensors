cmake_minimum_required(VERSION 2.8)
project(chaos_driver_sensors_ids)
INCLUDE_DIRECTORIES(ueye)
LINK_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/ueye/x86_64)
SET(sensors_src IDSGEXXDriver.cpp Camera.cpp)

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
  ENDIF()


	
#separate_arguments(pylon)
FILE(GLOB ids_libs ueye/x86_64/*.so*)
MESSAGE(STATUS "copying ${ids_libs}")
FILE(COPY ${ids_libs} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  chaos_driver_sensors ueye_api ${FrameworkLib})
INSTALL_TARGETS(/lib ${PROJECT_NAME})

