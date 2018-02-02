cmake_minimum_required(VERSION 2.8)
project(chaos_driver_sensors_basler)

SET(sensors_src BaslerScoutDriver.cpp)
INCLUDE_DIRECTORIES(${BASLER}/include)

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
ENDIF()

find_library (pylon NAMES pylonbase pylonutility GenApi_gcc_v3_0_Basler_pylon_v5_0 GCBase_gcc_v3_0_Basler_pylon_v5_0 PATHS ${BASLER}/lib64 ${BASLER}/lib)

SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  chaos_driver_sensors ${pylon} ${FrameworkLib})

INSTALL_TARGETS(/lib ${PROJECT_NAME})

