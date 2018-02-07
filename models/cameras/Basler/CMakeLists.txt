cmake_minimum_required(VERSION 2.8)
project(chaos_driver_sensors_basler)

SET(sensors_src BaslerScoutDriver.cpp)
INCLUDE_DIRECTORIES(${BASLER}/include)

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
  ENDIF()

SET(PYLON_VERSION 5.0.11)

find_library (pylonbase NAMES pylonbase-${PYLON_VERSION}  PATHS ${BASLER}/lib64 ${BASLER}/lib)
find_library (pylonutil NAMES pylonutility-${PYLON_VERSION} PATHS ${BASLER}/lib64 ${BASLER}/lib)
find_library (pylon_gcc_api_basler NAMES GenApi_gcc_v3_0_Basler_pylon_v5_0 PATHS ${BASLER}/lib64 ${BASLER}/lib)

find_library (pylon_gcc_base NAMES GCBase_gcc_v3_0_Basler_pylon_v5_0 PATHS ${BASLER}/lib64 ${BASLER}/lib)
find_library (pylon_gcc_log NAMES Log_gcc_v3_0_Basler_pylon_v5_0 PATHS ${BASLER}/lib64 ${BASLER}/lib)
find_library (pylon_gcc_parser NAMES MathParser_gcc_v3_0_Basler_pylon_v5_0 PATHS ${BASLER}/lib64 ${BASLER}/lib)

find_library (pylon_gcc_xml NAMES XmlParser_gcc_v3_0_Basler_pylon_v5_0 PATHS ${BASLER}/lib64 ${BASLER}/lib)
find_library (pylon_gcc_map NAMES NodeMapData_gcc_v3_0_Basler_pylon_v5_0 PATHS ${BASLER}/lib64 ${BASLER}/lib)

	
SET(pylon ${pylonbase} ${pylonutil} ${pylon_gcc_api_basler} ${pylon_gcc_base} ${pylon_gcc_log} ${pylon_gcc_parser} ${pylon_gcc_xml} ${pylon_gcc_map})
#separate_arguments(pylon)
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  chaos_driver_sensors ${pylon} ${FrameworkLib})
MESSAGE(STATUS "pylon libs ${pylon}")
INSTALL(FILES ${pylon} DESTINATION lib)
INSTALL_TARGETS(/lib ${PROJECT_NAME})

