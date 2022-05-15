cmake_minimum_required(VERSION 2.8)
project(chaos_driver_sensors_basler)
SET(PYLON_MAJOR 5)
SET(PYLON_SOFT ${CMAKE_CURRENT_SOURCE_DIR}/pylon5)

IF(PYLON_VERSION)
string(REPLACE "." ";" VERSION_LIST ${PYLON_VERSION})
list(GET VERSION_LIST 0 PYLON_MAJOR)
list(GET VERSION_LIST 1 PYLON_MINOR)
list(GET VERSION_LIST 2 PYLON_PATCH)
MESSAGE(STATUS "Pylon major: ${PYLON_MAJOR} minor  ${PYLON_MINOR}")

ELSE()
SET(PYLON_VERSION "5.1.0")
ENDIF()

IF(PYLON_DIR)
SET(PYLON_SOFT ${PYLON_DIR})
ENDIF()

SET(sensors_src BaslerScoutDriver.cpp)
IF(${PYLON_MAJOR} GREATER 5)
  SET (pylon_gcc_api_basler  GenApi_gcc_v3_1_Basler_pylon)
  SET (pylon_gcc_base GCBase_gcc_v3_1_Basler_pylon)
  SET (pylon_gcc_log Log_gcc_v3_1_Basler_pylon)
  SET (pylon_gcc_parser MathParser_gcc_v3_1_Basler_pylon)

  SET (pylon_gcc_xml XmlParser_gcc_v3_1_Basler_pylon)
  SET (pylon_gcc_map NodeMapData_gcc_v3_1_Basler_pylon)

ELSE()
  find_library (pylon_gcc_api_basler NAMES GenApi_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON_SOFT}/lib)
  find_library (pylon_gcc_base NAMES GCBase_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON_SOFT}/lib)
  find_library (pylon_gcc_log NAMES Log_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON_SOFT}/lib)
  find_library (pylon_gcc_parser NAMES MathParser_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON_SOFT}/lib)

  find_library (pylon_gcc_xml NAMES XmlParser_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON_SOFT}/lib)
  find_library (pylon_gcc_map NAMES NodeMapData_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON_SOFT}/lib)

ENDIF()
LINK_DIRECTORIES(${PYLON_SOFT}/lib)
MESSAGE(STATUS "Adding Basler pylon in ${PYLON_SOFT}/lib")

INCLUDE_DIRECTORIES(${PYLON_SOFT}/include)

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
  ENDIF()

ADD_DEFINITIONS(-DBASLERVER=\"${PYLON_VERSION}\")

#SET(pylonbase pylonbase)
#SET (pylonutil pylonutility)
	
SET(pylon pylonbase-${PYLON_VERSION} pylonutility-${PYLON_VERSION} ${pylon_gcc_api_basler} ${pylon_gcc_base} ${pylon_gcc_log} ${pylon_gcc_parser} ${pylon_gcc_xml} ${pylon_gcc_map})
#separate_arguments(pylon)
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  chaos_driver_sensors ${pylon} ${FrameworkLib} common_misc_data)
FILE(GLOB pylon_libs ${PYLON_SOFT}/lib/*.so)
if(${CMAKE_VERSION} VERSION_LESS "3.23.1") 
FILE(COPY ${pylon_libs} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
ELSE()
FILE(COPY ${pylon_libs} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib FOLLOW_SYMLINK_CHAIN)
ENDIF()

MESSAGE(STATUS "pylon libs ${pylon_libs}")

#INSTALL(FILES ${pylon_libs} DESTINATION lib)
FILE(GLOB files "*.h")
INSTALL(FILES ${files} DESTINATION include/driver/sensors/models/camera/Basler)

#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
