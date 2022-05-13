cmake_minimum_required(VERSION 2.8)
project(chaos_driver_sensors_basler)

SET(sensors_src BaslerScoutDriver.cpp)
IF(EXISTS /opt/pylon)
SET(PYLON_VERSION 6.3.0)
SET (pylon_gcc_api_basler  GenApi_gcc_v3_1_Basler_pylon)
SET (pylon_gcc_base GCBase_gcc_v3_1_Basler_pylon)
SET (pylon_gcc_log Log_gcc_v3_1_Basler_pylon)
SET (pylon_gcc_parser MathParser_gcc_v3_1_Basler_pylon)

SET (pylon_gcc_xml XmlParser_gcc_v3_1_Basler_pylon)
SET (pylon_gcc_map NodeMapData_gcc_v3_1_Basler_pylon)


ELSE()
SET(PYLON_SOFT pylon5)
SET(PYLON_VERSION 5.1.0)
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
FILE(COPY ${pylon_libs} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib FOLLOW_SYMLINK_CHAIN)
MESSAGE(STATUS "pylon libs ${pylon_libs}")

#INSTALL(FILES ${pylon_libs} DESTINATION lib)
FILE(GLOB files "*.h")
INSTALL(FILES ${files} DESTINATION include/driver/sensors/models/camera/Basler)

#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
