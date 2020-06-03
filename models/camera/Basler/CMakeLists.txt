cmake_minimum_required(VERSION 2.8)
project(chaos_driver_sensors_basler)

SET(sensors_src BaslerScoutDriver.cpp)
SET(PYLON5 pylon5)
INCLUDE_DIRECTORIES(${PYLON5}/include)

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
  ENDIF()
  MESSAGE(STATUS "Adding Basler")

SET(PYLON_VERSION 5.1.0)

find_library (pylonbase NAMES pylonbase-${PYLON_VERSION}  PATHS ${PYLON5}/lib)
find_library (pylonutil NAMES pylonutility-${PYLON_VERSION} PATHS ${PYLON5}/lib)
find_library (pylon_gcc_api_basler NAMES GenApi_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON5}/lib)

find_library (pylon_gcc_base NAMES GCBase_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON5}/lib)
find_library (pylon_gcc_log NAMES Log_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON5}/lib)
find_library (pylon_gcc_parser NAMES MathParser_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON5}/lib)

find_library (pylon_gcc_xml NAMES XmlParser_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON5}/lib)
find_library (pylon_gcc_map NAMES NodeMapData_gcc_v3_1_Basler_pylon_v5_1 PATHS ${PYLON5}/lib)

	
SET(pylon ${pylonbase} ${pylonutil} ${pylon_gcc_api_basler} ${pylon_gcc_base} ${pylon_gcc_log} ${pylon_gcc_parser} ${pylon_gcc_xml} ${pylon_gcc_map})
#separate_arguments(pylon)
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  chaos_driver_sensors ${pylon} ${FrameworkLib} common_misc_data)
MESSAGE(STATUS "pylon libs ${pylon}")
FILE(GLOB pylon_libs ${PYLON5}/lib/*.so)
INSTALL(FILES ${pylon_libs} DESTINATION lib)
FILE(GLOB files "*.h")
INSTALL(FILES ${files} DESTINATION include/driver/sensors/models/camera/Basler)

#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
