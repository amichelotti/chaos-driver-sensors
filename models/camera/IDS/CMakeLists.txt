cmake_minimum_required(VERSION 2.8)
project(chaos_driver_sensors_ids)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/${IDS}/include)
LINK_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/${IDS}/lib)
ADD_DEFINITIONS(-DIDSVER=\"${IDS}\")
SET(sensors_src IDSGEXXDriver.cpp Camera.cpp)

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
  ENDIF()


	MESSAGE(STATUS "Adding IDS ${IDS}")

#separate_arguments(pylon)
FILE(GLOB ids_libs ${IDS}/lib/libueye_api.so.1)
MESSAGE(STATUS "copying ${ids_libs}")
FILE(COPY ${ids_libs} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  chaos_driver_sensors ueye_api ${FrameworkLib})
FILE(GLOB files "*.h ${IDS}/include/*.h")
FILE(COPY ${files} DESTINATION include/driver/sensors/models/camera/IDS)
#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

