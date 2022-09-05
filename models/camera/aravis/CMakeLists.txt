cmake_minimum_required(VERSION 3.0)
project(chaos_driver_sensors_aravis)

SET(sensors_src AravisDriver.cpp)

IF(NOT ARAVIS_LIB)
  SET(ARAVIS_LIB ${ARAVIS}/lib/x86_64-linux-gnu)
ENDIF()

    FILE(GLOB aravis_l ${ARAVIS_LIB}/*.so)

    FILE(COPY ${aravis_l} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

    INCLUDE_DIRECTORIES(${ARAVIS}/include/aravis-0.8 /usr/include/glib-2.0 /usr/lib/x86_64-linux-gnu/glib-2.0/include)
    link_directories(${ARAVIS_LIB})
    SET(camera_lib ${camera_lib} aravis-0.8)
    SET(camera_src models/camera/aravis/AravisDriver.cpp)
    # FILE(GLOB aravis_h "${ARAVIS_INCLUDE}/*.h")
    # FILE(COPY ${aravis_h} DESTINATION include/driver/sensors/core)
    # INSTALL(FILES ${aravis_h} DESTINATION include/driver/sensors/core)
MESSAGE(STATUS "Generic ARAVIS camera support PACKAGE INCLUDE: ${ARAVIS_INCLUDE} LIB: ${ARAVIS_LIB} ${aravis_l}")

IF(BUILD_FORCE_STATIC)
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 
ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 
ENDIF()

SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  chaos_driver_sensors aravis-0.8 ${FrameworkLib} )


#INSTALL(FILES ${pylon_libs} DESTINATION lib)
FILE(GLOB files "*.h")
INSTALL(FILES ${files} DESTINATION include/driver/sensors/models/camera/aravis)

#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
