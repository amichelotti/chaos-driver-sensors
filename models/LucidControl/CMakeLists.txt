cmake_minimum_required(VERSION 2.6)
project(chaos_driver_sensors_lucidcontrol)

SET(sensors_src LucidTemperatureDriver.cpp LucidOutputDriver.cpp)

IF(BUILD_FORCE_STATIC)
  SET(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--whole-archive -lchaos_common -Wl,--no-whole-archive")
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src})		 

ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src})		 

ENDIF()
SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_driver_sensors chaos_common common_serial ${FrameworkLib})
FILE(GLOB src *.h)    
INSTALL(FILES ${src} DESTINATION include/driver/sensors/models/LucidControl ) 

 #INSTALL_TARGETS(/lib ${PROJECT_NAME})
 INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
