cmake_minimum_required(VERSION 2.6)
project(chaos_driver_sensors)
find_package(chaos MODULE REQUIRED)
include_directories(../../ ${chaos_INCLUDE_DIRS})
add_definitions(-Wno-ignored-attributes -Wno-deprecated-declarations)
FILE(GLOB sensors_src core/*.cpp)
SET(camera_src "")
SET(camera_lib "")
IF(OPENCV)
  include_directories(../../ ${chaos_INCLUDE_DIRS} ${OPENCV}/include)
  FILE(GLOB camera_src models/camera/*.cpp)
  FILE(REMOVE models/camera/UnitServerIgnore)
  SET(camera_lib ${OPENCV}/lib/libopencv_core.so  ${OPENCV}/lib/libopencv_highgui.so ${OPENCV}/lib/libopencv_imgproc.so ${OPENCV}/lib/libopencv_imgcodecs.so)
  ADD_DEFINITIONS(-DCAMERA)
ELSE()
  FILE(WRITE models/camera/UnitServerIgnore "disabled ${PROJECT_NAME}")
ENDIF()

# SET(sensors_src core/BasicSensor.cpp core/AbstractSensorDriver.cpp core/SensorDriverInterface.cpp core/SCUTA.cpp core/CmdUTAShutter.cpp core/CmdUTADefault.cpp models/plugins.cpp)

SET(BasicSensorClient_src test/sensorsClient.cpp)


IF(BUILD_FORCE_STATIC)
  SET(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--whole-archive -lchaos_common -Wl,--no-whole-archive")
   ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src} ${camera_src})		 

ELSE()
    ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src} ${camera_src})		 

ENDIF()

 TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common common_serial pthread ${camera_lib})

ADD_SUBDIRECTORY(models)

ADD_EXECUTABLE(sensorsServer test/sensorsServer)
ADD_EXECUTABLE(sensorsClient test/sensorsClient)

 TARGET_LINK_LIBRARIES(sensorsServer ${PROJECT_NAME} ${DRIVERS_LIB} chaos_cutoolkit chaos_common common_serial pthread)
 TARGET_LINK_LIBRARIES(sensorsClient ${PROJECT_NAME} chaos_uitoolkit chaos_common ${DRIVERS_LIB} )

 INSTALL_TARGETS(/bin/sensor sensorsServer)
 INSTALL_TARGETS(/bin/sensor sensorsClient)
 INSTALL_TARGETS(/lib ${PROJECT_NAME})

