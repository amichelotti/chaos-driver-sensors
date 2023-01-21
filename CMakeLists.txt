cmake_minimum_required(VERSION 2.9)
project(chaos_driver_sensors)
SET(camera_src )
SET(camera_lib )
find_package(chaos MODULE REQUIRED)
INCLUDE_DIRECTORIES(../../ ${chaos_INCLUDE_DIRS} ${CMAKE_INSTALL_PREFIX}/include/driver/sensors/core/aravis)
add_definitions(-Wno-ignored-attributes -Wno-deprecated-declarations)
FILE(GLOB sensors_src core/BasicSensorProperty.cpp    core/CmdUTADefault.cpp   core/SensorDriverInterface.cpp core/AbstractSensorDriver.cpp   core/CmdUTAShutter.cpp    
core/BasicSensor.cpp  core/SCUTA.cpp)
IF(OPENCV)
  MESSAGE("Enable Camera support")
  include_directories(../../ ${chaos_INCLUDE_DIRS} ${CMAKE_INSTALL_PREFIX}/include ${OPENCV}/include )

  link_directories(${OPENCV}/lib ${CMAKE_INSTALL_PREFIX}/lib)
  ADD_DEFINITIONS(-DCAMERA -DOPENCV)

  FILE(GLOB camera_src models/camera/Encoder.cpp  models/camera/RTCameraFilter.cpp models/camera/RTCameraBase.cpp models/camera/CameraDriverInterface.cpp  core/AbstractCameraDriver.cpp models/camera/CameraDriverBridge.cpp models/camera/RTCameraProcess.cpp)

 IF(CERN_ROOT)
   
  FILE(GLOB camera_src ${camera_src} core/rootGaussianImage2dFit.cpp)
ENDIF()
  SET(camera_lib   opencv_world)
  # SET(camera_lib   opencv_highgui opencv_imgproc opencv_videoio opencv_imgcodecs opencv_core)
  FILE(REMOVE models/camera/UnitServerIgnore)
ELSE()
  link_directories(${CMAKE_INSTALL_PREFIX}/lib)
  FILE(WRITE models/camera/UnitServerIgnore "disabled ${PROJECT_NAME}")
ENDIF()


SET(BasicSensorClient_src test/sensorsClient.cpp)


IF(BUILD_FORCE_STATIC)
  SET(CMAKE_EXE_LINKER_FLAGS "-static -Wl,--whole-archive -lchaos_common -lpthread -Wl,--no-whole-archive")
  ADD_LIBRARY(${PROJECT_NAME} STATIC ${sensors_src} ${camera_src})		 

ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${sensors_src} ${camera_src})		 

ENDIF()
if(CERN_ROOT)
  SET(CERNLIBS Core Hist)
  
  target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
ELSE()
 SET(CERNLIBS )
ENDIF()
 
TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit  chaos_common common_serial pthread ${camera_lib}  ${CERNLIBS} common_misc_data)

ADD_SUBDIRECTORY(models)

ADD_EXECUTABLE(sensorsServer test/sensorsServer)
# ADD_EXECUTABLE(sensorsClient test/sensorsClient)

 TARGET_LINK_LIBRARIES(sensorsServer ${PROJECT_NAME} ${DRIVERS_LIB} chaos_cutoolkit chaos_common common_serial pthread)
 # TARGET_LINK_LIBRARIES(sensorsClient ${PROJECT_NAME} chaos_common pthread ${DRIVERS_LIB})

 FILE(GLOB files "core/*.h")
INSTALL(FILES ${files} DESTINATION include/driver/sensors/core)

 #INSTALL_TARGETS(/bin/sensor sensorsServer)
 INSTALL(TARGETS sensorsServer
        DESTINATION "bin"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
 #INSTALL_TARGETS(/bin/sensor sensorsClient)
# INSTALL(TARGETS sensorsClient
#        DESTINATION "bin"
#        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
 #INSTALL_TARGETS(/lib ${PROJECT_NAME})
 INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

