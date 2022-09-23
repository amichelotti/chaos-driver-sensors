cmake_minimum_required(VERSION 3.0)
project(cameraUS)
IF(OPENCV)
FILE(REMOVE UnitServerIgnore)

FILE(REMOVE ShapeSim/UnitServerIgnore)
FILE(REMOVE CameraShared/UnitServerIgnore)
add_subdirectory(ShapeSim)
add_subdirectory(CameraShared)

ELSE()
FILE(WRITE ShapeSim/UnitServerIgnore "disabled ${PROJECT_NAME}")
FILE(WRITE CameraShred/UnitServerIgnore "disabled ${PROJECT_NAME}")

ENDIF()
ADD_DEFINITIONS(-DCAMERA -DOPENCV)


IF (BASLER)
  MESSAGE(STATUS "Adding Basler Support")
  add_definitions(-DBASLER_DRIVER)

  add_subdirectory(Basler)
  FILE(REMOVE Basler/UnitServerIgnore)
ELSE()
  FILE(WRITE Basler/UnitServerIgnore "disabled ${PROJECT_NAME}")
ENDIF()

IF(ARAVIS)
MESSAGE(STATUS "Adding Aravis Support")
  add_definitions(-UCERN_ROOT)
  INCLUDE_DIRECTORIES(${ARAVIS}/include/aravis-0.8 /usr/include/glib-2.0 /usr/lib/x86_64-linux-gnu/glib-2.0/include)

  add_definitions(-DARAVIS_DRIVER)
  add_subdirectory(aravis)
FILE(REMOVE aravis/UnitServerIgnore)
ELSE()
FILE(WRITE aravis/UnitServerIgnore "disabled ${PROJECT_NAME}")
ENDIF()

IF (IDS)
  MESSAGE(STATUS "Adding IDS ${IDS} Support")
  add_definitions(-DIDS_DRIVER)

  add_subdirectory(IDS)
  FILE(REMOVE IDS/UnitServerIgnore)
ELSE()
  FILE(WRITE IDS/UnitServerIgnore "disabled ${PROJECT_NAME}")
ENDIF()
IF (ENABLE_MEMCACHE)
  MESSAGE(STATUS "Adding MC Support")
  add_definitions(-DMC_CAMERA_DRIVER)

  add_subdirectory(MemCache)
  FILE(REMOVE MemCache/UnitServerIgnore)
ELSE()
  FILE(WRITE MemCache/UnitServerIgnore "disabled ${PROJECT_NAME}")
ENDIF()
SET(DRIVERS_LIB ${DRIVERS_LIB} PARENT_SCOPE)

FILE(GLOB files "*.h")
INSTALL(FILES ${files} DESTINATION include/driver/sensors/models/camera)


SET(sensors_src cameraUS.cpp Encoder.cpp RTCameraBase.cpp ../../core/AbstractCameraDriver.cpp CameraDriverBridge.cpp CameraDriverInterface.cpp)

add_executable(${PROJECT_NAME} ${sensors_src})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} chaos_cutoolkit chaos_common  ${DRIVERS_LIB})

INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "bin"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
