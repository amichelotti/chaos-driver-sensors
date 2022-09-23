cmake_minimum_required(VERSION 2.9)
project(sensor_models)
add_subdirectory(LucidControl)
add_subdirectory(Simulator)
add_subdirectory(ZBSensor)
add_subdirectory(LeyboldCenterOne)

IF(OPENCV)
MESSAGE("Enable Camera support")
ADD_DEFINITIONS(-DCAMERA -DOPENCV)

FILE(REMOVE camera/UnitServerIgnore)
add_subdirectory(camera)
ELSE()
FILE(WRITE camera/UnitServerIgnore "disabled ${PROJECT_NAME}")
ENDIF()
SET(DRIVERS_LIB ${DRIVERS_LIB} PARENT_SCOPE)

