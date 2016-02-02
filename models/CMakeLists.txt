cmake_minimum_required(VERSION 2.6)
project(sensor_models)
add_subdirectory(LucidControl)
add_subdirectory(Simulator)
add_subdirectory(ZBSensor)

SET(DRIVERS_LIB ${DRIVERS_LIB} PARENT_SCOPE)
