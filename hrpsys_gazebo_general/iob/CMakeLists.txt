add_definitions(-DHRPSYS_PACKAGE_VERSION=\"\\"${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}\\"\")

# hrpIo
add_library(hrpIo_gazebo SHARED iob.cpp)

if (NOT APPLE AND NOT QNXNTO)
   target_link_libraries(hrpIo_gazebo rt)
endif()

target_link_libraries(hrpIo_gazebo roscpp rostime)

# RobotHardware
set(comp_source  ${ROBOTHARDWARE_SOURCE}/robot.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardware.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardwareService_impl.cpp)
set(libs hrpIo_gazebo hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)

add_library(RobotHardware_gazebo SHARED ${comp_source})

target_link_libraries(RobotHardware_gazebo  ${libs})
set_target_properties(RobotHardware_gazebo PROPERTIES PREFIX "")

add_executable(RobotHardwareComp_gazebo ${ROBOTHARDWARE_SOURCE}/RobotHardwareComp.cpp ${comp_source})
target_link_libraries(RobotHardwareComp_gazebo ${libs} ${omniorb_LIBRARIES} ${omnidynamic_LIBRARIES} RTC coil)

install(TARGETS RobotHardware_gazebo RobotHardwareComp_gazebo hrpIo_gazebo
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)
