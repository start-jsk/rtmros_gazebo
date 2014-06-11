add_definitions(-DHRPSYS_PACKAGE_VERSION=\"\\"${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}\\"\")

# hrpIo
add_library(hrpIo_atlas SHARED iob.cpp)

if (NOT APPLE AND NOT QNXNTO)
   target_link_libraries(hrpIo_atlas rt)
endif()

target_link_libraries(hrpIo_atlas roscpp rostime)

# RobotHardware
set(comp_source  ${ROBOTHARDWARE_SOURCE}/robot.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardware.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardwareService_impl.cpp)
set(libs hrpIo hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)

add_library(RobotHardware_atlas SHARED ${comp_source})

target_link_libraries(RobotHardware_atlas  ${libs})
set_target_properties(RobotHardware_atlas PROPERTIES PREFIX "")
set_target_properties(RobotHardware_atlas PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(RobotHardware_atlas PROPERTIES OUTPUT_NAME RobotHardware)

add_executable(RobotHardwareComp_atlas ${ROBOTHARDWARE_SOURCE}/RobotHardwareComp.cpp ${comp_source})
target_link_libraries(RobotHardwareComp_atlas ${libs} ${omniorb_LIBRARIES} ${omnidynamic_LIBRARIES} RTC coil)
set_target_properties(RobotHardwareComp_atlas PROPERTIES OUTPUT_NAME RobotHardwareComp)

install(TARGETS RobotHardwareComp_atlas RobotHardware_atlas hrpIo_atlas
  RUNTIME DESTINATION bin CONFIGURATIONS Release Debug
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug
)



