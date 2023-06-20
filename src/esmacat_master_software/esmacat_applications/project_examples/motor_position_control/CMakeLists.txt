file(GLOB MOTOR_POSITION_SOURCES *.c *.cpp)
file(GLOB MOTOR_POSITION__HEADERS *.h)

add_executable(motor_position_control ${MOTOR_POSITION_SOURCES} ${MOTOR_POSITION_HEADERS})
target_link_libraries(motor_position_control ethercat_driver esmacat)
install(TARGETS motor_position_control DESTINATION ./bin)
