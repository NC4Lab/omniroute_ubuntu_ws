file(GLOB MOTOR_POTENTIOMETER_SOURCES *.c *.cpp)
file(GLOB MOTOR_POTENTIOMETER_HEADERS *.h)

add_executable(motor_pot ${MOTOR_POTENTIOMETER_SOURCES} ${MOTOR_POTENTIOMETER_HEADERS})
target_link_libraries(motor_pot ethercat_driver esmacat)
install(TARGETS motor_pot DESTINATION ./bin)
