file(GLOB MOTOR_DRIVER_SOURCES *.c *.cpp)
file(GLOB MOTOR_DRIVER_HEADERS *.h)

add_executable(motor_driver ${MOTOR_DRIVER_SOURCES} ${MOTOR_DRIVER_HEADERS})
target_link_libraries(motor_driver ethercat_driver esmacat)
install(TARGETS motor_driver DESTINATION ./bin)
