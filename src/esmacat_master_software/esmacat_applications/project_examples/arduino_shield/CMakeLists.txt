file(GLOB ARDUINO_SHIELD_HEADERS *.h)
file(GLOB ARDUINO_SHIELD_SOURCES *.cpp *.c)

add_executable(arduino_shield ${ARDUINO_SHIELD_SOURCES} ${ARDUINO_SHIELD_HEADERS})
target_link_libraries(arduino_shield ethercat_driver esmacat)
install(TARGETS arduino_shield DESTINATION ./bin)
