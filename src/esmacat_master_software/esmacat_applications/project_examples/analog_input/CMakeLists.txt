file(GLOB ANALOG_INPUT_SOURCES *.c *.cpp)
file(GLOB ANALOG_INPUT_HEADERS *.h)

add_executable(analog_input ${ANALOG_INPUT_SOURCES} ${ANALOG_INPUT_HEADERS})
target_link_libraries(analog_input ethercat_driver esmacat)
install(TARGETS analog_input DESTINATION ./bin)
