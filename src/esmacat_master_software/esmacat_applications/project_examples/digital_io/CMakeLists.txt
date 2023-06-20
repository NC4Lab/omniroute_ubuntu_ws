file(GLOB DIGITAL_IO_SOURCES *.c *.cpp)
file(GLOB DIGITAL_IO_HEADERS *.h)

add_executable(digital_io ${DIGITAL_IO_SOURCES} ${DIGITAL_IO_HEADERS})
target_link_libraries(digital_io ethercat_driver esmacat)
install(TARGETS digital_io DESTINATION ./bin)
