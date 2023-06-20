file(GLOB LED_ON_SOURCES *.c *.cpp)
file(GLOB LED_ON_HEADERS *.h)

add_executable(led_on ${LED_ON_SOURCES} ${LED_ON_HEADERS})
target_link_libraries(led_on ethercat_driver esmacat)
install(TARGETS led_on DESTINATION ./bin)
