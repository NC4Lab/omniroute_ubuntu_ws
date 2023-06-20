file(GLOB DEMO_KIT_SOURCES *.c *.cpp)
file(GLOB DEMO_KIT_HEADERS *.h)

add_executable(demo_kit ${DEMO_KIT_SOURCES} ${DEMO_KIT_HEADERS})
target_link_libraries(demo_kit ethercat_driver esmacat)
install(TARGETS demo_kit DESTINATION ./bin)
