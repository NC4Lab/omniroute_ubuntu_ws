file(GLOB LOADCELL_INTERFACE_SOURCES *.c *.cpp)
file(GLOB LOADCELL_INTERFACE_HEADERS *.h)

add_executable(loadcell_interface ${LOADCELL_INTERFACE_SOURCES} ${LOADCELL_INTERFACE_HEADERS})
target_link_libraries(loadcell_interface ethercat_driver esmacat)
install(TARGETS loadcell_interface DESTINATION ./bin)
