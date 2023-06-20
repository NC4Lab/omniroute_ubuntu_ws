set(SOURCES slave_import_wizard.c)
add_executable(slave_import_wizard ${SOURCES})
target_link_libraries(slave_import_wizard ethercat_driver)
install(TARGETS slave_import_wizard DESTINATION bin)
