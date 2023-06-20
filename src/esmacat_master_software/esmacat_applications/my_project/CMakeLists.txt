file(GLOB MY_PROJECT_HEADERS *.h)
file(GLOB MY_PROJECT_SOURCES *.cpp *.c)

add_executable(my_project ${MY_PROJECT_SOURCES} ${MY_PROJECT_HEADERS})
target_link_libraries(my_project ethercat_driver esmacat)
install(TARGETS my_project DESTINATION ./bin)
