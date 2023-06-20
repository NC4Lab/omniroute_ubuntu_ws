file(GLOB SLAVEINFO_HEADERS *.h)
file(GLOB SLAVEINFO_SOURCES *.cpp *.c)

add_executable(slaveinfo ${SLAVEINFO_SOURCES} ${SLAVEINFO_HEADERS})
target_link_libraries(slaveinfo ethercat_driver esmacat)
install(TARGETS slaveinfo DESTINATION ./bin)
