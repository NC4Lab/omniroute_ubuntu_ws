file(GLOB SALES_KIT_SOURCES *.c *.cpp)
file(GLOB SALES_KIT_HEADERS *.h)

add_executable(sales_kit ${SALES_KIT_SOURCES} ${SALES_KIT_HEADERS})
target_link_libraries(sales_kit ethercat_driver esmacat)
install(TARGETS sales_kit DESTINATION ./bin)
