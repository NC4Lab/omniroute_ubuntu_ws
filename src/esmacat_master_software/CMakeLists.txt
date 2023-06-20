cmake_minimum_required(VERSION 2.8.4)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")
project(Esmacat C CXX)

#if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  # Default to installing in SOEM source directory
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
#endif()

set(SOEM_INCLUDE_INSTALL_DIR ethercat_driver)
set(ESMACAT_INCLUDE_INSTALL_DIR esmacat)

if(WIN32)
    set(OS "win32")
    include_directories(ethercat_driver/oshw/win32/wpcap/Include)
    link_directories(${CMAKE_SOURCE_DIR}/ethercat_driver/oshw/win32/wpcap/Lib/x64)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /W4")
    set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
    set(OS "linux")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread rt)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
    message("ARCH is ${ARCH}")
    message("BSP is ${BSP}")
    include_directories(ethercat_driver/oshw/${OS}/${ARCH})
    file(GLOB OSHW_EXTRA_SOURCES ethercat_driver/oshw/${OS}/${ARCH}/*.c)
    set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
    set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
endif()

message("OS is ${OS}")

file(GLOB SOEM_SOURCES ethercat_driver/soem/*.c)
file(GLOB UTIL_SOURCES ethercat_driver/utilties/*.c)
file(GLOB OSAL_SOURCES ethercat_driver/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES ethercat_driver/oshw/${OS}/*.c)
file(GLOB ESMACAT_SOURCES esmacat_library/*.cpp esmacat_slave_drivers/drivers/*.cpp esmacat_applications/drivers/*.cpp)

file(GLOB SOEM_HEADERS ethercat_driver/soem/*.h)
file(GLOB UTIL_HEADERS ethercat_driver/utilities/*.h)
file(GLOB OSAL_HEADERS ethercat_driver/osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS ethercat_driver/oshw/${OS}/*.h)
file(GLOB ESMACAT_HEADERS esmacat_library/*.h esmacat_slave_drivers/drivers/*.h esmacat_applications/*.h)

include_directories(ethercat_driver/soem)
include_directories(ethercat_driver/osal)
include_directories(ethercat_driver/utilities)
include_directories(ethercat_driver/osal/${OS})
include_directories(ethercat_driver/oshw/${OS})
include_directories(esmacat_slave_drivers/drivers)
include_directories(esmacat_library)

#Generate the shared library from the sources
add_library(ethercat_driver STATIC
  ${UTIL_SOURCES}
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
target_link_libraries(ethercat_driver ${OS_LIBS})

add_library(esmacat STATIC
    ${ESMACAT_SOURCES}
)
target_link_libraries(esmacat ethercat_driver)

install(TARGETS ethercat_driver esmacat DESTINATION lib)
install(FILES
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  DESTINATION ${SOEM_INCLUDE_INSTALL_DIR})

add_subdirectory(esmacat_slave_drivers)
add_subdirectory(esmacat_applications)
add_subdirectory(ethercat_driver)
