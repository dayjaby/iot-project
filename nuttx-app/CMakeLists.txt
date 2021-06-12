cmake_minimum_required(VERSION 3.2...3.15)

project(DW1000
        VERSION 1.0
	DESCRIPTION "DW1000 C++ Nuttx"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)

set(NUTTX_PATH "${CMAKE_SOURCE_DIR}/nuttx-export-10.1.0-RC1")

include(cmake/pozyx_f401rc.cmake)

set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -fno-builtin -Wall -Wshadow -Wundef -fno-strict-aliasing -fno-strength-reduce -fomit-frame-pointer -Os")
set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -D_DEBUG -D_LIBCPP_BUILD_STATIC -D_LIBCPP_NO_EXCEPTIONS ")
set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -fno-exceptions -fcheck-new -fno-rtti -pedantic ")
set(AC_COMMON_FLAGS "${AC_COMMON_FLAGS} -nostdinc++")

set(AC_DEFINES "${AC_DEFINES} -DCONFIG_WCHAR_BUILTIN")

include_directories(
        include
        ${NUTTX_PATH}/include
        ${NUTTX_PATH}/include/cxx
        ${NUTTX_PATH}/arch/chip
)

set(EXE_NAME dw1000)

set(CMAKE_CXX_FLAGS     "${AC_HW_FLAGS} ${AC_DEFINES} ${AC_COMMON_FLAGS} ${AC_CXX_EXTRA_FLAGS}")
if (PARAM_DEBUG)
    set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -g")
endif()

set(CMAKE_SKIP_RPATH ON)
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_LINKER} ${AC_LINKER_FLAGS} -o ${EXE_NAME}.elf <OBJECTS> <LINK_LIBRARIES>")

set(BUILD_SHARED_LIBS OFF)


set(SOURCE_FILES
)

link_directories(${EXE_NAME} ${NUTTX_PATH}/libs)

add_executable(${EXE_NAME} ${SOURCE_FILES} src/main.cpp ${HEADER_FILES})

add_custom_command(
        TARGET ${EXE_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ARGS -S -O binary ${CMAKE_BINARY_DIR}/${EXE_NAME}.elf ${CMAKE_BINARY_DIR}/${EXE_NAME}.bin
)

target_link_libraries(${EXE_NAME} --start-group)

target_link_libraries(${EXE_NAME} sched)
target_link_libraries(${EXE_NAME} drivers)
target_link_libraries(${EXE_NAME} boards)
target_link_libraries(${EXE_NAME} c)
target_link_libraries(${EXE_NAME} mm)
target_link_libraries(${EXE_NAME} arch)
target_link_libraries(${EXE_NAME} xx)
target_link_libraries(${EXE_NAME} apps)
target_link_libraries(${EXE_NAME} fs)
target_link_libraries(${EXE_NAME} binfmt)
target_link_libraries(${EXE_NAME} board)
target_link_libraries(${EXE_NAME} gcc)
# target_link_libraries(${EXE_NAME} supc++)

target_link_libraries(${EXE_NAME} --end-group)
