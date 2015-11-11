cmake_minimum_required(VERSION 3.2)

FIND_PACKAGE( OpenCV 2.4 REQUIRED )


set(LIB_SRC
    contour.cpp
    rect.cpp
    tools.cpp
)

set(CDP_SRC
    contour.cpp
    rect.cpp
    tools.cpp
    cdp.cpp
    cdp_main.cpp
)

set(CDP_BIN
    cdp
)


include_directories("${PROJECT_SOURCE_DIR}")

#Sends the -Wall flag to the gcc compiler
add_definitions(-Wall)

# Set Flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

# Executable binary
add_executable("${CDP_BIN}" "${CDP_SRC}")

target_link_libraries("${CDP_BIN}" "${OpenCV_LIBS}")
