cmake_minimum_required(VERSION 3.12)
project(BMP180_Test C)

# This CMakeLists.txt is for testing purposes only.
add_compile_definitions(TESTING)

# Collect all source files
set(BMP180_SOURCES
    BMP180.c)
add_executable(BMP180_Test ${BMP180_SOURCES} BMP180_Test.c)

# Setup testing
enable_testing()
include(CTest)

add_test(
    NAME BMP180_UnitTest
    COMMAND BMP180_Test
)
