cmake_minimum_required(VERSION 3.15)
project(esphome_tests)

set(CMAKE_CXX_STANDARD 17)

include (GoogleTest)

enable_testing()

set(TESTS_BASE_DIR ${CMAKE_CURRENT_LIST_DIR})
set(ESPHOME_DIR ${TESTS_BASE_DIR}/../../)

include_directories(${ESPHOME_DIR})
include_directories(${TESTS_BASE_DIR}/include)

function(add_esphome_test TARGET)
	list(SUBLIST "${ARGV}" 1 -1 SOURCES)
	list(INSERT SOURCES 0 ${TARGET}.cpp)
	add_executable(${TARGET} ${SOURCES})
	target_link_libraries(${TARGET} gtest_main gtest)
	gtest_add_tests(TARGET ${TARGET} SOURCES ${SOURCES})
endfunction()

add_esphome_test(pid_controller)

