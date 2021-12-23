cmake_minimum_required(VERSION 3.0.0)
project(rsa_test VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(rsa_test main.cpp)
target_link_libraries(rsa_test gmp pthread crypto)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
