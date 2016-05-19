cmake_minimum_required(VERSION 3.2.2)
project (sceneSynth)

## GTEST
    # We need thread support
    find_package(Threads REQUIRED)

    # Enable ExternalProject CMake module
    include(ExternalProject)

    # Download and install GoogleTest
    ExternalProject_Add(
        gtest
        URL https://github.com/google/googletest/archive/release-1.7.0.zip
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
        CMAKE_ARGS -DCMAKE_CXX_COMPILER=g++-4.8
        # Disable install step
        INSTALL_COMMAND ""
    )

    # Create a libgtest target to be used as a dependency by test programs
    add_library(libgtest IMPORTED STATIC GLOBAL)
    add_dependencies(libgtest gtest)

    # Set gtest properties
    ExternalProject_Get_Property(gtest source_dir binary_dir)
    set_target_properties(libgtest PROPERTIES
        "IMPORTED_LOCATION" "${binary_dir}/libgtest.a"
        "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
    #    "INTERFACE_INCLUDE_DIRECTORIES" "${source_dir}/include"
    )
    # I couldn't make it work with INTERFACE_INCLUDE_DIRECTORIES
    include_directories("${source_dir}/include")

## JSONCPP

    # Download and install GoogleTest
    ExternalProject_Add(
        jsoncpp
        URL https://github.com/open-source-parsers/jsoncpp/archive/0.10.6.zip
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/jsoncpp
        CMAKE_ARGS -DCMAKE_CXX_COMPILER=g++-4.8
        # Disable install step
        INSTALL_COMMAND ""
    )

    # Create a libgtest target to be used as a dependency by test programs
    add_library(libjsoncpp IMPORTED STATIC GLOBAL)
    add_dependencies(libjsoncpp jsoncpp)

    # Set jsoncpp properties
    ExternalProject_Get_Property(jsoncpp source_dir binary_dir)
    set_target_properties(libjsoncpp PROPERTIES
        "IMPORTED_LOCATION" "${binary_dir}/libjsoncpp.a"
        "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
    #    "INTERFACE_INCLUDE_DIRECTORIES" "${source_dir}/include"
    )
    # I couldn't make it work with INTERFACE_INCLUDE_DIRECTORIES
    include_directories("${source_dir}/include")

#includes
include_directories(include)
include_directories(/usr/local/include)
link_directories ( /usr/local/lib )

# Sources
file(GLOB_RECURSE MAIN_SOURCES "${CMAKE_SOURCE_DIR}/newSrc/main/*.cpp")
file(GLOB_RECURSE TESTABLE_SOURCES "${CMAKE_SOURCE_DIR}/newSrc/main/*/*.cpp")
file(GLOB_RECURSE TEST_SOURCES "${CMAKE_SOURCE_DIR}/newSrc/test/*.cpp")

# set compiler and linker flags
set ( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -m64 -U__STRICT_ANSI__ -fopenmp -O0 -Wno-deprecated" )
set ( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -fopenmp" )

set ( LIBRARY_LINKS_LST "libjsoncpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)


# set ( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${COMPILER_FLAGS}" )
# set ( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${LINK_FLAGS}" )

add_executable(sceneSynth ${MAIN_SOURCES})
target_link_libraries(sceneSynth ${LIBRARY_LINKS_LST})

add_library(codeToTest ${TESTABLE_SOURCES})
add_executable(unit-test ${TEST_SOURCES})
target_link_libraries(unit-test ${LIBRARY_LINKS_LST} libgtest codeToTest)
