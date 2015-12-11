cmake_minimum_required(VERSION 3.3.0)
project (sceneSynth)

#includes
include_directories(AnalysisPhase)
include_directories(Debug)
include_directories(Display)
include_directories(Experiments)
include_directories(/usr/local/include)

link_directories ( /usr/local/lib )

# set compiler and linker flags
set (COMPILER_FLAGS "-std=c++11 -U__STRICT_ANSI__ -fopenmp -O3")
set (LINK_FLAGS "-fopenmp")
set ( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${COMPILER_FLAGS}" )
set ( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${LINK_FLAGS}" )

# set preprocessor definitions
add_definitions ("-DDEBUG")
if (EXPS)
    add_definitions("-DEXP")
endif()
if (PERF)
    add_definitions("-DPERFTIMER")
endif()

#[[set ( LIBRARY_LINKS_LST "")
set ( DLLS_TO_COPY "")]]

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

# LIBRARIES
set ( LIB_DIR ${CMAKE_SOURCE_DIR}/../libs)

    ## Assimp
    set ( ASSIMP_DIR ${LIB_DIR}/assimp_3.2)
    include_directories ( ${ASSIMP_DIR}/include )
    link_directories( ${ASSIMP_DIR}/lib )
    list ( APPEND LIBRARY_LINKS_LST  assimp.dll )
    file (GLOB DLL ${ASSIMP_DIR}/bin/cygassimp-3.dll)
    list ( APPEND DLLS_TO_COPY ${DLL})

    ## EIGEN
    include_directories ( ${LIB_DIR}/eigen )

    ## Qhull
    set ( QHULL_DIR /usr/local)
    include_directories ( ${QHULL_DIR}/include/libqhullcpp )
    include_directories ( ${QHULL_DIR}/include/libqhull_r )
    link_directories( ${QHULL_DIR}/lib )
    list ( APPEND LIBRARY_LINKS_LST qhullcpp qhull_r.dll )
    file (GLOB DLL ${QHULL_DIR}/bin/*qhull*.dll)
    list ( APPEND DLLS_TO_COPY ${DLL})

    ## CGAL
    set ( CGAL_DIR ${LIB_DIR}/CGAL-4.7)
    link_directories ( ${CGAL_DIR}/lib )
    list ( APPEND LIBRARY_LINKS_LST CGAL.dll CGAL_Core.dll)
    file (GLOB DLL ${CGAL_DIR}/bin/cygCGAL*)
    list ( APPEND DLLS_TO_COPY ${DLL})

    ## GnuPlot-IOSTREAM
    set ( GPIOS_DIR ${LIB_DIR}/gnuplot-iostream)
    include_directories ( ${GPIOS_DIR} )
    list ( APPEND LIBRARY_LINKS_LST boost_iostreams boost_system boost_filesystem util )
    
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(sceneSynth ${SOURCES})
target_link_libraries(sceneSynth ${LIBRARY_LINKS_LST})