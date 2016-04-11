cmake_minimum_required(VERSION 3.2.2)
project (sceneSynth)

#includes
include_directories(AnalysisPhase)
include_directories(Debug)
include_directories(Display)
include_directories(Experiments)
include_directories(/usr/local/include)

link_directories ( /usr/local/lib )

# set compiler and linker flags
set (COMPILER_FLAGS "-std=c++11 -m64 -U__STRICT_ANSI__ -fopenmp -O2 -Wno-deprecated ")
set (LINK_FLAGS "-fopenmp")

if (PROFILING)
    set (COMPILER_FLAGS "${COMPILER_FLAGS} -pg")
    set (LINK_FLAGS "${LINK_FLAGS} -pg")
endif()
if (SYMBOLS)
    set (COMPILER_FLAGS "${COMPILER_FLAGS} -g -ggdb")
    add_definitions ("-DSYMBOLS")
endif()
if (EXPS)
    add_definitions("-DEXP")
endif()
if (PERF)
    add_definitions("-DPERFTIMER")
endif ()
if (DEBUG)
    add_definitions ("-DDEBUG")
endif ()


set ( LIBRARY_LINKS_LST "")
set ( DLLS_TO_COPY "")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)

# LIBRARIES
set ( LIB_DIR ${CMAKE_SOURCE_DIR}/../libs)

    ## Assimp
    set ( ASSIMP_DIR ${LIB_DIR}/assimp)
    include_directories ( ${ASSIMP_DIR}/include )
    link_directories( ${ASSIMP_DIR}/lib )
    if (CYGWIN)
        list ( APPEND LIBRARY_LINKS_LST  assimp.dll )
        file (GLOB DLL ${ASSIMP_DIR}/bin/cygassimp-3.dll)
        list ( APPEND DLLS_TO_COPY ${DLL})
    else ()
        list ( APPEND LIBRARY_LINKS_LST assimp )
    endif ()

    ## EIGEN
    include_directories ( ${LIB_DIR}/eigen )

    ## IGL
    include_directories ( ${LIB_DIR}/libigl/include )

    ## Qhull
    #set ( QHULL_DIR /usr/local)
    set ( QHULL_DIR ${LIB_DIR}/qhull )
    include_directories ( ${QHULL_DIR}/include/libqhullcpp )
    include_directories ( ${QHULL_DIR}/include/libqhull_r )
    link_directories( ${QHULL_DIR}/lib )
    if (CYGWIN)
        list ( APPEND LIBRARY_LINKS_LST qhullcpp qhull_r.dll )
        file (GLOB DLL ${QHULL_DIR}/bin/*qhull*.dll)
        list ( APPEND DLLS_TO_COPY ${DLL})
    else ()
        list (APPEND LIBRARY_LINKS_LST qhullcpp qhull_r)
    endif ()

    ## CGAL
    #set ( CGAL_DIR ${LIB_DIR}/CGAL-4.7)
    #set ( CGAL_DIR /usr/local)
    set ( CGAL_DIR ${LIB_DIR}/cgal )
    link_directories ( ${CGAL_DIR}/lib )
    if (CYGWIN)
        list ( APPEND LIBRARY_LINKS_LST CGAL.dll CGAL_Core.dll)
        file (GLOB DLL ${CGAL_DIR}/bin/cygCGAL*)
        list ( APPEND DLLS_TO_COPY ${DLL})
    else ()
        list ( APPEND LIBRARY_LINKS_LST CGAL CGAL_Core )
    endif ()

    ## GnuPlot-IOSTREAM
    set ( GPIOS_DIR ${LIB_DIR}/gnuplot-iostream)
    include_directories ( ${GPIOS_DIR} )
    list ( APPEND LIBRARY_LINKS_LST mpfr gmp boost_thread boost_iostreams boost_system boost_filesystem util )
    
    ## TBB (For CGAL)
    #set ( TBB_DIR ${LIB_DIR}/tbb42_20140601oss_src/tbb42_20140601oss)
    #include_directories (${TBB_DIR}/include)
    #link_directories ( ${TBB_DIR}/build/cygwin_intel64_gcc_cc4.9.3_newlib_release )
    #add_definitions( -DNOMINMAX -DCGAL_LINKED_WITH_TBB )
    #if (CYGWIN)
        #list ( APPEND LIBRARY_LINKS_LST tbb.dll tbbmalloc.dll pthread)
        #file (GLOB DLL ${TBB_DIR}/build/cygwin_intel64_gcc_cc4.9.3_newlib_release/*tbb*.dll)
        #list ( APPEND DLLS_TO_COPY ${DLL})
    #endif ()
    #link_directories (${TBB_DIR}/lib)
    #list ( APPEND LIBRARY_LINKS_LST qhullcpp qhull_r.dll )
    
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

set ( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${COMPILER_FLAGS}" )
set ( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${LINK_FLAGS}" )

add_executable(sceneSynth ${SOURCES})
target_link_libraries(sceneSynth ${LIBRARY_LINKS_LST})
