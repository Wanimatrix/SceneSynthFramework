#!lua

-- Solution
solution "SceneSynthesisFramework"
   configurations { "DebugNoSymbols", "DebugSymbols", "Timer", "Profiler", "Release" }

   -- Project
   project "SceneSynth"
      kind "ConsoleApp"
      language "C++"
      buildoptions { "-std=c++11 -U__STRICT_ANSI__ -fopenmp" }
      linkoptions { "-fopenmp" }
      includedirs ("/usr/local/include")
      libdirs ("/usr/local/lib")
      files { "**.h", "**.cpp" }

      excludes {"AnalysisPhase/IBSConstraint*","Vertex*","Face*"}

      -- newoption {
      --    trigger     = "exps",
      --    value = "experimentNumbers (eg. 1,2,6,10,15)",
      --    description = "Run experiments instead of original main function."
      -- }

      -- if _OPTIONS["exps"] then
      --    defines { "EXP=" .. _OPTIONS["exps"] }
      -- end

      -- exps = ""
      --defines { "EXP=$(exps)" }
      --prelinkcommands { "ifdef exps\n DEFINES += -DEXP=$(exps)\n endif" }

      -- Output
      targetdir ("../bin")
      targetname ("sceneSynth")

      ---------------
      -- LIBRARIES --
      ---------------
      libdir = "../../libs"

         -- Assimp 3.2 --
         assimpdir = libdir .. "/assimp_3.2"
         includedirs (assimpdir .. "/include")
         libdirs (assimpdir .. "/lib")

         -- EIGEN --
         includedirs (libdir .. "/eigen")

         -- QHull --
         qhulldir = "/usr/local"
         -- qhulldir = libdir .. "/qhull"
         includedirs (qhulldir .. "/include/libqhullcpp")
         includedirs (qhulldir .. "/include/libqhull_r")
         -- includedirs (qhulldir .. "/src")
         libdirs (qhulldir .. "/lib")
         links ("qhullcpp")
         -- links ("qhull_r")

         -- CGAL --
         cgaldir = libdir .. "/CGAL-4.7"

         -- GNUPLOT-IOSTREAM --
         gnuplotiosdir = libdir .. "/gnuplot-iostream"
         includedirs (gnuplotiosdir)
         links ("boost_iostreams")
         links ("boost_system")
         -- links ("boost_device")
         links ("boost_filesystem")
         links ("util")

      configuration "windows"
         links ("assimp.dll")
         libdirs (cgaldir .. "/lib")
         links ("CGAL.dll")
         links ("CGAL_Core.dll")

         -- qhulldir = "/usr/local"
         -- includedirs (qhulldir .. "/include/libqhullcpp")
         -- includedirs (qhulldir .. "/include/libqhull_r")
         -- libdirs (qhulldir .. "/lib")
         -- links ("qhullcpp")
         links ("qhull_r.dll")
         postbuildcommands {"cp " .. assimpdir .. "/bin/cygassimp-3.dll ../bin"}
         postbuildcommands {"cp " .. qhulldir .. "/bin/*qhull*.dll ../bin"}
         postbuildcommands {"cp " .. cgaldir .. "/bin/cygCGAL* ../bin"}

      configuration "macosx"
         links ("assimp")
         links ("CGAL")
         -- qhulldir = libdir .. "/qhull"
         -- includedirs (qhulldir .. "/src/libqhullcpp")
         -- includedirs (qhulldir .. "/src")
         -- libdirs (qhulldir .. "/lib")
         -- links ("qhullcpp")
         links ("qhull_r")

      configuration "DebugSymbols"
         defines { "DEBUG", "SYMBOLS"}
         flags { "Symbols" }

      configuration {"not DebugSymbols"}
         defines { "NOSYMBOLS" }
         flags { "Optimize" }

      configuration {"not Release"}
         defines { "DEBUG" }

      configuration "Profiler"
         buildoptions {"-pg"}
         linkoptions {"-pg -static"}
         defines { "PROFILER" }

      configuration "Timer"
         defines { "PERFTIMER" }

      configuration "Release"
         defines { "NDEBUG" }