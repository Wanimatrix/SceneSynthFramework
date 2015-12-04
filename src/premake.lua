#!lua

-- Solution
solution "SceneSynthesisFramework"
   configurations { "Debug", "Profiler", "Release" }

   -- Project
   project "SceneSynth"
      kind "ConsoleApp"
      language "C++"
      buildoptions { "-std=c++11 -fopenmp" }
      linkoptions { "-fopenmp" }
      includedirs ("/usr/local/include")
      libdirs ("/usr/local/lib")
      files { "**.h", "**.cpp" }

      excludes {"AnalysisPhase/IBS.*","AnalysisPhase/IBSConstraint*","Vertex*","Face*"}

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
         --links ("qhullstatic_r")

         -- CGAL --
         cgaldir = libdir .. "/CGAL-4.7"

      configuration "windows"
         links ("assimp.dll")
         libdirs (cgaldir .. "/lib")
         links ("CGAL.dll")
         links ("CGAL_Core.dll")

         links ("qhull_r.dll")
         -- links ("qhull.dll")
         --links ("qhull_p.dll")
         postbuildcommands {"cp " .. assimpdir .. "/bin/cygassimp-3.dll ../bin"}
         postbuildcommands {"cp " .. qhulldir .. "/bin/*qhull*.dll ../bin"}
         postbuildcommands {"cp " .. cgaldir .. "/bin/cygCGAL* ../bin"}

      configuration "macosx"
         links ("assimp")
         links ("CGAL")

      configuration "Debug"
         defines { "DEBUG" }
         flags { "Symbols" }

      configuration "Profiler"
         buildoptions {"-pg"}
         linkoptions {"-pg -static"}
         defines { "DEBUG" }
         flags { "Symbols" }

      configuration "Release"
         defines { "NDEBUG" }
         flags { "Optimize" }