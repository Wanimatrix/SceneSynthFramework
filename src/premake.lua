#!lua

-- Solution
solution "SceneSynthesisFramework"
   configurations { "Debug", "Release" }

   -- Project
   project "SceneSynth"
      kind "ConsoleApp"
      language "C++"
      buildoptions { "-std=c++11" }
      -- includedirs ("/usr/local/include")
      files { "**.h", "**.cpp" }

      excludes {"AnalysisPhase/Analysis*","AnalysisPhase/IBS*","AnalysisPhase/Constraint*","Mesh*","Vertex*","Face*"}

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
         qhulldir = libdir .. "/qhull"
         includedirs (qhulldir .. "/src/libqhullcpp")
         includedirs (qhulldir .. "/src")
         libdirs (qhulldir .. "/lib")
         links ("qhullcpp")

         -- CGAL -- 
         links ("CGAL")

      configuration "windows"
         links ("assimp.dll")
         postbuildcommands {"cp " .. assimpdir .. "/bin/cygassimp-3.dll ../bin", "make"}

      configuration "macosx"
         links ("assimp")
         links ("CGAL")

      configuration "Debug"
         defines { "DEBUG" }
         flags { "Symbols" }

      configuration "Release"
         defines { "NDEBUG" }
         flags { "Optimize" }