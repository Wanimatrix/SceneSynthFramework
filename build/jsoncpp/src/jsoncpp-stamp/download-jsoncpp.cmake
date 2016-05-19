message(STATUS "downloading...
     src='https://github.com/open-source-parsers/jsoncpp/archive/0.10.6.zip'
     dst='/home/wouter/Documenten/Doctoraat/Implementation/SceneSynthesisFramework2/build/jsoncpp/src/0.10.6.zip'
     timeout='none'")




file(DOWNLOAD
  "https://github.com/open-source-parsers/jsoncpp/archive/0.10.6.zip"
  "/home/wouter/Documenten/Doctoraat/Implementation/SceneSynthesisFramework2/build/jsoncpp/src/0.10.6.zip"
  SHOW_PROGRESS
  # no TIMEOUT
  STATUS status
  LOG log)

list(GET status 0 status_code)
list(GET status 1 status_string)

if(NOT status_code EQUAL 0)
  message(FATAL_ERROR "error: downloading 'https://github.com/open-source-parsers/jsoncpp/archive/0.10.6.zip' failed
  status_code: ${status_code}
  status_string: ${status_string}
  log: ${log}
")
endif()

message(STATUS "downloading... done")
