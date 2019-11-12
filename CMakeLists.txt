#--------------------------------------------------------
# The CMakeLists.txt for: pMavlink
# Author(s): David Battle, Mission Systems Pty Ltd
# Based on pMavlinkCoverter by Muthukumaran Chandrasekaran
#--------------------------------------------------------
cmake_minimum_required(VERSION 3.5)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{BIN_HOME})

# This allows us to use the header-only JSON library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC" )

# Switching on C++11 has unleashed deprecation warnings for MOOS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

find_package(MOOS 10.0)
find_package(MOOSGeodesy)

#set(CMAKE_PREFIX_PATH $ENV{JSON_HOME};$ENV{JSON_HOME}/build)
find_package(nlohmann_json 3.5.0 CONFIG REQUIRED)

set(SRC
  Mavlink.cpp
  Mavlink_Info.cpp
  translation.cpp
  main.cpp
)

add_executable(pMavlink ${SRC})

target_include_directories(pMavlink PUBLIC
    $ENV{MOOSIVP_SOURCE_TREE_BASE}/include/ivp
    $ENV{MAVLINK_HOME}/include/common
    ${MOOS_INCLUDE_DIRS}
    ${MOOSGeodesy_INCLUDE_DIRS}
)

target_link_libraries(pMavlink
   $ENV{MOOSIVP_SOURCE_TREE_BASE}/lib/libmbutil.a
   ${MOOSGeodesy_LIBRARIES}
   ${MOOS_LIBRARIES}
   nlohmann_json::nlohmann_json
   boost_system
   m
   pthread
)
